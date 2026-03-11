-- ============================================================
-- Path_Planner.lua
-- ------------------------------------------------------------
-- Purpose:
--   - Compute a collision-free 3D path (x,y,z) using OMPL (simOMPL)
--   - Use a simple "proxy" shape (omplRobot) as the collision geometry
--   - Draw the resulting path in cyan
--   - Store the path in /Path custom data so a follower script can move
--     the Quadcopter target along it
--
-- Scene requirements (names must match):
--   /Path                 (dummy container)
--   /Path/start           (dummy start marker)
--   /Path/goal            (dummy goal marker)
--   /Path/omplRobot       (shape proxy, Collidable ON, Dynamic/Respondable OFF)
--
-- Data exchange (written into /Path custom data blocks):
--   OMPL_XYZ_PATH : packed float array [x1 y1 z1 x2 y2 z2 ...]
--   OMPL_READY    : packed int32 {0/1}  (handshake: 0 = not ready, 1 = ready)
--   OMPL_PATH_ID  : packed int32 {k}    (increments each time a new path is created)
--
-- Notes:
--   - This example uses sim.handle_all for collision pairs. For this to work
--     as intended, only obstacles + omplRobot should be Collidable.
--   - Start/goal must not be in collision (e.g., not on the floor) or OMPL
--     will reject the start state as invalid.
-- ============================================================

-- Load CoppeliaSim APIs:
sim=require 'sim'
simOMPL=require 'simOMPL'


-- ------------------------------------------------------------
-- Convert a dummy pose into a pose3d state for OMPL.
-- OMPL pose3d uses: [x y z qx qy qz qw]
-- Here the orientation is kept fixed (identity quaternion) because only
-- position planning is required for this demo.
-- ------------------------------------------------------------
local function pose3dFromDummy(h)
    local p=sim.getObjectPosition(h)         -- [x y z]
    return {p[1],p[2],p[3], 0,0,0,1}         -- [x y z qx qy qz qw]
end


-- ------------------------------------------------------------
-- Convert a pose3d OMPL path to an XYZ-only list:
-- Input pathPose3d format:
--   [x y z qx qy qz qw  x y z qx qy qz qw  ...]
-- Output xyz format:
--   [x y z  x y z  ...]
-- This is convenient for drawing and for the path follower.
-- ------------------------------------------------------------
local function xyzFromPose3dPath(pathPose3d)
    local xyz={}
    for i=1,#pathPose3d,7 do
        xyz[#xyz+1]=pathPose3d[i]
        xyz[#xyz+1]=pathPose3d[i+1]
        xyz[#xyz+1]=pathPose3d[i+2]
    end
    return xyz
end


-- ------------------------------------------------------------
-- Draw the XYZ path in the scene as cyan line segments.
-- The drawing container is created once and reused.
-- ------------------------------------------------------------
local function drawPathXYZ(xyz)
    -- Create a drawing object container only once:
    if not _lineContainer then
        -- sim.drawing_lines: draw line segments
        -- 3: line thickness
        -- 0: parent object (-1 would be world; here it is handled by default)
        -- -1: "no object handle" for drawing object attachment
        -- 99999: max items
        -- {0,1,1}: cyan color
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,3,0,-1,99999,{0,1,1})
    end

    -- Clear previous path drawing:
    sim.addDrawingObjectItem(_lineContainer,nil)

    -- Draw line segments between consecutive points:
    for i=1,#xyz-3,3 do
        sim.addDrawingObjectItem(
            _lineContainer,
            {xyz[i],xyz[i+1],xyz[i+2], xyz[i+3],xyz[i+4],xyz[i+5]}
        )
    end
end


-- ------------------------------------------------------------
-- sysCall_init() runs once when the simulation starts.
-- The follower script may still have old data from a previous run,
-- so the path is invalidated here. READY=0 signals "wait".
-- ------------------------------------------------------------
function sysCall_init()
    local pathHandle=sim.getObject('/Path')

    -- Invalidate old path data BEFORE planning:
    sim.writeCustomDataBlock(pathHandle,'OMPL_XYZ_PATH',nil)

    -- Handshake flag: 0 = not ready (path not computed yet):
    sim.writeCustomDataBlock(pathHandle,'OMPL_READY',sim.packInt32Table({0}))
end


-- ------------------------------------------------------------
-- sysCall_thread() runs as a threaded script (can use sim.step()).
-- It configures OMPL, solves the planning problem, stores the path,
-- increments PATH_ID, and finally marks READY=1.
-- ------------------------------------------------------------
function sysCall_thread()
    -- Get required scene objects (must exist and match names):
    local pathHandle = sim.getObject('/Path')
    local startDummy = sim.getObject('/Path/start')
    local goalDummy  = sim.getObject('/Path/goal')
    local proxy      = sim.getObject('/Path/omplRobot')  -- collision proxy shape

    -- Create an OMPL planning task:
    local task=simOMPL.createTask('droneTask')

    -- Define planning bounds (world coordinates):
    -- For a ~5x5 floor centered near (0,0), x,y in [-2.5, 2.5] is typical.
    -- z bounds define the allowed altitude range.
    local low={-2.5,-2.5,0.2}
    local high={ 2.5, 2.5,1.5}

    -- Create a 3D pose state space. The last parameter is useForProjection=1.
    -- This avoids "Dimension of projection needs to be larger than 0" errors.
    local ss={
        simOMPL.createStateSpace(
            's',
            simOMPL.StateSpaceType.pose3d,
            proxy,
            low,
            high,
            1 -- useForProjection = 1
        )
    }
    simOMPL.setStateSpace(task, ss)

    -- Choose a planner:
    simOMPL.setAlgorithm(task, simOMPL.Algorithm.RRTConnect)

    -- Collision pairs:
    -- Using sim.handle_all: therefore only obstacles + proxy should be Collidable.
    simOMPL.setCollisionPairs(task, {proxy, sim.handle_all})

    -- State validity resolution:
    -- Smaller values = more accurate collision checking, slower planning.
    simOMPL.setStateValidityCheckingResolution(task, 0.02)

    -- Set OMPL start and goal states from dummies:
    simOMPL.setStartState(task, pose3dFromDummy(startDummy))
    simOMPL.setGoalState(task,  pose3dFromDummy(goalDummy))

    -- Finalize task setup:
    simOMPL.setup(task)

    -- Compute path:
    -- Arguments: (task, maxTime, maxSimplifyTime, minStateCount)
    -- - maxTime: seconds for planning
    -- - maxSimplifyTime: -1 uses default simplification behavior
    -- - minStateCount: increases interpolation density (smoother path)
    local solved, path=simOMPL.compute(task, 5.0, -1.0, 300)

    -- If no path is found, log and exit:
    if not path then
        sim.addLog(sim.verbosity_errors,'OMPL: No path found.')
        return
    end

    -- Convert pose3d path to xyz-only:
    local xyz=xyzFromPose3dPath(path)

    -- Store path for the follower script:
    sim.writeCustomDataBlock(pathHandle,'OMPL_XYZ_PATH',sim.packFloatTable(xyz))

    -- Increment PATH_ID only after a new path exists (prevents race issues):
    local id=0
    local packed=sim.readCustomDataBlock(pathHandle,'OMPL_PATH_ID')
    if packed then
        id=sim.unpackInt32Table(packed)[1]
    end
    id=id+1
    sim.writeCustomDataBlock(pathHandle,'OMPL_PATH_ID',sim.packInt32Table({id}))

    -- Mark path as ready for the follower:
    sim.writeCustomDataBlock(pathHandle,'OMPL_READY',sim.packInt32Table({1}))

    -- Draw the path in cyan:
    drawPathXYZ(xyz)
end