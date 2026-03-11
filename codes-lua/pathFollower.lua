-- ============================================================
-- pathFollower.lua
-- ------------------------------------------------------------
-- Purpose:
--   - Wait for the OMPL planner to generate a NEW path in the current run
--     (handshake via OMPL_READY flag stored in /Path).
--   - Read the computed XYZ waypoint list from /Path (OMPL_XYZ_PATH).
--   - Move the Quadcopter "target" along the path using time interpolation.
--   - The Quadcopter internal controller then moves the drone (dynamics).
--
-- Scene requirements (names must match):
--   /Path                 (dummy container holding planner data)
--   Planner script writes:
--     OMPL_READY    : int32 {0/1} (0 = invalidated/not ready, 1 = ready)
--     OMPL_XYZ_PATH : float[] packed [x1 y1 z1 x2 y2 z2 ...]
--
-- Notes:
--   - This script is typically attached as a THREADED child script
--     (e.g., under a dummy named "pathFollower") so it can use sim.step().
--   - The "target" name varies depending on the Quadcopter model/version,
--     so getTarget() tries common paths and can be customized.
-- ============================================================

sim=require 'sim'


-- ------------------------------------------------------------
-- Euclidean distance between 3D points a and b.
-- a, b are tables: {x, y, z}
-- ------------------------------------------------------------
local function dist(a,b)
    local dx=a[1]-b[1]
    local dy=a[2]-b[2]
    local dz=a[3]-b[3]
    return math.sqrt(dx*dx+dy*dy+dz*dz)
end


-- ------------------------------------------------------------
-- Locate the Quadcopter "target" object.
-- Different CoppeliaSim models/scenes may name it differently.
-- Customize here if your scene uses a different target path.
-- ------------------------------------------------------------
local function getTarget()
    -- Most common:
    local h=sim.getObject('/target',{noError=true})
    if h~=-1 then return h end

    -- Other common variants:
    h=sim.getObject('/Quadcopter_target',{noError=true})
    if h~=-1 then return h end

    h=sim.getObject('/Quadcopter/target',{noError=true})
    return h -- may be -1 if not found
end


-- ------------------------------------------------------------
-- Read the planner "READY" flag from /Path.
-- Returns:
--   0 -> planner invalidated path (current run start)
--   1 -> planner finished and path is available
--   nil -> flag not found yet
-- ------------------------------------------------------------
local function readReady(pathHandle)
    local r=sim.readCustomDataBlock(pathHandle,'OMPL_READY')
    if not r then return nil end
    return sim.unpackInt32Table(r)[1]
end


-- ------------------------------------------------------------
-- Threaded main:
--   1) Wait until the planner invalidates old data in THIS run (READY=0).
--      This prevents using a stored path from a previous simulation run.
--   2) Wait until planner publishes the new path (READY=1 and OMPL_XYZ_PATH exists).
--   3) Convert packed xyz array into waypoint list.
--   4) Move the target smoothly along consecutive segments.
-- ------------------------------------------------------------
function sysCall_thread()
    -- Get Quadcopter target:
    local target=getTarget()
    if target==-1 then
        error("Drone target not found. Update getTarget() with the correct object path.")
    end

    -- /Path contains the planner output (custom data blocks):
    local pathHandle=sim.getObject('/Path')

    -- 1) Wait for planner invalidation (READY=0) in THIS execution.
    -- This guarantees the follower does not start from a stale path.
    while true do
        local ready=readReady(pathHandle)
        if ready==0 then break end
        sim.step()
    end

    -- 2) Wait for a NEW path (READY=1 and OMPL_XYZ_PATH present):
    local xyz=nil
    while true do
        local ready=readReady(pathHandle)
        local p=sim.readCustomDataBlock(pathHandle,'OMPL_XYZ_PATH')
        if ready==1 and p then
            xyz=sim.unpackFloatTable(p) -- xyz = [x y z x y z ...]
            break
        end
        sim.step()
    end

    -- Convert packed xyz array into waypoint list:
    local wp={}
    for i=1,#xyz,3 do
        wp[#wp+1]={xyz[i],xyz[i+1],xyz[i+2]}
    end

    -- Motion parameters:
    -- speed: desired target speed in m/s (start small for stable tracking)
    local speed=0.3
    local dt=sim.getSimulationTimeStep()

    -- Follow each segment between consecutive waypoints:
    for k=1,#wp-1 do
        local p1=wp[k]
        local p2=wp[k+1]

        -- Segment length and time needed at the given speed:
        local d=dist(p1,p2)
        local T=d/speed

        -- Number of simulation steps for this segment:
        local steps=math.max(1, math.floor(T/dt))

        -- Interpolate along the segment:
        for i=1,steps do
            local s=i/steps
            local p={
                p1[1]*(1-s)+p2[1]*s,
                p1[2]*(1-s)+p2[2]*s,
                p1[3]*(1-s)+p2[3]*s
            }

            -- Move target; Quadcopter controller follows it:
            sim.setObjectPosition(target,p)

            -- Advance simulation one step:
            sim.step()
        end
    end
end