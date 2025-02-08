-- Support follow in GUIDED mode in plane

local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "GFOLL_"

local MODE_GUIDED = 15
local MODE_AUTO = 10
local turnFlag = 0
local ALT_FRAME_ABSOLUTE = 0

-- Current target
local target_pos
local current_pos
local GS_com = -10 * math.pi / 180 -- Glide slope command in radians (-10 deg)

-- Other state
local have_target = false

-- PID Variables
-- local dr_error_prior = 0
-- local dr_integral_prior = 0
-- local kp = 15
-- local ki = 0.8
-- local kd = 16
-- local k_pr = 100
-- local k_phi = -40
-- local bias = 0
local GS_target_deg = -10  -- Target glide slope in degrees
local airspeed_target = 15 -- Target airspeed in m/s

function update_glide_slope()

    local velNED = ahrs:get_velocity_NED()
    if not velNED then return end

    local ground_speed = math.sqrt(velNED:x()^2 + velNED:y()^2)  -- Horizontal speed (m/s)
    
    -- Convert glide slope angle to vertical speed
    local GS_target_rad = GS_target_deg * math.pi / 180 -- Convert degrees to radians
    local descent_rate = math.tan(GS_target_rad) * ground_speed -- Compute vertical speed

    -- Command TECS to follow the descent rate
    vehicle:set_target_climbrate(descent_rate)

    -- Optionally, set target airspeed too
    --vehicle:set_speed(airspeed_target)

    -- Send feedback to GCS
    gcs:send_text(0, string.format("Glide Slope: %.1f deg, Descent Rate: %.2f m/s", GS_target_deg, descent_rate))

    return update_glide_slope, 1000 -- Run every 1 second
end

return update_glide_slope()

