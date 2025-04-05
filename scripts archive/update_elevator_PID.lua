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

--PID Variables
local dr_error_prior = 0
local dr_integral_prior = 0
local kp = 15
local ki = 0.8
local kd = 16
local k_pr = 100
local k_phi = -40
--local bias = 0
local GS_target_deg = -10  -- Target glide slope in degrees
--local airspeed_target = 15 -- Target airspeed in m/s
local last_time = millis()  -- To store the time of the last update
function update()
    if vehicle:get_mode() ~= MODE_GUIDED then
        gcs:send_text(0,"not in guided")
        return update, 1000
     end
    local current_time = millis()  -- Get the current time (in milliseconds)
    gcs:send_text(0, string.format("current_time: %s", tostring(current_time)))
    gcs:send_text(0, string.format("last_time: %s", tostring(last_time)))
    local dt
    -- if current_time==last_time then
    --     dt = 0.02
    -- else
    --      dt = (current_time - last_time) -- Calculate time difference in seconds
    -- end
    gcs:send_text(0, string.format("dt: %s", tostring(dt)))

    -- Ensure the time difference is not too small
    -- if dt == 0 then
      --     gcs:send_text(0, "dt catch")
    --     return update, 50
    -- end
     dt = 0.02
    -- Update last_time for the next iteration
    last_time = current_time
    if not ahrs:healthy() then
        gcs:send_text(0,"AHRS not healthy!")
        return update, 1000
     end
    local velNED = ahrs:get_velocity_NED()
    if not velNED then return end
    local speed = math.sqrt(velNED:x()^2 + velNED:y()^2)*3.28084 -- in ft/s
    local des_rate_current = velNED:z()*3.28084 -- in ft/s
    local des_rate_target = 0.1--math.tan(GS_com) * speed
    gcs:send_text(0, string.format("dr_current: %s", tostring(des_rate_current)))
    gcs:send_text(0, string.format("dr_target: %s", tostring(des_rate_target)))
    local dr_error = des_rate_current - des_rate_target
    local integral = dr_integral_prior + dr_error*dt
    local derivative = (dr_error - dr_error_prior)/dt
    local rates = ahrs:get_gyro()
    local pitch_rate = math.deg(rates:y())
    local phi = ahrs:get_roll()
    gcs:send_text(0, string.format("dr_error: %s", tostring(dr_error)))
    gcs:send_text(0, string.format("integral: %s", tostring(integral)))
    gcs:send_text(0, string.format("derivative: %s", tostring(derivative)))
    gcs:send_text(0, string.format("pitch_rate: %s", tostring(pitch_rate)))
    gcs:send_text(0, string.format("phi: %s", tostring(phi)))


    local elev_rad = dr_error * kp + integral * ki + derivative * kd + pitch_rate * k_pr + math.tan(math.abs(phi)) * k_phi
    if not elev_rad or elev_rad ~= elev_rad then  -- Check for nil or NaN
        elev_rad = 0
    end
    elev_rad = elev_rad*0.01
    elev_rad = apply_transfer_function(elev_rad)
    gcs:send_text(0, string.format("Elevator Command: %.1f deg", elev_rad*180/3.14))
    local pwm_min, pwm_max = 1000, 2000
    local elev_max, elev_min = 40 * math.pi / 180, -40 * math.pi / 180
    local pwm_target = pwm_min + (elev_rad-elev_min) * (pwm_max - pwm_min)/(elev_max - elev_min)
    pwm_target = math.floor(pwm_target + 0.5)  -- Rounds to the nearest integer
    if pwm_target>pwm_max then
        pwm_target = pwm_max
    elseif pwm_target<pwm_min then
        pwm_target = pwm_min
    end
    SRV_Channels:set_output_pwm_chan_timeout(5, pwm_target, 50)
    gcs:send_text(0, string.format("pwm Command: %.1f deg", pwm_target))
     dr_error_prior = dr_error
     dr_integral_prior = integral
    -- -- Send feedback to GCS
    gcs:send_text(0,"command succeeded")

    return update, 50 -- Run every 50 ms
end

-- Transfer function coefficients
local b0 = 0.487
local a1 = -0.606  -- (negative because it's moved to RHS)
local y_prior = 0  -- Store previous output

function apply_transfer_function(elev_rad)
    local y = b0 * elev_rad - a1 * y_prior
    y_prior = y  -- Store for next iteration
    return y
end


return update()

