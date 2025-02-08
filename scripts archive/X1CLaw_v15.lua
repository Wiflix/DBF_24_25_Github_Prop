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
local dr_error_prior = 0
local dr_integral_prior = 0
local kp = 15
local ki = 0.8
local kd = 16
local k_pr = 100
local k_phi = -40
local bias = 0

function wrap_360(angle)
    local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
        res = res - 360
    end
    return res
end

-- Update target state
function update_target()
    if not follow or not follow.have_target or not follow:get_target_location_and_velocity_ofs then
        gcs:send_text(0, "Follow system not available!")
        return
    end
    if not follow:have_target() then
        if have_target then
            gcs:send_text(0, "Lost beacon")
        end
        have_target = false
        return
    end
    if not have_target then
        gcs:send_text(0, "Have beacon")
    end
    have_target = true
    target_pos = follow:get_target_location_and_velocity_ofs()
end

function update()
    if not ahrs or not ahrs.healthy then
        gcs:send_text(0, "AHRS module unavailable!")
        return
    end
    if not ahrs:healthy() then
        gcs:send_text(0, "AHRS not healthy!")
        return
    end

    current_pos = ahrs:get_position()
    if not current_pos then
        return
    end

    if vehicle:get_mode() ~= MODE_AUTO then
        return
    end

    local next_WP = vehicle:get_target_location()
    if not next_WP then
        return
    end

    -- Turn decision logic
    if turnFlag == 0 then
        local pN = loc_2_pN_VanNuys(current_pos)
        local pE = loc_2_pE_VanNuys(current_pos)
        local distance_to_box_center = 1.25 * math.pi * 50 * pN
        local alt_ft = current_pos:alt()
        local dist_to_ground = alt_ft / math.tan(-GS_com)
        if math.abs(distance_to_box_center - dist_to_ground) < 50 then
            turnFlag = 1
            local wpNew_pE = 150
            local wpNew_pN = pN - 100
            local new_alt = 50
            local locNew = current_pos
            locNew:lng(pN_pE_VanNuys_2_lng(pN, pE))
            locNew:lat(pN_pE_VanNuys_2_lat(pN, pE))
            vehicle:set_target_location(locNew)
        end
    end

    -- Control Logic
    local velNED = ahrs:get_velocity_NED()
    if not velNED then return end
    local speed = math.sqrt(velNED:x()^2 + velNED:y()^2)
    local des_rate_current = velNED:z()
    local des_rate_target = math.tan(GS_com) * speed
    local dr_error = des_rate_current - des_rate_target
    local integral = dr_integral_prior + dr_error
    local derivative = dr_error - dr_error_prior
    local rates = ahrs:get_gyro()
    local pitch_rate = math.deg(rates:y())
    local phi = ahrs:get_roll()
    
    local elev_rad = dr_error * kp + integral * ki + derivative * kd + pitch_rate * k_pr + math.tan(math.abs(phi)) * k_phi
    local pwm_min, pwm_max = 800, 2000
    local elev_max, elev_min = 45 * math.pi / 180, -45 * math.pi / 180
    local pwm_target = 800 + elev_rad * (elev_max - elev_min) / (pwm_max - pwm_min)
    SRV_Channels:set_output_pwm_chan_timeout(6, pwm_target, 1000)

    dr_error_prior = dr_error
    dr_integral_prior = integral
end

function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 50
end

return protected_wrapper()
