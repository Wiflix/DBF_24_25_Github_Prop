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
-- function update_target()
--     if not follow or not follow.have_target or not follow:get_target_location_and_velocity_ofs() then
--         gcs:send_text(0, "Follow system not available!")
--         return
--     end
--     if not follow:have_target() then
--         if have_target then
--             gcs:send_text(0, "Lost beacon")
--         end
--         have_target = false
--         return
--     end
--     if not have_target then
--         gcs:send_text(0, "Have beacon")
--     end
--     have_target = true
--     target_pos = follow:get_target_location_and_velocity_ofs()
-- end

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
            local wp = mission:get_item(1)  
	
          -- local instance = gps:primary_sensor()
            --local position = gps:location(instance)
           -- local position = ahrs:get_position()
       
           if (not wp) then
             gcs:send_text(0, "no homepoint set!")
               return update()
           end
           
           --if (not position) then
             ----gcs:send_text(0, "no position data!")
            --   return false
           --end
          
           -- gcs:send_text(0, mission:num_commands())
           
           --wp:command(LAND)
           wp:x(341756750)
           wp:y(-1184813476)
           wp:z(20)  
       
           -- mission:set_item(mission:num_commands(), wp)
           mission:set_item(1, wp)

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
        return nil
    end
    return protected_wrapper, 50
end
function loc_2_pN_VanNuys(loc)
    local heading = -38.09 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.1752756 *3.14159265/180 -- in rad
    local home_long = -118.4811115*3.14159265/180 --in rad
    local current_lat = loc:lat()/1e7 * 3.14159265/180 -- in rad
    local current_long = loc:lng()/1e7 * 3.14159265/180 -- in rad
    local R = 20903520 -- earth's radius in feet
    local delta_east = R*math.cos(home_lat)* (current_long-home_long) --distance east of home in ft.
    local delta_north = R*(current_lat-home_lat)--distance north of home in ft
    local pN = -delta_east*math.sin(-heading)+delta_north*math.cos(-heading)
    return pN;
end

function loc_2_pE_VanNuys(loc)
   local heading = -38.09 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
   local home_lat = 34.1752756 *3.14159265/180 -- in rad
   local home_long = -118.4811115*3.14159265/180 --in rad
   local current_lat = loc:lat()/1e7 * 3.14159265/180 -- in rad
   local current_long = loc:lng()/1e7 * 3.14159265/180 -- in rad
   local R = 20903520 -- earth's radius in feet
   local delta_east = R*math.cos(home_lat)* (current_long-home_long) --distance east of home in ft.
   local delta_north = R*(current_lat-home_lat)--distance north of home in ft
   local pE = delta_east*math.cos(-heading)+delta_north*math.sin(-heading);
    return pE;
end

function pN_pE_VanNuys_2_lng(pN, pE)
    local heading = -38.09 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.1752756 *3.14159265/180 -- in rad
    local home_long = -118.4811115*3.14159265/180 --in rad
    local R = 20903520 -- earth's radius in feet
    local delta_east = pE*math.cos(heading)+pN*math.sin(heading)
    --delta_north = -pN*sin(heading)+pN*cos(heading);
    local current_long = home_long+delta_east/(R*math.cos(home_lat))
    return math.floor(current_long*1e7*180/3.14159265)
end

function pN_pE_VanNuys_2_lat(pN, pE)
    local heading = -38.09 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.1752756 *3.14159265/180 -- in rad
    local home_long = -118.4811115*3.14159265/180 --in rad
    local R = 20903520 -- earth's radius in feet
    --delta_east = pE*cos(heading)+pN*sin(heading);
    local delta_north = -pN*math.sin(heading)+pN*math.cos(heading)
    local current_lat = home_lat+delta_north/R
    return math.floor(current_lat*1e7*180/3.1415926)
end

return protected_wrapper()
