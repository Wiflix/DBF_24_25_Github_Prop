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
local locNew
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

--location home
local home_lat = 34.1749737 *3.14159265/180 -- in rad
local home_long = -118.4814977*3.14159265/180 --in rad
local heading = -38.5 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
local R = 20903520 -- earth's radius in feet


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
        return update, 1000
    end
    if not ahrs:healthy() then
        gcs:send_text(0, "AHRS not healthy!")
        return update, 1000
    end

    current_pos = ahrs:get_position()
    if not current_pos then
        return update, 1000
    end

    if vehicle:get_mode() ~= MODE_GUIDED then
        return update, 1000
    end

    -- local next_WP = vehicle:get_target_location()
    -- if not next_WP then
    --     return update, 1000
    -- end

    -- Turn decision logic
    if turnFlag == 0 then
        local target_pos_1 = current_pos --"far" waypoint 
        target_pos_1:lng(pN_pE_VanNuys_2_lng(1000, 0))
        target_pos_1:lat(pN_pE_VanNuys_2_lat(1000, 0))
        vehicle:set_target_location(target_pos_1)
        local pN = loc_2_pN_VanNuys(current_pos)
        local pE = loc_2_pE_VanNuys(current_pos)
        local distance_to_box_center = 1.25 * math.pi * 50 + pN
        local alt_ft = current_pos:alt()
        local dist_to_ground = alt_ft / math.tan(-GS_com)
        if math.abs(distance_to_box_center - dist_to_ground) < 50 then
            turnFlag = 1
            local wpNew_pE = 150
            local wpNew_pN = pN - 100
            local new_alt = 15 -- in m 
            locNew = current_pos
            locNew:lng(pN_pE_VanNuys_2_lng(wpNew_pN, wpNew_pE))
            locNew:lat(pN_pE_VanNuys_2_lat(wpNew_pN, wpNew_pE))
            locNew:alt(new_alt)
            vehicle:set_target_location(locNew)
        end
    elseif turnFlag == 1 then
        local dist_targ = current_pos:get_distance(locNew) -- dist in m
        if math.abs(dist_targ)<15 then
            local locBB = locNew
            locBB:lng(pN_pE_VanNuys_2_lng(0, 100))
            locBB:lat(pN_pE_VanNuys_2_lat(0, 100))
            locBB:alt(0)
            vehicle:set_target_location(locBB)
            turnFlag = 2
        end
    end


        
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
    local current_lat = loc:lat()/1e7 * 3.14159265/180 -- in rad
    local current_long = loc:lng()/1e7 * 3.14159265/180 -- in rad
    local delta_east = R*math.cos(home_lat)* (current_long-home_long) --distance east of home in ft.
    local delta_north = R*(current_lat-home_lat)--distance north of home in ft
    local pN = -delta_east*math.sin(-heading)+delta_north*math.cos(-heading)
    return pN;
end

function loc_2_pE_VanNuys(loc)
   local current_lat = loc:lat()/1e7 * 3.14159265/180 -- in rad
   local current_long = loc:lng()/1e7 * 3.14159265/180 -- in rad
   local delta_east = R*math.cos(home_lat)* (current_long-home_long) --distance east of home in ft.
   local delta_north = R*(current_lat-home_lat)--distance north of home in ft
   local pE = delta_east*math.cos(-heading)+delta_north*math.sin(-heading);
    return pE;
end

function pN_pE_VanNuys_2_lng(pN, pE)
    local delta_east = pE*math.cos(-heading)-pN*math.sin(-heading)
    --delta_north = -pN*sin(heading)+pN*cos(heading);
    local current_long = home_long+delta_east/(R*math.cos(home_lat))
    return math.floor(current_long*1e7*180/3.14159265)
end

function pN_pE_VanNuys_2_lat(pN, pE)
    --gcs:send_text(0, string.format("pN: %s", tostring(pN)))
    --gcs:send_text(0, string.format("pE: %s", tostring(pE)))
    local delta_north = pE*math.sin(-heading)+pN*math.cos(-heading)
    local current_lat = home_lat+delta_north/R
    return math.floor(current_lat*1e7*180/3.1415926)
end

return protected_wrapper()
