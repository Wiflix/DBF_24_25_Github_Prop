-- Support follow in GUIDED mode in plane
--DESC: TESTING pNpE FUNCTIONALITY FOR COURT OF SCIENCES. ALSO TESTING ALTITUDE
local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "GFOLL_"

local MODE_GUIDED = 15
local MODE_AUTO = 10
local turnFlag = 0
local initFlag = 0
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
local home_lat = 34.0688538 *3.14159265/180 -- in rad
local home_long = -118.4422356*3.14159265/180 --in rad
local heading = 0 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
local R = 20903520 -- earth's radius in feet

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


    --this block of code runs first. Sets initial "target" position as soon as altitude is available in order to define glideslope. Only runs once
    
        local target_pos_1 = current_pos --"far" waypoint 
        local pN = loc_2_pN_VanNuys(current_pos)
        local pE = loc_2_pE_VanNuys(current_pos)
        local alt_ft = current_pos:alt()*3.28 -- in ft
        gcs:send_text(0, string.format("pN: %s", tostring(pN)))
        gcs:send_text(0, string.format("pE: %s", tostring(pE)))
        gcs:send_text(0, string.format("Alt_ft: %s", tostring(alt_ft)))
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
