--COORDS SET FOR WHITTIER AIRFIELD
-- Support follow in GUIDED mode in plane
--IMPORTANT FOR THIS TO WORK: Make sure to turn SpdWeight is set to 0. Set min airspeed to 1.2*stall speed to make sure x1 doesn't stall
local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "GFOLL_"

local MODE_GUIDED = 15
local MODE_AUTO = 10
local turnFlag = 0
local initFlag = 0
local stupid_error_flag = 0;
local ALT_FRAME_ABSOLUTE = 0

-- Current target
local target_pos
local current_pos
local locNew
local GS_com = -15 * math.pi / 180 -- Glide slope command in radians (-10 deg)

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
local home_lat = 34.0442899 *3.14159265/180 -- in rad
local home_long = -118.0705457*3.14159265/180 --in rad
local heading = -155.9 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
local R = 20903520 -- earth's radius in feet

function update()
    if vehicle:get_mode() ~= MODE_GUIDED then
        gcs:send_text(0,"not in guided")
        return update, 1000
     end
    if not ahrs or not ahrs.healthy then
        gcs:send_text(0, "AHRS module unavailable!")
        return update, 1000

    end
    if not ahrs:healthy() then
        
        gcs:send_text(0, "AHRS not healthy!")
        return update, 1000
    end

    current_pos = ahrs:get_position()
    gcs:send_text(0, "current pos sent!")
    if not current_pos then
        return update, 1000
    end

 --   if vehicle:get_mode() ~= MODE_GUIDED then
      --  return update, 1000
    --end

    -- local next_WP = vehicle:get_target_location()
    -- if not next_WP then
    --     return update, 1000
    -- end


    --this block of code runs first. Sets initial "target" position as soon as altitude is available in order to define glideslope. Only runs once
    if initFlag == 0 then
        local target_pos_1 = current_pos --"far" waypoint 
        local pN = loc_2_pN_VanNuys(current_pos)
        local pE = loc_2_pE_VanNuys(current_pos)
        --local alt_ft = current_pos:alt()*3.28 -- in ft
        local dist = ahrs:get_relative_position_NED_home()
        local alt_ft = -1*dist:z()*3.28
        local delta_pN_req = alt_ft / math.tan(-GS_com)
        target_pos_1:lng(pN_pE_VanNuys_2_lng(pN+delta_pN_req, pE))
        target_pos_1:lat(pN_pE_VanNuys_2_lat(pN+delta_pN_req, pE))
        --target_pos_1:lng(pN_pE_VanNuys_2_lng(-5000, 0))
        --target_pos_1:lat(pN_pE_VanNuys_2_lat(-5000, 0))
        local temp = target_pos_1:alt()
        target_pos_1:alt(temp+100*dist:z()) -- should set it to 0 in its own frame, in cm
        vehicle:set_target_location(target_pos_1)
        initFlag = 1
        gcs:send_text(0,"init")
    end
    -- Turn decision logic
    if turnFlag == 0 then
        local pN = loc_2_pN_VanNuys(current_pos)
        local pE = loc_2_pE_VanNuys(current_pos)
        local distance_to_box_center = 1.25 * math.pi * 50 + pN --1.25 factor can be modified depending on performance. It's an estimate to how much wider the turn will be than a perfect semi-circle
       -- local alt_ft = current_pos:alt()*3.28 -- in ft
        local dist = ahrs:get_relative_position_NED_home()
        local alt_ft = -1*dist:z()*3.28
        local dist_to_ground = alt_ft / math.tan(-GS_com)
        gcs:send_text(0, string.format("pN: %s", tostring(pN)))
        gcs:send_text(0, string.format("pE: %s", tostring(pE)))
        gcs:send_text(0, string.format("Alt_ft: %s", tostring(alt_ft)))
        gcs:send_text(0, string.format("DTG: %s", tostring(dist_to_ground)))
        gcs:send_text(0, string.format("DTBC: %s", tostring(distance_to_box_center)))
        logger:write("CST", "pN,pE,altft,DB,DG", "fffff",pN,pE,alt_ft,distance_to_box_center,dist_to_ground)
        if stupid_error_flag==0 then
            stupid_error_flag =1
            gcs:send_text(0, "stupid error flag")
            return update,100
        end
        if (dist_to_ground - distance_to_box_center) < 50 then
            turnFlag = 1
            --considering deleting this entire block and going straight to the turn flag 1 case (set target location immediately to bonus box)
            local wpNew_pE = 150
            local wpNew_pN = pN - 100
            local delta_alt = 15 -- in m. This is a guess to how much the glider will bleed altitude during the turn. Experiment with different values
            local new_alt = current_pos:alt()-delta_alt -- in m 
            locNew = current_pos
            locNew:lng(pN_pE_VanNuys_2_lng(wpNew_pN, wpNew_pE))
            locNew:lat(pN_pE_VanNuys_2_lat(wpNew_pN, wpNew_pE))
            locNew:alt(new_alt)
            vehicle:set_target_location(locNew)
            gcs:send_text(0,"TF1")
        end
    elseif turnFlag == 1 then
        local dist_targ = current_pos:get_distance(locNew) -- dist in m
       -- if math.abs(dist_targ)<15 then
       local pN = loc_2_pN_VanNuys(current_pos)
       local pE = loc_2_pE_VanNuys(current_pos)
       gcs:send_text(0, string.format("pN1: %s", tostring(pN)))
       gcs:send_text(0, string.format("pE1: %s", tostring(pE)))
       logger:write("CST", "pN,pE,altft,DB,DG", "fffff",pN,pE,0,0,0)
        if true then
            local locBB = locNew
            locBB:lng(pN_pE_VanNuys_2_lng(0, 100))
            --locBB:lng(home_long)
            locBB:lat(pN_pE_VanNuys_2_lat(0, 100))
            locBB:alt(0)
            vehicle:set_target_location(locBB)
            turnFlag = 2
            gcs:send_text(0,"TF2")
            gcs:send_text(0, string.format("Target location lng: %s", vehicle:get_target_location():lng()))
            gcs:send_text(0, string.format("Target location lat: %s", vehicle:get_target_location():lat()))
        end
    end
    if turnFlag == 2 then
        local pN = loc_2_pN_VanNuys(current_pos)
        local pE = loc_2_pE_VanNuys(current_pos)
        gcs:send_text(0, string.format("pN2: %s", tostring(pN)))
        gcs:send_text(0, string.format("pE2: %s", tostring(pE)))
        logger:write("CST", "pN,pE,altft,DB,DG", "fffff",pN,pE,0,0,0)
    end
    return update, 50
end

-- function protected_wrapper()
--     local success, err = pcall(update)
--     if not success then
--         gcs:send_text(0, "Internal Error: " .. err)
--         return nil
--     end
--     return protected_wrapper, 50
-- end
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

return update()
