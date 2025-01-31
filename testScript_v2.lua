-- support follow in GUIDED mode in plane

local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "GFOLL_"

local MODE_GUIDED = 15
local MODE_AUTO = 10
local turnFlag = 0
local ALT_FRAME_ABSOLUTE = 0


-- -- current target
local target_pos
local current_pos
local GS_com = -10 * math.pi/180 --glide slope command in radians (-10 deg)

-- -- other state
local have_target = false

-- -- check key parameters
-- function check_pa rameters()
--   --[[
--      parameter values which are auto-set on startup
--   --]]
--    local key_params = {
--       FOLL_ENABLE = 1,
--       FOLL_OFS_TYPE = 1,
--       FOLL_ALT_TYPE = 0,
--    }

--    for p, v in pairs(key_params) do
--       local current = param:get(p)
--       assert(current, string.format("Parameter %s not found", p))
--       if math.abs(v-current) > 0.001 then
--          param:set_and_save(p, v)
--          gcs:send_text(0,string.format("Parameter %s set to %.2f was %.2f", p, v, current))
--       end
--    end
-- end


function update()
  gcs:send_text(0, "hello, world") -- send the traditional message

  if ahrs then 
     current_pos = ahrs:get_position() -- fetch the current position of the vehicle
     home = ahrs:get_home() -- fetch the home position of the vehicle
  end        

  if current_pos and home then  -- check that both a vehicle location, and home location are available
     local distance = current_pos:get_distance(home) -- calculate the distance from home in meters
     if distance > 1000 then -- if more than 1000 meters away
        distance = 1000  -- clamp the distance to 1000 meters
     end
     -- servo.set_output_pwm(96, 1000 + distance) -- set the servo assigned function 96 (scripting3) to a proportional value
  end

  if current_pos then
     local pN = loc_2_pN_VanNuys(current_pos)
     local pE = loc_2_pE_VanNuys(current_pos)
     local alt_ft = current_pos:alt() -- altitude
     local velNED = ahrs:get_velocity_NED()

     -- Ensure values sent to `gcs:send_text()` are strings
     gcs:send_text(0, "Altitude (ft): " .. tostring(alt_ft))
     gcs:send_text(0, "pN: " .. tostring(pN))
     gcs:send_text(0, "pE: " .. tostring(pE))
     gcs:send_text(0, "Velocity X: " .. tostring(velNED:x()))
     gcs:send_text(0, "Velocity Y: " .. tostring(velNED:y()))
     gcs:send_text(0, "Velocity Z: " .. tostring(velNED:z()))
  end
    
  return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
end



-- function loop()
--    update()
--    -- run at 20Hz
--    return loop, 50
-- end

--check_parameters()

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 50
end
 --for test, "home" is near dbf lab and heading is east
--NOTE: pN and pE aren't actually N/E positions. They are defined relative to the runway position and orientation. pN is along runway with bonus box to the right, and pE is 90 deg of that
function loc_2_pN_VanNuys(loc)
    local heading = 90--in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.0684539 *3.14159265/180; -- in rad
    local home_long = -118.4427023*3.14159265/180 --in rad
    local current_lat = loc:lat()/1e7 * 3.14159265/180 -- in rad
    local current_long = loc:lng()/1e7 * 3.14159265/180 -- in rad
    local R = 20903520 -- earth's radius in feet
    local delta_east = R*math.cos(home_lat)* (current_long-home_long) --distance east of home in ft.
    local delta_north = R*(current_lat-home_lat)--distance north of home in ft
    local pN = -delta_east*math.sin(-heading)+delta_north*math.cos(-heading)
    return pN;
end

function loc_2_pE_VanNuys(loc)
   local heading = 90 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
   local home_lat = 34.0684539 *3.14159265/180 -- in rad
   local home_long = -118.4427023*3.14159265/180 --in rad
   local current_lat = loc:lat()/1e7 * 3.14159265/180 -- in rad
   local current_long = loc:lng()/1e7 * 3.14159265/180 -- in rad
   local R = 20903520 -- earth's radius in feet
   local delta_east = R*math.cos(home_lat)* (current_long-home_long) --distance east of home in ft.
   local delta_north = R*(current_lat-home_lat)--distance north of home in ft
   local pE = delta_east*math.cos(-heading)+delta_north*math.sin(-heading);
    return pE;
end

function pN_pE_VanNuys_2_lng(pN, pE)
    local heading = 90 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.0684539 *3.14159265/180 -- in rad
    local home_long = -118.4427023*3.14159265/180 --in rad
    local R = 20903520 -- earth's radius in feet
    local delta_east = pE*math.cos(heading)+pN*math.sin(heading)
    --delta_north = -pN*sin(heading)+pN*cos(heading);
    local current_long = home_long+delta_east/(R*math.cos(home_lat))
    return math.floor(current_long*1e7*180/3.14159265)
end

function pN_pE_VanNuys_2_lat(pN, pE)
    local heading = 90 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.0684539 *3.14159265/180 -- in rad
    local ome_long = -118.4427023*3.14159265/180 --in rad
    local R = 20903520 -- earth's radius in feet
    --delta_east = pE*cos(heading)+pN*sin(heading);
    local delta_north = -pN*math.sin(heading)+pN*math.cos(heading)
    local current_lat = home_lat+delta_north/R
    return math.floor(current_lat*1e7*180/3.1415926)
end

-- start running update loop
--return protected_wrapper()
return update()

