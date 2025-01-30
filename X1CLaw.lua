-- support follow in GUIDED mode in plane

local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "GFOLL_"

local MODE_GUIDED = 15
local MODE_AUTO = 10
local turnFlag = 0
local ALT_FRAME_ABSOLUTE = 0

-- -- bind a parameter to a variable
-- function bind_param(name)
--    local p = Parameter()
--    assert(p:init(name), string.format('could not find %s parameter', name))
--    return p
-- end

-- -- add a parameter and bind it to a variable
-- function bind_add_param(name, idx, default_value)
--    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
--    return bind_param(PARAM_TABLE_PREFIX .. name)
-- end

-- -- setup SHIP specific parameters
-- assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')
-- GFOLL_ENABLE     = bind_add_param('ENABLE', 1, 0)

-- -- current target
local target_pos
local current_pos
local GS_com = -10 * math.pi/180 --glide slope command in radians (-10 deg)

-- -- other state
local have_target = false

-- -- check key parameters
-- function check_parameters()
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

-- function wrap_360(angle)
--    local res = math.fmod(angle, 360.0)
--     if res < 0 then
--         res = res + 360.0
--     end
--     return res
-- end

-- function wrap_180(angle)
--     local res = wrap_360(angle)
--     if res > 180 then
--        res = res - 360
--     end
--     return res
-- end

-- -- update target state
-- function update_target()
--    if not follow:have_target() then
--       if have_target then
--          gcs:send_text(0,"Lost beacon")
--       end
--       have_target = false
--       return
--    end
--    if not have_target then
--       gcs:send_text(0,"Have beacon")
--    end
--    have_target = true

--    target_pos = follow:get_target_location_and_velocity_ofs()
-- end
-- main update function

--vars to define PID

local dr_error_prior = 0
local dr_integral_prior = 0
local kp = 15
local ki = 0.8
local kd = 16
local k_pr = 100
local k_phi = -40
local bias = 0
function update()
   if not ahrs:healthy() then
      gcs:send_text(0,"AHRS not healthy!")
      return
   end
   current_pos = ahrs:get_position()
   if not current_pos then
      return
   end

 --  current_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)

   if vehicle:get_mode() ~= MODE_AUTO then
      return
   end

   local next_WP = vehicle:get_target_location()
   if not next_WP then
      -- not in a flight mode with a target location
      return
   end
   if turnFlag==0 then
      local pN = loc_2_pN_VanNuys(current_pos)
      local pE = loc_2_pE_VanNuys(current_pos)
      local distance_to_box_center = 1.25*math.pi*50*pN;
      local alt_ft = current_pos:alt() --/30.48; --convert cm to ft
      local dist_to_ground = alt_ft/math.tan(-GS_com);
      if math.abs(distance_to_box_center-dist_to_ground)<50 then
          turnFlag=1;
          --set wp loc
          local wpNew_pE = 150;
          local wpNew_pN = pN-100;
          local new_alt = 50; --placeholder
          --Location::AltFrame altFrame = plane.current_loc.get_alt_frame;
          local locNew = current_pos--Location(pN_pE_VanNuys_2_lat(pN,pE),pN_pE_VanNuys_2_lng(pN,pE), new_Alt, altFrame);
          locNew:lng(pN_pE_VanNuys_2_lng(pN,pE))
          locNew:lat(pN_pE_VanNuys_2_lat(pN,pE))
          vehicle:set_target_location(locNew)

      end   


   end

   -- update the target position from the follow library, which includes the offsets
   --target_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
   --vehicle:update_target_location(next_WP, target_pos)
   local velNED = ahrs:get_velocity_NED() --ahrs velocity//FIND WAY TO GET NED VELOCITY VECTOR. This gets us descent rate and speed, which is what we need for GS control
   --local velNED = gps:velocity()
  --local des_rate = velNED:z()
   local speed = (velNED:x())^2+(velNED:y())^2
   --local gs_current = math.atan(des_rate,speed)
   local des_rate_current = velNED:z()
   local des_rate_target = math.tan(GS_com)*speed;
   local dr_error = des_rate_current - des_rate_target
   local integral = dr_integral_prior + dr_error
   local derivative = dr_error-dr_error_prior
   rates = ahrs:get_gyro()
   local pitch_rate = math.deg(rates:y())
   local phi = ahrs:get_roll()
   --MATLAB PID:
   local elev_rad = dr_error*kp + integral*ki + derivative*kd + pitch_rate*k_pr + math.tan(math.abs(phi))*k_phi
   local pwm_min = 800
   local pwm_max = 2000
   local elev_max = 45*3.14/180
   local elev_min = -45*3.14/180
   local pwm_target = 800+elev_rad*(elev_max-elev_min)/(pwm_max-pwm_min)
   SRV_Channels:set_output_pwm_chan_timeout(6, pwm_target, 1000)
   --ki and kd might be wrong due to discrete vs cont. time, double check elevator and pwm max and min values

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

--NOTE: pN and pE aren't actually N/E positions. They are defined relative to the runway position and orientation. pN is along runway with bonus box to the right, and pE is 90 deg of that
function loc_2_pN_VanNuys(loc)
    local heading = -38.09 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.17535989 *3.14159265/180; -- in rad
    local home_long = -118.4818518*3.14159265/180 --in rad
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
   local home_lat = 34.17535989 *3.14159265/180 -- in rad
   local home_long = -118.4818518*3.14159265/180 --in rad
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
    local home_lat = 34.17535989 *3.14159265/180 -- in rad
    local home_long = -118.4818518*3.14159265/180 --in rad
    local R = 20903520 -- earth's radius in feet
    local delta_east = pE*math.cos(heading)+pN*math.sin(heading)
    --delta_north = -pN*sin(heading)+pN*cos(heading);
    local current_long = home_long+delta_east/(R*math.cos(home_lat))
    return math.floor(current_long*1e7*180/3.14159265)
end

function pN_pE_VanNuys_2_lat(pN, pE)
    local heading = -38.09 --in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    local home_lat = 34.17535989 *3.14159265/180 -- in rad
    local ome_long = -118.4818518*3.14159265/180 --in rad
    local R = 20903520 -- earth's radius in feet
    --delta_east = pE*cos(heading)+pN*sin(heading);
    local delta_north = -pN*math.sin(heading)+pN*math.cos(heading)
    local current_lat = home_lat+delta_north/R
    return math.floor(current_lat*1e7*180/3.1415926)
end

-- start running update loop
return protected_wrapper()

