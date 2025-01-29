
function update()
   gcs:send_text(0, "hello, world") -- send the traditional message
   local current_pos = ahrs:get_position() -- fetch the current position of the vehicle
   local home = ahrs:get_home()            -- fetch the home position of the vehicle
   if current_pos and home then            -- check that both a vehicle location, and home location are available
      local distance = current_pos:get_distance(home) -- calculate the distance from home in meters
      if distance > 1000 then -- if more then 1000 meters away
      distance = 1000;      -- clamp the distance to 1000 meters
      end
     -- servo.set_output_pwm(96, 1000 + distance) -- set the servo assigned function 96 (scripting3) to a proportional value
   end
   local pN = loc_2_pN_VanNuys(current_pos)
   local pE = loc_2_pE_VanNuys(current_pos)
   local distance_to_box_center = 1.25*math.pi*50*pN;
   local alt_ft = current_pos:alt() --/30.48; --convert cm to ft
   local dist_to_ground = alt_ft/math.tan(-GS_com);
   local velNED = ahrs:get_velocity_NED()
   gcs:send_text(0, alt_ft)
   gcs:send_text(0, pN)
   gcs:send_text(0, pE)
   gcs:send_text(0, velNED:x())
   gcs:send_text(0, velNED:y())
   gcs:send_text(0, velNED:z())
     
   return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
end
return update, 1000

