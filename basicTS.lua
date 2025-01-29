function update()
   --gcs:send_text(0, "hello, world") -- send the traditional message
   --print("hello world print")
   local current_pos = ahrs:get_location() -- fetch the current position of the vehicle
   if current_pos == nil then
      gcs:send_text(0,"position is nil")
   --else
   end
      --local home = ahrs:get_home()            -- fetch the home position of the vehicle
      -- servo.set_output_pwm(96, 1000 + distance) -- set the servo assigned function 96 (scripting3) to a proportional value
     -- local pN = current_pos:lng()
     -- local pE = current_pos:lat()
     -- local distance_to_box_center = 1.25*3.14*50*pN
     -- local alt_ft = current_pos:alt() --/30.48; --convert cm to ft
     -- local dist_to_ground = alt_ft/math.tan(-10*3.14/180)
      --local velNED = ahrs:get_velocity_NED()
      --gcs:send_text(0, alt_ft)
      -- gcs:send_text(0, pN)
      -- gcs:send_text(0, pE)
      -- gcs:send_text(0, velNED:x())
      -- gcs:send_text(0, velNED:y())
      --gcs:send_text(0, velNED:z())
   gcs:send_text(0, string.format("alt:%.1f pN:%.1f vx:%.1f", 200, 100, 3))
      --print(alt_ft)
      --print("printing alt")
  -- end
   
   return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
end
return update()

