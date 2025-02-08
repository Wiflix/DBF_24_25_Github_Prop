
function update()

  --local scripting_rc_1 = rc:find_channel_for_option(300)

  local WAYPOINT    = 16
  local LAND        = 21
  local flag = true
  local armed = arming:is_armed()
  if (armed and flag) then
    local timenow = millis()
    while timenow+3000<millis() do
      --wait 10 seconds after arming before replacing waypoint
    end

    local wp = mission:get_item(1)  
	
   -- local instance = gps:primary_sensor()
	  --local position = gps:location(instance)
    -- local position = ahrs:get_position()

    if (not wp) then
		gcs:send_text(0, "no homepoint set!")
        return false
    end
    
    --if (not position) then
		----gcs:send_text(0, "no position data!")
     --   return false
    --end
	
	 -- gcs:send_text(0, mission:num_commands())
    
    wp:command(LAND)
    wp:x(341756750)
    wp:y(-1184813476)
    wp:z(0)  

    -- mission:set_item(mission:num_commands(), wp)
    mission:set_item(1, wp)
    flag = false
  end
   --gcs:send_text(0, "hello, world") -- send the traditional message
   return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
end
return update()

