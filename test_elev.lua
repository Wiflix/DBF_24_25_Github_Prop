--5 is elevator, confirmed
local MODE_AUTO = 10
--1000 is elev down, 2000 is up
function update1()
    if vehicle:get_mode() ~= MODE_AUTO then
        return update1(), 100
     end
    --override servo for 5s in direction 1
    gcs:send_text(0, "Update1")
    SRV_Channels:set_output_pwm_chan_timeout(5, 1000, 5000)
    return update2, 10000 -- wait 10s
end
function update2()
    if vehicle:get_mode() ~= MODE_AUTO then
        return update2(), 100
     end
    gcs:send_text(0, "Update2")
    SRV_Channels:set_output_pwm_chan_timeout(5, 2000, 5000)
    return update1, 10000 -- wait 10s
end

return update1()

