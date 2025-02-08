-- Define servo output channel
local servo_channel = 6  -- Servo 6 (Ensure SERVO6_FUNCTION = 94 in parameters)

-- Define PWM limits (adjust as needed)
local min_pwm = 1100  -- Minimum PWM (microseconds)
local max_pwm = 1900  -- Maximum PWM (microseconds)
local step = 5        -- PWM step size for smooth motion
local direction = 1   -- 1 for increasing, -1 for decreasing
local current_pwm = min_pwm

-- Function to update servo position
local function update_servo()
    -- Change direction at limits
    if current_pwm >= max_pwm then
        direction = -1
    elseif current_pwm <= min_pwm then
        direction = 1
    end

    -- Update PWM signal
    current_pwm = current_pwm + (direction * step)
    SRV_Channels:set_output_pwm(servo_channel, current_pwm)

    -- Schedule the next execution (50 ms)
    return update_servo, 50
end

-- Start the script safely
return update_servo()