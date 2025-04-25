#!/usr/bin/env python
## Test servo on Jetson Orin Nano with J401 carrier board

import gpiod
import time

# Initialize the GPIO chip
try:
    chip = gpiod.Chip('gpiochip0')
except Exception as e:
    print(f"Error opening gpiochip0: {e}")
    exit(1)

# Request the GPIO pin (line 41 = PJ.04, pin 15)
try:
    line_lift = chip.get_line(41)  # GPIO PJ.04, physical pin 15
    line_lift.request(consumer='servo', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
except Exception as e:
    print(f"Error requesting line 41: {e}")
    chip.close()
    exit(1)

# Define the pulse range for the servo
min_pulse = 1.0  # in milliseconds (0 degrees)
max_pulse = 2.0  # in milliseconds (180 degrees)
min_angle = 0    # in degrees
max_angle = 180  # in degrees
cycle = 20.0     # 20 ms cycle for 50 Hz

def set_servo_angle(line_lift, angle, serv_time):
    """Set servo angle using software PWM for specified duration."""
    if not min_angle <= angle <= max_angle:
        print(f"Angle must be between {min_angle} and {max_angle} degrees")
        return
    print(f"Setting angle to {angle} degrees")
    # Calculate pulse width
    pulse_width = min_pulse + (max_pulse - min_pulse) * (angle / (max_angle - min_angle))
    time_end = time.time() + serv_time
    while time.time() < time_end:
        line_lift.set_value(1)  # Start pulse
        time.sleep(pulse_width / 1000.0)  # Pulse width in seconds
        line_lift.set_value(0)  # End pulse
        time.sleep((cycle - pulse_width) / 1000.0)  # Remaining cycle

def main():
    try:
        set_servo_angle(line_lift, 0, 2)
        print("Lifting Blade")
        time.sleep(3)
        set_servo_angle(line_lift, 80, 2)
        print("Lowering Blade")
        return 0
    except Exception as e:
        print(f"Error in main: {e}")
        return 1

if __name__ == "__main__":
    try:
        main()
    finally:
        line_lift.set_value(0)  # Ensure servo is off
        line_lift.release()     # Release GPIO line
        chip.close()            # Close GPIO chip
        print("GPIO cleaned up")