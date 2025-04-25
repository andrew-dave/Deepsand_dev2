#!/usr/bin/env python
## This is run on the robots

from digger.srv import botCommand, botCommandResponse
import rospy
import gpiod
import time

# Initialize the GPIO chip
chip = gpiod.Chip('2')

# Request the GPIO pins
line_lift = chip.get_line(6) #pwm2_u LINE 6
line_blade= chip.get_line(23)
#right drive
in1 = chip.get_line(25)
in2 = chip.get_line(24)
ena1 = chip.get_line(20)
#left drive
in3 = chip.get_line(15)
in4 = chip.get_line(16)
ena2 = chip.get_line(18)

# Request GPIO lines and set direction
line_lift.request(consumer='servo', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
line_blade.request(consumer='servo', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
in1.request(consumer='motor1', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]) 
in2.request(consumer='motor1', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]) 
ena1.request(consumer='motor1', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]) 
in3.request(consumer='motor1', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]) 
in4.request(consumer='motor1', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]) 
ena2.request(consumer='motor1', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
# Define the pulse range for the Lift servo
min_pulse = 1  # in milliseconds
max_pulse = 2  # in milliseconds
# Define the angles for the Lift servo's minimum and maximum positions
min_angle = 0  # in degrees
max_angle = 180  # in degrees



def set_servo_angle(line, angle,serv_time):
    time_end = time.time() + serv_time
    while time.time() < time_end: #seconds for this 
        # Calculate the pulse width for the desired angle
        pulse_width = (max_pulse - min_pulse) * angle / (max_angle - min_angle) + min_pulse
        #print(pulse_width)
        
        # Set the line_lift to high to start the pulse
        line.set_value(1)

        # Sleep for the pulse width
        time.sleep(pulse_width/ 1000.0) #time.sleep is in seconds
        
        # Set the line_lift to low to end the pulse
        line.set_value(0)

        # Sleep for the remaining time of the 20ms cycle
        time.sleep((20.0 - pulse_width) / 1000.0)

def brake():
    in1.set_value(0) 
    in2.set_value(0) 
    in3.set_value(0)
    in4.set_value(0)

#full forward
def forward(secs):
    in1.set_value(1)
    in2.set_value(0)
    ena1.set_value(1)
    in3.set_value(1)
    in4.set_value(0)
    ena2.set_value(1)

    time.sleep(secs)
    brake()

#full backward    
def reverse(secs):
    in1.set_value(0)
    in2.set_value(1)
    ena1.set_value(1)
    in3.set_value(0)
    in4.set_value(1)
    ena2.set_value(1)

    time.sleep(secs)
    brake() 

#rotate left
def rotL(secs):
    in1.set_value(1)
    in2.set_value(0)
    ena1.set_value(1)
    in3.set_value(0)
    in4.set_value(1)
    ena2.set_value(1)

    time.sleep(secs)
    brake()

#rotate Right
def rotR(secs):
    in1.set_value(0)
    in2.set_value(1)
    ena1.set_value(1)
    in3.set_value(1)
    in4.set_value(0)
    ena2.set_value(1)

    time.sleep(secs)
    brake() 



def bot_action(req):
    secs = 3
    # possible commands: ['fu','fd','bu','bd','lu','ld','ru','rd']
    bucket_command = req.comm[1]
    drive_command = req.comm[0]

    if bucket_command == 'u':
        set_servo_angle(line_lift, 0, 2)
        buck_resp = 'bucket up'
    elif bucket_command =='d':
        set_servo_angle(line_lift, 80, 2)
        buck_resp = 'bucket down'
    else:
        buck_resp = "ERROR:do not know bucket command"
    
    if drive_command == 'f':
        forward(secs)
        drive_resp = 'forward'
    elif drive_command == 'b':
        reverse(secs)
        drive_resp = 'backward'
    elif drive_command == 'r':
        rotR(secs)
        drive_resp = 'right'
    elif drive_command == 'l':
        rotL(secs)
        drive_resp = 'left'    
    else:
        drive_resp = "ERROR:do not know drive command"
        
    
    return botCommandResponse(buck_resp+ ' ' + drive_resp)
        


def bot_server():
    rospy.init_node('bot_server_node')
    s = rospy.Service('bot_srv', botCommand, bot_action)
    rospy.spin()
    


if __name__ == "__main__":
    bot_server()

    #cleanup
    line_lift.release()
    line_blade.release()
    in1.release()
    in2.release() 
    ena1.release()
    in3.release()
    in4.release() 
    ena2.release()
    chip.close()
