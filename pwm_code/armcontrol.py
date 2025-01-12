#!/usr/bin/env python3

import rospy
import std_msgs.msg
import tty
import sys
import termios
import pygame
grip_state = 0
wrist_state = 0
stepper_state = 0
upper_linacc_state = 160
lower_linacc_state = 55

def on_key_press(key_events):
    global grip_state, wrist_state, stepper_state, upper_linacc_state, lower_linacc_state
    if key_events ==  'z':
        print("Lower Linacc Up")
        lower_linacc_state -= 0.5
        lower_linacc_state = lower_linacc_state if lower_linacc_state > 10 else 10
    elif key_events ==  'a':
        print("Lower Linacc Down")
        lower_linacc_state += 0.5
        lower_linacc_state = lower_linacc_state if lower_linacc_state < 160 else 160
    
    if key_events ==  's':
        print("Upper Linac Up")
        upper_linacc_state -= 0.5
        upper_linacc_state = upper_linacc_state if upper_linacc_state > 10 else 10
    elif key_events ==  'x':
        print("Upper Linac Down")
        upper_linacc_state += 0.5
        upper_linacc_state = upper_linacc_state if upper_linacc_state < 160 else 160
    
    if key_events ==  'q':
        print("Gripper Close")
        grip_state = 1
    elif key_events ==  'w':
        print("Gripper Open")
        grip_state = 2
    else:
        grip_state = 0
    
    if key_events ==  'c':
        print("Stepper Clockwise")
        stepper_state = 1
    elif key_events ==  'v':
        print("Stepper Anticlockwise")
        stepper_state = 2
    else:
        stepper_state = 0

    if key_events ==  'e':
        print("Wrist Up")
        wrist_state = 1
    elif key_events ==  'd':
        print("Wrist down")
        wrist_state = 2
    elif key_events ==  'r':
        print("Wrist Clockwise")
        wrist_state = 3
    elif key_events ==  'f':
        print("Wrist Anticlockwise")
        wrist_state = 4
    else:
        wrist_state = 0


def armcontrol():
    global grip_state, wrist_state, stepper_state, upper_linacc_state, lower_linacc_state
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    grip_out = std_msgs.msg.Int32()
    wrist_out = std_msgs.msg.Int32()
    stepper_out = std_msgs.msg.Int32()
    upper_linacc_out = std_msgs.msg.Float32()
    lower_linacc_out = std_msgs.msg.Float32()

    grip_pub = rospy.Publisher('arm/gripper_control', std_msgs.msg.Int32, queue_size=10)
    stepper_pub = rospy.Publisher('arm/stepper_control', std_msgs.msg.Int32, queue_size=10)
    wrist_pub = rospy.Publisher('arm/wrist_control', std_msgs.msg.Int32, queue_size=10)
    upper_linacc_pub = rospy.Publisher('arm/upperacc_control', std_msgs.msg.Float32, queue_size=10)
    lower_linacc_pub = rospy.Publisher('arm/loweracc_control', std_msgs.msg.Float32, queue_size=10)

    rospy.init_node('armcontrol', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        key_in = sys.stdin.read(1)[0]
        on_key_press(key_in)
        grip_out.data = grip_state
        grip_pub.publish(grip_out)
        stepper_out.data = stepper_state
        stepper_pub.publish(stepper_out)
        wrist_out.data = wrist_state
        wrist_pub.publish(wrist_out)
        lower_linacc_out.data = lower_linacc_state
        lower_linacc_pub.publish(lower_linacc_out)
        upper_linacc_out.data = upper_linacc_state
        upper_linacc_pub.publish(upper_linacc_out)

        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

if __name__ == '__main__':
    try:
        armcontrol()
    except rospy.ROSInterruptException:
        pass