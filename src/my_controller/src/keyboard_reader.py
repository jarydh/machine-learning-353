#! /usr/bin/env python

# Run this to manually drive the robot around
# to run, call:
# rosrun my_controller keyboard_driver.py

import sys, select, termios, tty
import rospy

from std_msgs.msg import Char

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Driving straight:
    i: fast
    k: slow
    m: stop
Angular speed:
    a: left
    s: straight
    d: right

spacebar: toggle recording

(currently commented out to stop accidents) anything else : stop

CTRL-C to quit
"""

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    test, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if test:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)

# initialize ros node
rospy.init_node('keyboard_reader')

key_pub = rospy.Publisher('/keyboard_input', Char, queue_size = 1)

try:
    print(msg)
    while not rospy.is_shutdown():
        key = getKey(None)
        # print(key)
        
        # exit
        if (key == '\x03'):
                break
        else:
            char = Char()
            char.data = ord(key)
            key_pub.publish(char)
        # elif key in linBindings.keys():
        #     # print("yes")
        #     # print(linBindings[key])
        #     driver.set_linear_speed(linBindings[key])
        # elif key in angBindings.keys():
        #     # print("yes")
        #     # print(linBindings[key])
        #     driver.set_angular_speed(angBindings[key])
        # elif key == ' ':
        #     recorder.toggle_is_recording()
        # # any other key, stop the car
        # elif key != '':
        #     print("Unrecognized key")
        #     # driver.set_linear_speed(0)
        #     # driver.set_angular_speed(0)
        
        # driver.drive()

        # # will only save image if recording is toggle on
        # recorder.capture_frame()
        # rate.sleep()
        # print(loop)
        # loop += 1
except Exception as e:
    print(e)

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
