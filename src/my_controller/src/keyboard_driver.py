#! /usr/bin/env python

# Run this to manually drive the robot around
# to run, call:
# rosrun my_controller keyboard_driver.py

import rospy

from timer import simTime
import driver_controller as dc
import driver_image_recorder as rec

from std_msgs.msg import Char


linBindings = {
        'm':dc.LIN_STOP,
        'M':dc.LIN_STOP,
        'k':dc.LIN_SLOW,
        'K':dc.LIN_SLOW,
        'i':dc.LIN_FAST,
        'I':dc.LIN_FAST,
    }

angBindings = {
        'a':dc.ANG_LEFT,
        'A':dc.ANG_LEFT,
        's':dc.ANG_STRAIGHT,
        'S':dc.ANG_STRAIGHT,
        'd':dc.ANG_RIGHT,
        'D':dc.ANG_RIGHT,
    }


def newKey(data):
    global key_in
    key_in = chr(data.data)

def getKey():
    global key_in
    return key_in

def resetKey():
    global key_in
    key_in = ''

## MAIN PROGRAM
key_in = ''

key_in_sub = rospy.Subscriber('/keyboard_input', Char, newKey)


sim_time = simTime()


# initialize ros node
rospy.init_node('driver_keyboard')

# driver object
driver = dc.driverController(sim_time)

# recorder object
recorder = rec.driverImageRecorder(driver)
rate = rospy.Rate(10)
loop = 0

print("RECORDING OFF") 
while not rospy.is_shutdown():
    key = getKey()
    # print(key)
    
    # exit
    if (key == '\x03'):
            break
    elif key in linBindings.keys():
        # print("yes")
        # print(linBindings[key])
        driver.set_linear_speed(linBindings[key])
    elif key in angBindings.keys():
        # print("yes")
        # print(linBindings[key])
        driver.set_angular_speed(angBindings[key])
    elif key == ' ':
        recorder.toggle_is_recording()
    # any other key, stop the car
    elif key != '':
        print("Unrecognized key")
        # driver.set_linear_speed(0)
        # driver.set_angular_speed(0)
    
    resetKey()
    driver.drive()

    # will only save image if recording is toggle on
    recorder.capture_frame()
    rate.sleep()
    # print(loop)
    loop += 1


    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #   key_in = getKey(None)

    #   # 
    #   rate.sleep()