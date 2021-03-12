#! /usr/bin/env python

# Run this to manually drive the robot around
# to run, call:
# rosrun my_controller keyboard_driver.py

import sys, select, termios, tty
import rospy

from timer import simTime
import driver_controller as dc

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

anything else : stop

CTRL-C to quit
"""

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

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    sim_time = simTime()


    # initialize ros node
    rospy.init_node('driver_keyboard')

    # driver object
    driver = dc.driverController(sim_time)


    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey(0.1)
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
            # any other key, stop the car
            elif key != '':
                driver.set_linear_speed(0)
                driver.set_angular_speed(0)
            
            driver.drive()
    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #   key_in = getKey(None)

    #   # 
    #   rate.sleep()