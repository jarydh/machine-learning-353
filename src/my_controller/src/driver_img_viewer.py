# used for viewing driver image data

import sys, select, termios, tty
import driver_pickler as dp
import numpy as np

pickler = dp.driverPickler()

FILENAME = "2021-03-21_19:49:51"

def getKeyBlocking():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [])
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def view_pickle(filename):
    imgs, speeds = pickler.load_pickle(filename)

    current_frame = 0

    # key = getKeyBlocking()
    # print(key)
    print(np.shape(imgs[1]))
    # # exit
    # if (key == '\x03'):
    #         break
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

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    view_pickle(FILENAME)