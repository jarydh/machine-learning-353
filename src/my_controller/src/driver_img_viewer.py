#! /usr/bin/env python


# used for viewing driver image data

import sys, select, termios, tty
import driver_pickler as dp
import numpy as np
import cv2

pickler = dp.driverPickler()

FILENAME = "2021-03-22_15:41:11"

FRAME_SCALE_FACTOR = 3

msg = """
---------------------------
Next frame: h
Previous frame: g

Images are scaled up for easier viewing

CTRL-C to quit
"""


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
    print("Viewing Pickle File: " + filename +".pickle")
    print(msg)
    imgs, speeds = pickler.load_pickle(filename)

    current_frame = 0
    total_frames = np.shape(imgs)[0]


    while True:
        show_frame(imgs, speeds, current_frame, total_frames)
        key = getKeyBlocking()
        # exit
        if (key == '\x03'):
                break
        elif key == 'h':
            current_frame += 1
            if current_frame > total_frames - 1:
                current_frame = total_frames - 1
        elif key == 'g':
            current_frame -= 1
            if current_frame < 0:
                current_frame = 0
        elif key != '':
            print("Unrecognized key")

def show_frame(imgs, speeds, frame_number, total_frames):
    frame = imgs[frame_number]

    width = int(frame.shape[1] * FRAME_SCALE_FACTOR)
    height = int(frame.shape[0] * FRAME_SCALE_FACTOR)

    frame = cv2.resize(frame, (width, height))
    

    # Write some Text

    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,40)
    fontScale              = 1.2
    fontColor              = (0, 0,255)
    lineType               = 3
    text = 'Frame: ' + str(frame_number) + '/' + str(total_frames)

    cv2.putText(frame,'Frame: ' + str(frame_number) + '/' + str(total_frames - 1), 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        lineType)

    bottomLeftCornerOfText = (10,90)
    text = "Lin Speed: " + str(speeds[frame_number][0]) + "  Ang Speed: " + str(speeds[frame_number][1])
    cv2.putText(frame,text, 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        lineType)


    cv2.imshow("frame", frame)
    cv2.waitKey(3)
    pass

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    view_pickle(FILENAME)