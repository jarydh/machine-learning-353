# for manually driving the car around

import numpy as np


import driver_controller as dc

# number of frames for each
LOOP_TRANSITION_FRAMES = [2, 18, 1, 3]

# must be same length
LOOP_TRANSITION_SPEEDS = [(dc.LIN_SLOW, dc.ANG_STRAIGHT),
                          (dc.LIN_SLOW, dc.ANG_LEFT),
                          (dc.LIN_SLOW, dc.ANG_STRAIGHT),
                          (dc.LIN_STOP, dc.ANG_STRAIGHT)]

# number of frames for each
LEFT_TURN_FRAMES = [8, 18, 2]

# must be same length
LEFT_TURN_SPEEDS = [(dc.LIN_SLOW, dc.ANG_STRAIGHT),
                          (dc.LIN_SLOW, dc.ANG_LEFT),
                          (dc.LIN_SLOW, dc.ANG_STRAIGHT)]


class manualDriver:
    def __init__(self):
        self.loop_transition_count = 0
        self.left_turn_count = 0

        self.loop_transition_last_frame = []
        for i in range(0, len(LOOP_TRANSITION_FRAMES)):
            self.loop_transition_last_frame.append(sum(LOOP_TRANSITION_FRAMES[0:i + 1]))

        self.left_turn_last_frame = []
        for i in range(0, len(LEFT_TURN_FRAMES)):
            self.left_turn_last_frame.append(sum(LEFT_TURN_FRAMES[0:i + 1]))

    # returns lin_speed, ang_speed, is_done  
    def loop_transition(self):
        for index, next_frame_boundary in enumerate(self.loop_transition_last_frame):
            if self.loop_transition_count < next_frame_boundary:
                lin_speed = LOOP_TRANSITION_SPEEDS[index][0]
                ang_speed = LOOP_TRANSITION_SPEEDS[index][1]
                break

        self.loop_transition_count += 1
        is_done = self.loop_transition_count == self.loop_transition_last_frame[-1]
        if is_done:
            self.loop_transition_count = 0
        return (lin_speed, ang_speed, is_done)

    def left_turn(self):
        for index, next_frame_boundary in enumerate(self.left_turn_last_frame):
            if self.left_turn_count < next_frame_boundary:
                lin_speed = LEFT_TURN_SPEEDS[index][0]
                ang_speed = LEFT_TURN_SPEEDS[index][1]
                break

        self.left_turn_count += 1
        is_done = self.left_turn_count == self.left_turn_last_frame[-1]
        if is_done:
            self.left_turn_count = 0
        return (lin_speed, ang_speed, is_done)