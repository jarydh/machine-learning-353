# Used for saving driving data to files
import pickle
import numpy as np


SAVE_FOLDER = "/home/fizzer/machine-learning-353/src/my_controller/src/lesley_driving_data/"

class driverPickler:
    def __init__(self):
        self.folder = SAVE_FOLDER

    def save_pickle(self, filename, frames, speeds):
        with open(self.folder + filename + "_imgs" + ".pickle", 'wb') as f:
            pickle.dump(frames, f, pickle.HIGHEST_PROTOCOL)
        print("Saved file: " + filename + "_imgs.pickle")

        with open(self.folder + filename + "_speeds" + ".pickle", 'wb') as f:
            pickle.dump(speeds, f, pickle.HIGHEST_PROTOCOL)
        print("Saved file: " + filename + "_speeds.pickle")


    # returns a tuple of type ([RGB image numpy array], [target_lin, target_ang])
    def load_pickle(self, filename):
        try:
            with open(self.folder + filename + "_imgs" + ".pickle", 'rb') as f:
                frames = pickle.load(f)
            print("Loaded file: {}".format(filename + "_imgs.pickle"))
        except IOError:
            print("Invalid pickle file.")

        try:
            with open(self.folder + filename + "_speeds" + ".pickle", 'rb') as f:
                speeds = pickle.load(f)
            print("Loaded file: {}".format(filename + "_speeds.pickle"))
        except IOError:
            print("Invalid pickle file.")

        return (np.array(frames), speeds)

