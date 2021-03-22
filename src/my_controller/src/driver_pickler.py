# Used for saving driving data to files
import pickle
import numpy as np

SAVE_FOLDER = "driving_data/"


class driverPickler:
    def __init__(self):
        self.folder = SAVE_FOLDER

    def save_pickle(self, filename, frames, speeds):
        with open(self.folder + filename + ".pickle", 'wb') as f:
            pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

    # returns a tuple of type ([RGB image numpy array], [target_lin, target_ang])
    def load_pickle(self, filename):
        try:
            with open(self.folder + filename + ".pickle", 'rb') as f:
                data = pickle.load(f)
            print("Loaded file: {}".format(filename + ".pickle"))
        except IOError:
            print("Invalid pickle file.")

        return (np.array(data[0]), data[1])




        # 