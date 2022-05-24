import pandas as pd
import numpy as np
from time import localtime, strftime


class Log:
    def __init__(self, unique_file: bool, rate):
        self.unique_file = unique_file
        self.rate = rate
        self.dt = 1 / rate
        self.directory = ""
        self.filename = "log.csv" if not unique_file else ("log" + strftime("-%H-%M-%S-%d%b%Y", localtime()) + ".csv")
        self.data = []
        self.i = 0
        self.runs = []

    def add_data(self, position, orientation, run_id: int):
        # position x, y, z
        # orientation roll, pitch, yaw
        # run -> should be a unique integer

        # Check if the run index is new
        if run_id not in self.runs:
            self.runs.append(run_id)
            self.i = 0
            print(self.runs)

        # Calculate timestamp
        time = np.round(self.i / self.rate, 3)

        try:
            empty_cols = 7 * (len(self.runs) - 1) - len(self.data[self.i])
            self.data[self.i] += [np.nan] * empty_cols + [time, position[0], position[1], position[2],
                                                       orientation[0], orientation[1],
                                                       orientation[2]]
        except:
            self.data.append(
                [np.nan] * (7 * (len(self.runs) - 1)) + [time, position[0], position[1], position[2], orientation[0],
                                                      orientation[1], orientation[2]])
        self.i += 1

    def save_to_csv(self):
        self.data.insert(0, len(self.runs) * ["Time", "Pos x", "Pos y", "Pos z", "Roll", "Pitch", "Yaw"])
        id_header = []
        for r in self.runs:
            id_header += 7 * [r]

        self.data.insert(0, id_header)
        df = pd.DataFrame(self.data)
        df.to_csv(self.directory + self.filename, index=False, header=False)
