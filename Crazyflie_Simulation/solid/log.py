import pandas as pd
import numpy as np
from time import localtime, strftime
import matplotlib.pyplot as plt


class Log:
    def __init__(self, unique_file: bool):
        self.unique_file = unique_file
        self.rate = 0
        self.directory = "../Crazyflie_Simulation/solid/Logging/"
        self.filename = "log.csv" if not unique_file else ("log" + strftime("-%H-%M-%S-%d%b%Y", localtime()) + ".csv")
        self.data = []
        self.i = 0
        self.runs = []

    def add_data(self, position, orientation, run_id: int, rate: int, timestamp=None):
        # position x, y, z
        # orientation roll, pitch, yaw
        # run -> should be a unique integer
        self.rate = rate

        # Check if the run index is new
        if run_id not in self.runs:
            self.runs.append(run_id)
            self.i = 0
            print(self.runs)

        # Calculate timestamp
        if timestamp is None:
            time = np.round(self.i / self.rate, 3)
        else:
            time = timestamp

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


class Analyse:
    def __init__(self, filename: str, directory="../Crazyflie_Simulation/solid/Logging/"):
        self.filename = filename
        self.directory = directory
        self.df = self.import_dataset(path=directory + filename)
        self.runs = int(len(self.df.loc[1]) / 7)
        self.run_numbers = [int(self.df.columns.values.tolist()[7*i]) for i in range(self.runs)]
        self.temp_df = "To be assigned"
        self.diff = "To be assigned"
        self.diff_dict = {}
        self.mode_dict = {
            "x": 1,
            "y": 2,
            "z": 3,
            "roll": 4,
            "pitch": 5,
            "yaw": 6
        }

    @staticmethod
    def import_dataset(path: str):
        df = pd.read_csv(path)
        df = df[1:].astype(float)
        return df

    def calculate_differences(self, id: int, run1=1, run2=2):
        if run1 and run2 in self.run_numbers:
            diff = [self.df[str(run1)]]
            for i in range(1, 7):
                diff.append(self.df[str(run1) + "." + str(i)] - self.df[str(run2) + "." + str(i)])

            self.diff = diff
            self.diff_dict[id] = diff
        else:
            raise KeyError(f"Choose a run from the following options: {self.run_numbers}")

    def plot_graph(self, mode: str, id: int):
        try:
            mode_nr = self.mode_dict[mode]
        except KeyError:
            raise KeyError(f"Give one of the following modes as input: {[key for key in self.mode_dict.keys()]}")
        # output = self.diff[mode_nr]
        try:
            output = self.diff_dict[id][mode_nr]
        except KeyError:
            raise KeyError(f"Give one of the following integers as input: {[key for key in self.diff_dict.keys()]}")
        time = self.diff[0]
        plt.plot(time, output)

        plt.title(f"{mode} error vs. time ({id})")
        plt.xlabel("Time [s]")

        if mode_nr <= 3:
            unit = "m"
        else:
            unit = "deg"

        plt.ylabel(f"{mode} error [{unit}]")

        plt.show()

# df["index"] gives columns
# df.loc[index] gives rows
