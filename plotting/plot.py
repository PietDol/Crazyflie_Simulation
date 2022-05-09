# ____    ___   ___       __   ____    ___   _____  _   .     ____  __  _     .
# ||  \  ||    ||  \     //    ||  \  ||  |     //  ||  |    //     ||  ||\  /|
# ||__/  ||__  ||__/    //     ||__/  ||__|    //   ||__|    \\__   ||  || \/ |
# ||  \  ||    ||       \\     || \   ||  |   //        |       \\  ||  ||    |
# ||__/  ||__  ||        \\__  ||  \  ||  |  //___  ||__|    ___//  ||  ||    |
#
# Authors: Pieter-Jan van Dolderen, Fabian Gebben, Ewout Schokker en Christiaan Theunisse

import csv
import numpy as np
import matplotlib.pyplot as plt
import imageio

# Plot the position and orientation from a CSV file
# Format of CSV file (first row is a header row)
# Timestamp | Position X | Pos Y | Pos Z | Attitude R | Att P | Att Y

### Open the csv file
fileName = 'log.csv'            # Name of the CSV file
directory = 'logDirectory'      # Directory of the CSV file

rowCount = 0
position = []
attitudeRPY = []
with open((directory + '/' + fileName)) as csvFile:
    fileReader = csv.reader(csvFile, delimiter=',')
    for row in fileReader:
        ### Obtain dt
        if rowCount == 2:
            dt = float(row[0])
        ### Leave out header row
        if rowCount == 0:
            pass
        else:
            rowFloat = [float(row[i]) for i in range(len(row))]
            position.append(rowFloat[1:4])
            attitudeRPY.append(rowFloat[4:7])
        rowCount += 1

### Plot z-position relative to x-position with a line segment representing the drones orientation
segmentLength = 0.2             # -     The length of the lines, representing the drone, in the plot
horAxisRange = [0, 8]           # m     The horizontal axis range
vertAxisRange = [0.7, 1.3]      # m     The vertical axis range
ratePlot = 5                    # Hz    The rate of lines drawn in the plot
rateAnimation = 10             # Hz    The rate of the animation saved as a GIF

plt.figure(1)
timeStamps = []
reducedX = []
reducedY = []
reducedZ = []
reducedPitch = []
for i in range(len(position)):
    timeStamps.append(i * dt)
    reducedX.append(position[i][0])
    reducedY.append(position[i][1])
    reducedZ.append(position[i][2])
    reducedPitch.append(-attitudeRPY[i][1])     # Keep this (-) in mind

lines = []
horRangeLength = horAxisRange[1] - horAxisRange[0]
vertRangeLength = vertAxisRange[1] - vertAxisRange[0]
horSegment = segmentLength
vertSegment = segmentLength * vertRangeLength / horRangeLength

for i in range(len(reducedX)):
    if i % (1/(dt * ratePlot)) == 0:
        x = [reducedX[i] - horSegment*np.cos(reducedPitch[i]), reducedX[i] + horSegment*np.cos(reducedPitch[i])]
        y = [reducedZ[i] - vertSegment*np.sin(reducedPitch[i]), reducedZ[i] + vertSegment*np.sin(reducedPitch[i])]
        plt.plot(x, y, marker='o', markersize=2, color='blue', linestyle="-", linewidth=1)

plt.xlim(horAxisRange)
plt.ylim(vertAxisRange)
plt.xlabel("Position along x-axis [m]")
plt.ylabel("Height [m]")
plt.title("CrazyFlie position (x vs. z) and attitude (pitch \u03F4)")
plt.savefig('logDirectory/plot.png', dpi=300)
# plt.grid()

plt.show()

### Plot z-position relative to x-position with a line segment representing the drones orientation
### In a animation stored as a GIF
plt.figure(2)
filenames = []
directory = "logDirectory/gif"      # Directory where the PNGs are stored
for i in range(len(reducedX)):
    if i % int(1/(dt * rateAnimation)) == 0:
        x = [reducedX[i] - horSegment*np.cos(reducedPitch[i]), reducedX[i] + horSegment*np.cos(reducedPitch[i])]
        y = [reducedZ[i] - vertSegment*np.sin(reducedPitch[i]), reducedZ[i] + vertSegment*np.sin(reducedPitch[i])]

        plt.xlim(horAxisRange)
        plt.ylim(vertAxisRange)
        plt.xlabel("Position along x-axis [m]")
        plt.ylabel("Height [m]")
        plt.title("CrazyFlie position (x vs. z) and attitude (pitch \u03F4)")
        plt.plot(x, y, marker='o', markersize=2, color='blue', linestyle="-", linewidth=1)

        # create filename and append it to a list
        filename = f'{i}.png'
        filenames.append(filename)

        # save frame
        plt.savefig(directory + '/' + filename)
        plt.clf()

# build gif
frames = []
for filename in filenames:
    frames.append(imageio.imread(directory + '/' + filename))


exportName = "animation.gif"        # Filename of the GIF
directory = "logDirectory"          # Directory where the GIF is stored
imageio.mimsave(directory + '/' + exportName, frames, format='GIF', fps=rateAnimation)
