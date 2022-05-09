# ____    ___   ___       __   ____    ___   _____  _   .     ____  __  _     .
# ||  \  ||    ||  \     //    ||  \  ||  |     //  ||  |    //     ||  ||\  /|
# ||__/  ||__  ||__/    //     ||__/  ||__|    //   ||__|    \\__   ||  || \/ |
# ||  \  ||    ||       \\     || \   ||  |   //        |       \\  ||  ||    |
# ||__/  ||__  ||        \\__  ||  \  ||  |  //___  ||__|    ___//  ||  ||    |
#
# Authors: Pieter-Jan van Dolderen, Fabian Gebben, Ewout Schokker en Christiaan Theunisse

import pybullet as p
from time import sleep
import pybullet_data
import csv

# Script to replay a simulation in pybullet from a CSV file with the following format:
# Format of CSV file (first row is a header row)
# Timestamp | Position X | Pos Y | Pos Z | Attitude R | Att P | Att Y

### Setup simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
planeId = p.loadURDF("samurai.urdf")

### Start position and orientation
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("cf2x.urdf", cubeStartPos, cubeStartOrientation)

### Open the csv file
fileName = 'log.csv'          # Name of the CSV file
directory = 'logDirectory'    # Directory of the CSV file
dt = 0
timeFactor = 10

### Store the csv in an array
rowCount = 0
position = []
attitudeQ = []
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
            attitudeYPR = rowFloat[4:7]
            attitudeQ.append(p.getQuaternionFromEuler(attitudeYPR))
        rowCount += 1

### Make the simulation start on a random input
### (so you can set the gui to the right perspective)
input('Give some input to start...')

### factor to slowdown replay
slowdown = 1

for i in range(len(position)):
    p.resetBasePositionAndOrientation(1, position[i], attitudeQ[i])
    sleep(dt * slowdown)




