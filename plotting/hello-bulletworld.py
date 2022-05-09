# ____    ___   ___       __   ____    ___   _____  _   .     ____  __  _     .
# ||  \  ||    ||  \     //    ||  \  ||  |     //  ||  |    //     ||  ||\  /|
# ||__/  ||__  ||__/    //     ||__/  ||__|    //   ||__|    \\__   ||  || \/ |
# ||  \  ||    ||       \\     || \   ||  |   //        |       \\  ||  ||    |
# ||__/  ||__  ||        \\__  ||  \  ||  |  //___  ||__|    ___//  ||  ||    |
#
# Authors: Pieter-Jan van Dolderen, Fabian Gebben, Ewout Schokker en Christiaan Theunisse

import pybullet as p
from time import sleep
from time import localtime, strftime
import pybullet_data
import csv
from PIL import Image

# In the following file is, among others things, the position written to a CSV file with the following format:
# Format of CSV file (first row is a header row)
# Timestamp | Position X | Pos Y | Pos Z | Attitude R | Att P | Att Y
# REMARK: It's important to store the information during the simulation in an array
# and write it later to a CSV file, because it's not possible to write that
# fast to a CSV file.
# The relevant parts for writing to a CSV file are highlighted by a row of hashtags

### Variables
mass = 0.027      # kg
### BUG: if g = 9.81 drone goes through the ground (with 1 it doesn't)
g = 9.81         # m/s^2

### Setup simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)
planeId = p.loadURDF("samurai.urdf")

### Start position and orientation
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("cf2x.urdf", cubeStartPos, cubeStartOrientation)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

### Print objectId
# print('-' * 52)
# print('|', "The objectId:", ' ' * (50 - 3 - len("The objectId:")),'|')
# print('|', boxId, ' ' * 46, '|')
# print('-' * 52)

### Print visual information about the object
# print(p.getVisualShapeData(1))

### Export data to CSV #####################################################################################################################
### It is also possible to create a new file based on the current time
# fileName = 'crazyFlie' + strftime("-%H-%M-%S-%d%b%Y", localtime()) + '.csv'
fileName = 'log.csv'          # Name of the CSV file
directory = 'logDirectory'    # Directory of the CSV file
header = ['Timestamp [s]', 'x [m]', 'y [m]', 'z [m]', 'roll [rad]', 'pitch [rad]', 'yaw [rad]']
print(fileName)
file = open((directory + '/' + fileName) , 'w')
fileWriter = csv.writer(file)
fileWriter.writerow(header)
### End of first part #######################################################################################################################

### Make the simulation start on a random input
### (so you can set your gui to the right perspective)
# input('Give some input to start...')

dt = 0.001             # 's
stepCount = 0
runTime = 4           # 's
force = mass * g      # N   gravitational force
torque = 0            # Nm
printInterval = 0.2   # 's

### Set the timestep
p.setPhysicsEngineParameter(fixedTimeStep=dt)

positionCSV = []
eulerRPYCSV = []
timeStampCSV = []
### Run the simulation
while stepCount <= runTime / dt:

  ### Obtain position and rotation ###########################################################################################################
  position, quaternion = p.getBasePositionAndOrientation(1)
  positionCSV.append(position)
  ### Convert quaternion to euler
  eulerRPY = p.getEulerFromQuaternion(quaternion)
  eulerRPYCSV.append(eulerRPY)
  timeStampCSV.append(stepCount * dt)
  ### End of second part ######################################################################################################################

  ### print position every printInterval
  if stepCount % (printInterval / dt)  == 0:
    ### Print the position and rotation
    print('-' * 200)
    print('POSITION: x =', round(position[0], 2), '[m] | y =', round(position[1], 2), '[m] | z =', round(position[2], 2), '[m]',
          ' ' * 20, 'ORIENTATION: roll =', round(eulerRPY[0], 2), '[rad] | pitch =', round(eulerRPY[1], 2), '[rad] | yaw =', round(eulerRPY[2], 2), '[rad]')

  ### Write a row with position and orientation to the csv file
  # timeStamp = stepCount * dt
  # roundedOutput = [timeStamp]
  # roundedOutput += [round(num, 5) for num in (position + eulerRPY)]
  # print(roundedOutput)
  # fileWriter.writerow(roundedOutput)

  ### Apply force
  p.applyExternalForce(1, -1, (0, 0, force * (1 / position[2])), ((eulerRPY[1] - 0.1)/2000, 0, 0), p.LINK_FRAME)

  ### Apply external torque
  # p.applyExternalTorque(1, -1, (0, 0, torque), p.WORLD_FRAME)

  ### Step the simulation
  p.stepSimulation()
  sleep(dt)

  ### Save image every 100 steps
  # if stepCount % 100 == 0:
  #   img = p.getCameraImage(1024, 1024)
  #   rgbBuffer = img[2]
  #   rgbImg = Image.fromarray(rgbBuffer)
  #   print('sim' + str(stepCount) + '.png')
  #   rgbImg.save(('sim' + str(stepCount) + '.png'))

  stepCount += 1
### End of simulation

### Write the arrays to a CSV file ############################################################################################################
for i in range(len(timeStampCSV)):
  roundedOutput = [timeStampCSV[i]]
  roundedOutput += [round(num, 5) for num in (positionCSV[i] + eulerRPYCSV[i])]
  fileWriter.writerow(roundedOutput)

### Close csv file
file.close()
### End of third part #########################################################################################################################

### Keep GUI running
# while (p.isConnected()):
#   sleep(1 / 100)


