# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script

import threading
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import json

global sims
sims = {}

def set_gripper(gripper_handle, open, velocity=0.11, force=20):
    sim = sims['UR5']
    if not open:
        velocity = -velocity
    
    data = {
        'velocity': velocity,
        'force': force
    }
    
    #packed_data = json.dumps(data)  # Serialize the dictionary into a JSON formatted string
    return_code = sim.writeCustomTableData(gripper_handle, 'activity', data)
    
    if return_code == -1:
        print("Failed to write data")
    else:
        print("Data written successfully")

def blueRobot():
    robotColor = 'UR5'
    client = RemoteAPIClient()
    sim = client.require('sim')
    sims[robotColor] = sim 
    sim.setStepping(True)
    
    gripper_handle = sim.getObject('./RG2')
    
    set_gripper(gripper_handle, True)

    jointHandles = []
    for i in range(6):
        jointHandles.append(sim.getObject('/' + robotColor + '/joint', {'index': i}))

    vel = 110 * math.pi / 180
    accel = 40 * math.pi / 180
    jerk = 80 * math.pi / 180

    maxVel = [vel, vel, vel, vel, vel, vel, vel]
    maxAccel = [accel, accel, accel, accel, accel, accel, accel]
    maxJerk = [jerk, jerk, jerk, jerk, jerk, jerk, jerk]

    for i in range(1):
        targetPos1 = [0, -90 * math.pi / 180, 0, 0, 0, 0, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos1)
        print('Target 1 completed')
        
        targetPos2 = [0, 90 * math.pi / 180, -10 * math.pi / 180, 0, 0, 0, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos2)
        
        # Open the gripper
        set_gripper(gripper_handle, False)

        print('Target 2 completed')
        targetPos3 = [0, 0, 0, 0, 0, 0, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos3)
        print('Target 3 completed')

    sim.setStepping(False)




def confCallback(config, vel, accel, data):
    sim = sims[data['robotColor']]
    handles = data['handles']
    for i in range(len(handles)):
        if sim.isDynamicallyEnabled(handles[i]):
            sim.setJointTargetPosition(handles[i], config[i])
        else:
            sim.setJointPosition(handles[i], config[i])

def moveToConfig(robotColor, handles, maxVel, maxAccel, maxJerk, targetConf):
    sim = sims[robotColor]
    currentConf = []
    for i in range(len(handles)):
        currentConf.append(sim.getJointPosition(handles[i]))
    sim.moveToConfig(-1, currentConf, None, None, maxVel, maxAccel, maxJerk, targetConf, None, confCallback, {'robotColor': robotColor, 'handles': handles}, None)

print('Program started')
client = RemoteAPIClient()
sim = client.require('sim')



blueRobotThread = threading.Thread(target=blueRobot)


blueRobotThread.start()


sim.startSimulation()

blueRobotThread.join()


sim.stopSimulation()

print('Program ended')


