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
        
    targetHandle = sim.getObject('./ikTarget')

    vel = 0.5
    accel = 0.1
    jerk = 3

    maxVel = [vel, vel, vel, vel]
    maxAccel = [accel, accel, accel, accel * 10]
    maxJerk = [jerk, jerk, jerk, jerk]
    
    initTr = sim.getObjectPose(targetHandle)

    for i in range(1):
        goalTr = initTr.copy()
        goalTr[2] = goalTr[2] + 0.2
        sim.moveToPose(-1, initTr, maxVel, maxAccel, maxJerk, goalTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})
        print('Pose 1 completed')
        
        startTr = sim.getObjectPose(targetHandle)
        goalTr[2] = goalTr[2] - 0.2
        sim.moveToPose(-1, startTr, maxVel, maxAccel, maxJerk, goalTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})
        print('Pose 2 completed')
        
        startTr = sim.getObjectPose(targetHandle)
        goalTr = sim.rotateAroundAxis(goalTr, [1, 0, 0], [startTr[0], startTr[1], startTr[2]], 90 * math.pi / 180)
        sim.moveToPose(-1, startTr, maxVel, maxAccel, maxJerk, goalTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})
        print('Pose 3 completed')
        
        startTr = sim.getObjectPose(targetHandle)
        sim.moveToPose(-1, startTr, maxVel, maxAccel, maxJerk, initTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})
    
    sim.setStepping(False)
    
def poseCallback(tr, vel, accel, data):
    sim = sims[data['robotColor']]
    handle = data['handle']
    sim.setObjectPose(handle, tr)


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



blueRobotThread = threading.Thread(target=blueRobot) # Just to init the target?


blueRobotThread.start() # initialise or start the thread?

sim.startSimulation()


blueRobotThread.join() # Officially starts the thread


sim.stopSimulation()

print('Program ended')


