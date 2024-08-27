import threading
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import json

global sims
sims = {}

def set_gripper(gripper_handle, open, velocity=0.15, force=50):
    sim = sims['UR5']
    if not open:
        velocity = -velocity
    
    data = {
        'velocity': velocity,
        'force': force
    }
    
    return_code = sim.writeCustomTableData(gripper_handle, 'activity', data)
    
    if return_code == -1:
        print("Failed to write data")
    else:
        print("Data written successfully")

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

def ur5_control():
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

    maxVel = [vel] * 7
    maxAccel = [accel] * 7
    maxJerk = [jerk] * 7

    for i in range(1):
        targetPos1 = [90*math.pi/180, 0, 90*math.pi/180, 90*math.pi/180, 90*math.pi/180, 90*math.pi/180]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos1)
        print('Target 1 completed')
        
                
        targetPos2 = [90*math.pi/180, 35 * math.pi / 180, 55*math.pi/180, 90*math.pi/180, 90*math.pi/180, 90*math.pi/180]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos2)
        
        # Open the gripper
        set_gripper(gripper_handle, False)

        print('Target 2 completed')
        
        targetPos20 = [90*math.pi/180, 35 * math.pi / 180, 55*math.pi/180, 90*math.pi/180, 90*math.pi/180, 90*math.pi/180]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos20)
        
        # Open the gripper
        set_gripper(gripper_handle, False)

        print('Target 20 completed')
        
        targetPos3 = [90*math.pi/180, 0, 90*math.pi/180, 90*math.pi/180, 90*math.pi/180, 90*math.pi/180]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos3)
        print('Target 3 completed')

    sim.setStepping(False)
    print("UR5 control completed")

def main():
    print('Program started')
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.startSimulation()
    
    try:
        while True:
            print("Waiting for navigation signal...")
            while True:
                nav_signal = sim.getIntegerSignal("nav_completed")
                if nav_signal == 1:
                    print("Navigation task completed. Executing robotic arm control.")
                    ur5_thread = threading.Thread(target=ur5_control)
                    ur5_thread.start()
                    ur5_thread.join()  # Wait for UR5 control to complete
                    sim.setIntegerSignal("nav_completed", 0)  # Reset the signal
                    print("Robotic arm control completed. Ready for next cycle.")
                    break  # Exit the inner loop and wait for the next navigation signal
                sim.step()  # Step the simulation
            
    except KeyboardInterrupt: 
        print("Simulation interrupted by user.")
    finally:
        sim.stopSimulation()

    print('Program ended')

if __name__ == '__main__':
    main()