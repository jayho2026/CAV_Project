import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import threading
import time

global sims, simIK
sims = {}

def set_gripper_data(gripper_handle, open, velocity=0.11, force=20):
    sim = sims['UR5']
    if not open:
        velocity = -velocity
    
    data = {
        'velocity': velocity,
        'force': force
    }
    
    sim.writeCustomTableData(gripper_handle, 'activity', data)

def solve_ik(sim_tip, sim_target, ik_group, ik_env):
    sim = sims['UR5']
    # This line solves the ik problem, the syncWorlds synch the ik world with the simulation world
    result = simIK.handleGroup(ik_env, ik_group, {'syncWorlds': True})
    
    if result:
        # retrieves the handles of the joints involved in the IK calculation
        joint_handles = simIK.getGroupJoints(ik_env, ik_group)
        joint_positions = [simIK.getJointPosition(ik_env, handle) for handle in joint_handles]
        return True, joint_positions
    else:
        return False, []
    
    
def set_joint_positions(joint_handles, joint_positions):
    sim = sims['UR5']
    for handle, position in zip(joint_handles, joint_positions):
        sim.setJointPosition(handle, position)
      
def confCallback(config, vel, accel, data):
    sim = sims[data['robotColor']]
    handles = data['handles']
    for i in range(len(handles)):
        # Checks whether a scene object is dynamically enabled, i.e. is being handled and simulated by the physics engine.
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


def ur5_ik_control():
    robotColor = 'UR5'
    client = RemoteAPIClient()
    sim = client.require('sim')
    global simIK
    simIK = client.require('simIK')
    sims['UR5'] = sim
    sim.setStepping(False)

    # Initialize handles and IK environment
    gripper_handle = sim.getObject('./RG2')
    sim_tip = sim.getObject('./ikTip')
    sim_target = sim.getObject('/ikTarget')
    model_base = sim.getObject('/UR5')

    jointHandles = []
    for i in range(6):
        jointHandles.append(sim.getObject('/' + 'UR5' + '/joint', {'index': i}))

    # Create IK environment and group
    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)
    simIK.addElementFromScene(ik_env, ik_group, model_base, sim_tip, sim_target, simIK.constraint_pose)

    # Open the gripper
    set_gripper_data(gripper_handle, True)

    # Main control loop
    try:
        for x in range(3):
            # Solve IK
            success, joint_positions = solve_ik(sim_tip, sim_target, ik_group, ik_env)
            
            if success:  # IK solved successfully
                print(f"IK solved. Joint positions: {joint_positions}")
                # Apply joint positions to the actual robot
                for i, joint_handle in enumerate(jointHandles):
                    sim.setJointTargetPosition(joint_handle, joint_positions[i])
            else:
                print("Failed to solve IK")

            # Example movement of the target dummy
            current_pose = sim.getObjectPose(sim_target)
            current_pose[1] = current_pose[1] + 0.1  # Move slightly in x direction
            sim.setObjectPose(sim_target, current_pose)
            
            print("Update")

            #sim.step()  # Step the simulation
            time.sleep(1)  # Small delay to control the update rate

    except KeyboardInterrupt:
        print("Control loop interrupted")

    sim.setStepping(False)

def main():
    print('Program started')
    client = RemoteAPIClient()
    sim = client.require('sim')

    ur5_thread = threading.Thread(target=ur5_ik_control)
    ur5_thread.start()

    sim.startSimulation()
    
    try:
        ur5_thread.join()
    except KeyboardInterrupt: 
        print("Simulation interrupted by user.")
    finally:
        sim.stopSimulation()

    print('Program ended')

if __name__ == '__main__':
    main()