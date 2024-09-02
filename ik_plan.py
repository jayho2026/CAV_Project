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
 

def moveToPose_viaIK(sim, simIK, maxVelocity, maxAcceleration, maxJerk, targetQ, auxData):
    currentQ = sim.getObjectPose(auxData['tip'])
    return sim.moveToPose(-1, currentQ, maxVelocity, maxAcceleration, maxJerk, targetQ, moveToPoseCallback, auxData, None)

def moveToPoseCallback(q, velocity, accel, auxData):
    sim = sims['UR5']
    # Updating target pose during the movement
    sim.setObjectPose(auxData['target'], q)
    simIK.handleGroup(auxData['ikEnv'], auxData['ikGroup'], {'syncWorlds': True})

    
    
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

def simple_path_planning(start_pose, end_pose, num_steps):
    path = []
    for i in range(num_steps + 1):
        t = i / num_steps
        intermediate_pose = [
            start_pose[j] + t * (end_pose[j] - start_pose[j])
            for j in range(len(start_pose))
        ]
        path.append(intermediate_pose)
    return path

def ur5_ik_control():
    robotColor = 'UR5'
    client = RemoteAPIClient()
    sim = client.require('sim')
    global simIK
    simIK = client.require('simIK')
    sims['UR5'] = sim
    sim.setStepping(True)

    # Initialize handles and IK environment
    gripper_handle = sim.getObject('./RG2')
    sim_tip = sim.getObject('./ikTip')
    sim_target = sim.getObject('./ikTarget')
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
    
    # Set up auxiliary data
    auxData = {
        'tip': sim_tip,
        'target': sim_target,
        'ikEnv': ik_env,
        'ikGroup': ik_group
    }
    
    # Movement parameters
    maxVelocity = [0.4,0.4,0.4,1.8]  # m/s
    maxAcceleration = [0.8,0.8,0.8,0.9]  # m/s^2
    maxJerk = [0.6,0.6,0.6,0.8]  # m/s^3

    # Main control loop
    try:
         # Get initial pose of the tip
        initial_tip_pose = sim.getObjectPose(sim_tip)
        
        # Set initial target pose to match the tip pose
        sim.setObjectPose(sim_target, initial_tip_pose)

        # Example movement of the target dummy
        current_pose = sim.getObjectPose(sim_target)
        target_pose = current_pose.copy()
        target_pose[0] = current_pose[0] - 0.5  # Move 0.2m in Z direction

        print("Planning path...")
        path = simple_path_planning(current_pose, target_pose, 20)  # Break into 50 steps

        print("Moving to target position...")

        for step, pose in enumerate(path):
            movement_in_progress = True
            sim.setObjectPose(sim_target, pose)
            
            while movement_in_progress:
                result = moveToPose_viaIK(sim, simIK, maxVelocity, maxAcceleration, maxJerk, pose, auxData)
                movement_in_progress = result[0] is None  # If result[0] is None, movement is still in progress
                sim.step()
                time.sleep(0.01)  # Small delay to control the update rate
            
            print(f"Step {step + 1}/{len(path)} completed")

        print("Movement completed!")
        
        # Final pose check
        final_pose = sim.getObjectPose(sim_tip)
        distance = math.sqrt(sum((a-b)**2 for a, b in zip(final_pose[:3], target_pose[:3])))
        print(f"Final distance to target: {distance:.4f} m")
        
        #time.sleep(0.1)  # Small delay to control the update rate

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