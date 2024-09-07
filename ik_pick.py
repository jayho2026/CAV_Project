# This is the inverse kinematic for picking the object and holding it on air

import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import threading
import time

global sims, simIK
sims = {}

def set_gripper_data(gripper_handle, open, velocity=0.15, force=60):
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

def calculate_front_position(object_handle, distance=0.2):
    sim = sims['UR5']
    object_position = sim.getObjectPosition(object_handle, -1)
    #object_orientation = sim.getObjectOrientation(object_handle, -1)
    
    # Calculate the front direction of the object (assuming +X is front)
    front_vector = sim.getObjectMatrix(object_handle, -1)[0:3]
    
    # Calculate the position in front of the object
    front_position = [
        object_position[0] + front_vector[0] * distance,
        object_position[1] + front_vector[1] * distance,
        object_position[2] + front_vector[2] * distance
    ]
    
        
    return front_position 

def flat_gripper_quaternion():
    # This quaternion represents a rotation of -90 degrees around the X-axis
    angle = -math.pi / 2
    return [0.707, 0, 0, 1.407]

def calculate_gripper_orientation(object_handle):
    sim = sims['UR5']
    # Get the object's position
    #object_position = sim.getObjectPosition(object_handle, -1)
    
    # Calculate a flat gripper orientation
    flat_orientation = flat_gripper_quaternion()
    
    return flat_orientation

def reorient_gripper(sim, simIK, desired_orientation, sim_tip, sim_target, ik_env, model_base, joint_handles):
    # Create a new IK group for orientation only
    orient_ik_group = simIK.createGroup(ik_env)
    simIK.addElementFromScene(ik_env, orient_ik_group, model_base, sim_tip, sim_target, simIK.constraint_orientation)
    
    # Get current position
    current_pose = sim.getObjectPose(sim_tip)
    
    # Create target pose with current position and desired orientation
    target_pose = current_pose[:3] + desired_orientation
    
    # Set the target pose
    sim.setObjectPose(sim_target, target_pose)
    
    # Solve IK for the new orientation
    result, joint_positions = solve_ik(sim_tip, sim_target, orient_ik_group, ik_env)
    if result:
        set_joint_positions(joint_handles, joint_positions)
        print("Gripper reoriented successfully")
    else:
        print("Failed to reorient gripper")
    
    

def pick_object(sim, simIK, object_name, gripper_handle, sim_tip, sim_target, ik_env, ik_group, auxData):
    # Get object position
    object_handle = sim.getObject(object_name)
    object_position = sim.getObjectPosition(object_handle)
    object_pose = sim.getObjectPose(object_handle)

    # Calculate approach pose (position and orientation)
    approach_position = calculate_front_position(object_handle)
    
    approach_pose = sim.getObjectPose(sim_target)
    approach_pose = object_pose.copy()
    approach_pose[0] += 0.2
    move_to_pose(sim, simIK, approach_pose, sim_target, auxData)
    
    # Move to approach pose
    #move_to_pose(sim, simIK, approach_pose, sim_target, auxData)
    
    print("Approaching object!")
    
    # Calculate and apply the desired orientation
    desired_orientation = calculate_gripper_orientation(object_handle)
    orient_ik_group = simIK.createGroup(ik_env)
    simIK.addElementFromScene(ik_env, orient_ik_group, auxData['modelBase'], sim_tip, sim_target, simIK.constraint_position)
    
    # Get current position
    current_pose = sim.getObjectPose(sim_tip)
    
    # Create target pose with current position and desired orientation
    desired_pose = current_pose[:3] + desired_orientation
    print("Reoriented gripper. Current gripper pose:")
    print(sim.getObjectPose(sim_tip))
    
    move_to_pose(sim, simIK, desired_pose, sim_target, auxData)
    
   
    # Set the target to the grab pose
    #sim.setObjectPose(sim_target, grab_pose)
    
    # Move down to grab the object
    grab_pose = object_pose.copy()
    grab_pose[2] = object_pose[2] - 0  # Adjust height slightly below the object
    move_to_pose(sim, simIK, grab_pose, sim_target, auxData)

    print("Moved to grab pose. Current gripper pose:")
    print(sim.getObjectPose(sim_tip))
    
    # Close gripper
    set_gripper_data(gripper_handle, False)
    time.sleep(2)  # Wait for gripper to close
    
    # Lift object
    lift_position = object_pose.copy()
    lift_position[2] += 0.3  # Lift 10cm
    move_to_pose(sim, simIK, lift_position, sim_target, auxData)
    
    


def move_to_pose(sim, simIK, target_pose, sim_target, auxData):
    #current_pose = sim.getObjectPose(sim_target)
    #target_pose = current_pose.copy()
    #target_pose[:3] = position  # Update only the position, keep the orientation
    
    maxVelocity = [0.4, 0.4, 0.4, 1.8]
    maxAcceleration = [0.8, 0.8, 0.8, 0.9]
    maxJerk = [0.6, 0.6, 0.6, 0.8]
    
    movement_in_progress = True
    while movement_in_progress:
        sim.setObjectPose(sim_target, target_pose)
        result = moveToPose_viaIK(sim, simIK, maxVelocity, maxAcceleration, maxJerk, target_pose, auxData)
        movement_in_progress = result[0] is None
        sim.step()
        time.sleep(0.01)



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
    goal_object = sim.getObject('/Cuboid')
    #rover = sim.getObject('/PioneerP3DX')

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
        'ikGroup': ik_group,
        'modelBase': model_base,
        'jointHandles': jointHandles,
    }
    
   

    # Main control loop
    try:
        # Get initial pose of the tip
        initial_tip_pose = sim.getObjectPose(sim_tip)
        
        # Set initial target pose to match the tip pose
        sim.setObjectPose(sim_target, initial_tip_pose)

         # Move to a starting position
        initial_position = initial_tip_pose  # Example starting position
        move_to_pose(sim, simIK, initial_position, sim_target, auxData)
        
        # Pick up the object
        pick_object(sim, simIK, '/Cuboid', gripper_handle, sim_tip, sim_target, ik_env, ik_group, auxData)
        
        # Move to a drop-off position
        #rover_pose = sim.getObjectPose(rover)  # Example drop-off position
        #beside_rover = rover_pose.copy()
        #beside_rover -= 0.3
        #move_to_position(sim, simIK, beside_rover, sim_target, auxData)
        
        # Open gripper to release object
        set_gripper_data(gripper_handle, False)
        
        print("Pick and place completed!")

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
        input()
        sim.stopSimulation()

    print('Program ended')

if __name__ == '__main__':
    main()