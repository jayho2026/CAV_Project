# This is to signal and control the robotic arm to pick up the object after the ROS2 Nav2 is completed

import math
import threading
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

global sims, simIK
sims = {}

global pick_flag
pick_flag = True


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
    result = simIK.handleGroup(ik_env, ik_group, {'syncWorlds': True})
    
    if result:
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
    sim.setObjectPose(auxData['target'], q)
    simIK.handleGroup(auxData['ikEnv'], auxData['ikGroup'], {'syncWorlds': True})

def set_joint_positions(joint_handles, joint_positions):
    sim = sims['UR5']
    for handle, position in zip(joint_handles, joint_positions):
        sim.setJointPosition(handle, position)

def move_to_pose(sim, simIK, target_pose, sim_target, auxData):
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


def check_distance(sim, robot_handle, object_handle, threshold=0.8):
    robot_position = sim.getObjectPosition(robot_handle, -1)
    object_position = sim.getObjectPosition(object_handle, -1)
    
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(robot_position, object_position)))
    print(f"Distance to object: {distance} meters")
    return distance <= threshold

def pick_object(sim, simIK, object_name, gripper_handle, sim_tip, sim_target, ik_env, ik_group, auxData):
    object_handle = sim.getObject(object_name)
    object_pose = sim.getObjectPose(object_handle)

    print("Approaching object!")
    # Approach pose
    approach_pose = object_pose.copy()
    approach_pose[2] += 0.2     # on top of the object
    move_to_pose(sim, simIK, approach_pose, sim_target, auxData)
    
    
    ik_env2 = simIK.createEnvironment()
    orient_ik_group = simIK.createGroup(ik_env2)
    simIK.addElementFromScene(ik_env2, orient_ik_group, auxData['modelBase'], sim_tip, sim_target, simIK.constraint_pose)
    
    print("Grabbing object")
    # Grab pose
    grab_pose = object_pose.copy()
    grab_pose[2] -= 0.01  # Adjust height slightly below the object
    move_to_pose(sim, simIK, grab_pose, sim_target, auxData)

    # Close gripper
    set_gripper_data(gripper_handle, False)
    #time.sleep(2)  # Wait for gripper to close
    
    # Lift object
    lift_pose = grab_pose.copy()
    lift_pose[2] += 0.3  # Lift 30cm
    move_to_pose(sim, simIK, lift_pose, sim_target, auxData)

def drop_object(sim, simIK, object_name, gripper_handle, sim_tip, sim_target, ik_env, ik_group, auxData):
    object_handle = sim.getObject(object_name)
    object_pose = sim.getObjectPose(object_handle)

    print("Approaching drop off!")
    # Approach pose
    approach_pose = object_pose.copy()
    approach_pose[2] += 0.7     # on top of the object
    move_to_pose(sim, simIK, approach_pose, sim_target, auxData)
    
    
    ik_env2 = simIK.createEnvironment()
    orient_ik_group = simIK.createGroup(ik_env2)
    simIK.addElementFromScene(ik_env2, orient_ik_group, auxData['modelBase'], sim_tip, sim_target, simIK.constraint_pose)
    
    print("Closer position to drop object safely")
    # Drop pose
    drop_pose = object_pose.copy()
    drop_pose[2] += 0.3  # Adjust height slightly below the object
    move_to_pose(sim, simIK, drop_pose, sim_target, auxData)

    # Open gripper
    set_gripper_data(gripper_handle, True)
    
    # Return arm to og position
    og_pose = drop_pose.copy()
    og_pose[2] += 0.3  # Lift 30cm
    move_to_pose(sim, simIK, og_pose, sim_target, auxData)
    
    

def ur5_ik_control():
    robotColor = 'UR5'
    client = RemoteAPIClient()
    sim = client.require('sim')
    global simIK
    simIK = client.require('simIK')
    sims['UR5'] = sim
    sim.setStepping(True)

    gripper_handle = sim.getObject('./RG2')
    sim_tip = sim.getObject('./ikTip')
    sim_target = sim.getObject('./ikTarget')
    model_base = sim.getObject('/UR5')
    

    jointHandles = []
    for i in range(6):
        jointHandles.append(sim.getObject('/' + 'UR5' + '/joint', {'index': i}))

    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)
    simIK.addElementFromScene(ik_env, ik_group, model_base, sim_tip, sim_target, simIK.constraint_position)

    auxData = {
        'tip': sim_tip,
        'target': sim_target,
        'ikEnv': ik_env,
        'ikGroup': ik_group,
        'modelBase': model_base,
        'jointHandles': jointHandles,
    }

    if pick_flag:
        print("Picking sequence")
    else:
        print("Drop off sequence")
    
    robot_base = sim.getObject('/PioneerP3DX')
    
    if pick_flag:
        object_handle = sim.getObject('/Object')
        if check_distance(sim, robot_base, object_handle):
            print("Object is within reach. Executing robotic arm control.")
            # Open the gripper
            set_gripper_data(gripper_handle, True)

            # Get initial pose of the tip
            initial_tip_pose = sim.getObjectPose(sim_tip)
            
            # Set initial target pose to match the tip pose
            sim.setObjectPose(sim_target, initial_tip_pose)

            # Move to a starting position
            move_to_pose(sim, simIK, initial_tip_pose, sim_target, auxData)
            # Pick up the object
            pick_object(sim, simIK, '/Object', gripper_handle, sim_tip, sim_target, ik_env, ik_group, auxData)
            
            print("Picking completed!")
            
            pick_flag = False
        
    else:
        object_handle = sim.getObject('/DropLocation')
        if check_distance(sim, robot_base, object_handle):
            print("Drop off is within reach. Executing robotic arm control.")
            
            drop_object(sim, simIK, '/DropLocation', gripper_handle, sim_tip, sim_target, ik_env, ik_group, auxData)
        
            print("Drop off completed!")
        
            pick_flag = True
            
        else:
            print("Drop location is too far away. Skipping arm control.")
    
    
    # Move to a drop-off position (you can adjust this as needed)
    #drop_off_pose = initial_tip_pose.copy()
    #drop_off_pose[1] += 0.2  # Move from initial position
    #move_to_pose(sim, simIK, drop_off_pose, sim_target, auxData)
    
    # Open gripper to release object
    #set_gripper_data(gripper_handle, True)
    
    #print("Pick and place completed!")

    sim.setStepping(False)

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
                    print("Navigation task completed. Checking distance...")
                    
                    ur5_thread = threading.Thread(target=ur5_ik_control)
                    ur5_thread.start()
                    ur5_thread.join()  # Wait for UR5 control to complete
                    
                    sim.setIntegerSignal("nav_completed", 0)  # Reset the signal
                    print("Ready for next cycle.")
                    break  # Exit the inner loop and wait for the next navigation signal
                sim.step()  # Step the simulation
            
    except KeyboardInterrupt: 
        print("Simulation interrupted by user.")
    finally:
        sim.stopSimulation()

    print('Program ended')

if __name__ == '__main__':
    main()