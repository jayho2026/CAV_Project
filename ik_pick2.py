import math
import time
import threading
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class UR5Controller:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.simIK = self.client.getObject('simIK')
        self.init_handles()
        self.init_ik_environment()

    def init_handles(self):
        self.gripper_handle = self.sim.getObject('./RG2')
        self.sim_tip = self.sim.getObject('./ikTip')
        self.sim_target = self.sim.getObject('./ikTarget')
        self.model_base = self.sim.getObject('/UR5')
        self.goal_object = self.sim.getObject('/Cuboid')
        self.joint_handles = [self.sim.getObject(f'/UR5/joint', {'index': i}) for i in range(6)]

    def init_ik_environment(self):
        self.ik_env = self.simIK.createEnvironment()
        self.ik_group = self.simIK.createGroup(self.ik_env)
        self.simIK.addElementFromScene(self.ik_env, self.ik_group, self.model_base, self.sim_tip, self.sim_target, self.simIK.constraint_pose)

    def set_gripper_data(self, open, velocity=0.15, force=60):
        data = {'velocity': velocity if open else -velocity, 'force': force}
        self.sim.writeCustomTableData(self.gripper_handle, 'activity', data)

    def solve_ik(self):
        result = self.simIK.handleGroup(self.ik_env, self.ik_group, {'syncWorlds': True})
        if result:
            joint_handles = self.simIK.getGroupJoints(self.ik_env, self.ik_group)
            joint_positions = [self.simIK.getJointPosition(self.ik_env, handle) for handle in joint_handles]
            return True, joint_positions
        return False, []

    def move_to_pose(self, target_pose):
        max_velocity = [0.4, 0.4, 0.4, 1.8]
        max_acceleration = [0.8, 0.8, 0.8, 0.9]
        max_jerk = [0.6, 0.6, 0.6, 0.8]
        
        self.sim.setObjectPose(self.sim_target, target_pose)
        
        while True:
            result = self.sim.moveToPose(-1, self.sim.getObjectPose(self.sim_tip), max_velocity, max_acceleration, max_jerk, target_pose, self.move_callback, {'target': self.sim_target, 'ik_env': self.ik_env, 'ik_group': self.ik_group}, None)
            if result[0] is not None:
                break
            self.sim.step()
            time.sleep(0.01)

    def move_callback(self, pose, velocity, accel, aux_data):
        self.sim.setObjectPose(aux_data['target'], pose)
        self.simIK.handleGroup(aux_data['ik_env'], aux_data['ik_group'], {'syncWorlds': True})

    def calculate_approach_pose(self, object_handle, distance=0.2):
        object_position = self.sim.getObjectPosition(object_handle, -1)
        object_orientation = self.sim.getObjectQuaternion(object_handle, -1)
        
        # Calculate approach position
        approach_vector = self.sim.getObjectMatrix(object_handle, -1)[0:3]  # Get the object's local X axis
        approach_position = [
            object_position[0] - approach_vector[0] * distance,
            object_position[1] - approach_vector[1] * distance,
            object_position[2] - approach_vector[2] * distance
        ]
        
        # Calculate gripper orientation
        gripper_orientation = self.calculate_gripper_orientation(object_handle)
        
        return approach_position + gripper_orientation

    def calculate_gripper_orientation(self, object_handle):
        # Get the object's orientation
        object_orientation = self.sim.getObjectQuaternion(object_handle, -1)
        
        # Rotate the gripper 90 degrees around its Y-axis to align with the object
        rotation_q = [0, 0.7071068, 0, 0.7071068]  # 90 degrees rotation around Y-axis
        
        # Combine rotations
        final_orientation = self.quaternion_multiply(object_orientation, rotation_q)
        
        return final_orientation

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return [w, x, y, z]

    def pick_object(self, object_name):
        object_handle = self.sim.getObject(object_name)
        object_pose = self.sim.getObjectPose(object_handle)
        
        # Approach pose
        approach_pose = self.calculate_approach_pose(object_handle)
        self.move_to_pose(approach_pose)
        print("Approached object")
        
        # Grab pose
        grab_pose = object_pose.copy()
        grab_pose[2] += 0.05  # Slightly above the object
        self.move_to_pose(grab_pose)
        print("Moved to grab pose")
        
        # Close gripper
        self.set_gripper_data(False)
        time.sleep(1)
        
        # Lift object
        lift_pose = grab_pose.copy()
        lift_pose[2] += 0.2
        self.move_to_pose(lift_pose)
        print("Lifted object")

    def run(self):
        try:
            # Open gripper
            self.set_gripper_data(True)
            
            # Move to initial position
            initial_pose = self.sim.getObjectPose(self.sim_tip)
            self.move_to_pose(initial_pose)
            
            # Pick up the object
            self.pick_object('/Cuboid')
            
            print("Pick and place completed!")
        
        except Exception as e:
            print(f"An error occurred: {str(e)}")

def main():
    print('Program started')
    
    try:
        controller = UR5Controller()
        
        controller.sim.setStepping(True)
        controller.sim.startSimulation()
        
        ur5_thread = threading.Thread(target=controller.run)
        ur5_thread.start()
        
        ur5_thread.join()
    
    except Exception as e:
        print(f"An error occurred in main: {str(e)}")
    
    finally:
        if 'controller' in locals():
            controller.sim.stopSimulation()
        print('Program ended')

if __name__ == '__main__':
    main()