# Simple path planning for the PioneerP3DX car in the CoppeliaSim Simulation
# Put a dummy called "Goal" in Coppeliasim which represents the end goal
# Run the code and the car will calculate a path and move towards the path automatically

# First Created: 24/04/2024 by JJ


from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

def main():
    client = RemoteAPIClient()  # Create a client to connect to zmqRemoteApi server
    sim = client.require('sim')  # Get the simulation object

    # Ensure to use the correct object names as per your scene in CoppeliaSim
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')  # Get the left motor handle
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')  # Get the right motor handle
    robot = sim.getObject('/PioneerP3DX')  # Get the start dummy which is attached to the pioneer
    goal = sim.getObject('/Goal')  # Assume there is a dummy object named 'Goal'

    # Start the simulation
    sim.startSimulation()

    while True:
        robot_position = np.array(sim.getObjectPosition(robot, -1)[:2])
        goal_position = np.array(sim.getObjectPosition(goal, -1)[:2])
        robot_orientation = sim.getObjectOrientation(robot, -1)[2]  # Get yaw (z-axis rotation)

        direction = goal_position - robot_position
        distance = np.linalg.norm(direction)

        if distance < 0.3:  # 0.3m from the goal
            break

        goal_angle = np.arctan2(direction[1], direction[0])
        angle_diff = goal_angle - robot_orientation

        # Normalize the angle difference within the range -pi to pi
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        base_speed = 3.0
        turn_speed = 0.5 * angle_diff  # Control how fast the robot turns based on the angle difference

        left_speed = base_speed - turn_speed
        right_speed = base_speed + turn_speed

        sim.setJointTargetVelocity(motorLeft, left_speed)
        sim.setJointTargetVelocity(motorRight, right_speed)

        time.sleep(0.1)  # Short delay to prevent high CPU usage

    sim.setJointTargetVelocity(motorLeft, 0)
    sim.setJointTargetVelocity(motorRight, 0)
    sim.stopSimulation()

if __name__ == '__main__':
    main()