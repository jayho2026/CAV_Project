# Added navigation visualisation for the navigated and predicted path from the simplePathPlanning.py
# Put a dummy called "Goal" in Coppeliasim which represents the end goal
# Run the code and the car will calculate a path and move towards the path automatically

# First Created: 24/04/2024 by JJ


from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')
    robot = sim.getObject('/PioneerP3DX')
    goal = sim.getObject('/Goal')

    # Start the simulation
    sim.startSimulation()

    # Initialize drawing (the line handles)
    navigated_path_container = sim.addDrawingObject(sim.drawing_lines, 2, 0.0, -1, 10000, [0,0,1])  # Blue line for navigated path
    predicted_path_container = sim.addDrawingObject(sim.drawing_lines, 2, 0.0, -1, 10000, [1,0,0])  # Red line for predicted path

    previous_position = np.array(sim.getObjectPosition(robot, -1)[:2])

    try:
        while True:
            current_position = np.array(sim.getObjectPosition(robot, -1)[:2])
            goal_position = np.array(sim.getObjectPosition(goal, -1)[:2])
            robot_orientation = sim.getObjectOrientation(robot, -1)[2]  # Get yaw (z-axis rotation)

            # Draw the navigated path
            sim.addDrawingObjectItem(navigated_path_container, previous_position.tolist() + [0.05] + current_position.tolist() + [0.05])
            previous_position = current_position

            # Predict path forward and display it
            predict_path(sim, predicted_path_container, current_position, goal_position, robot_orientation)

            # Determine the direction and move the robot
            direction = goal_position - current_position
            distance = np.linalg.norm(direction)
            
            if distance < 0.3:  # 0.3m from the goal
                break

            goal_angle = np.arctan2(direction[1], direction[0])
            angle_diff = goal_angle - robot_orientation
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize the angle

            base_speed = 3.0
            turn_speed = 0.5 * angle_diff

            left_speed = base_speed - turn_speed
            right_speed = base_speed + turn_speed

            sim.setJointTargetVelocity(motorLeft, left_speed)
            sim.setJointTargetVelocity(motorRight, right_speed)

            time.sleep(0.1)  # Short delay to prevent high CPU usage

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")

    finally:
        # Cleanup
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        sim.removeDrawingObject(navigated_path_container)
        sim.removeDrawingObject(predicted_path_container)
        sim.stopSimulation()
        print("Simulation stopped and cleaned up.")

def predict_path(sim, path_container, start_pos, goal_pos, start_orientation):
    sim.removeDrawingObject(path_container)  # Clear previous projection
    path_container = sim.addDrawingObject(sim.drawing_lines, 2, 0.0, -1, 10000, [1,0,0])

    steps = 50
    step_size = 0.1
    position = start_pos
    orientation = start_orientation

    for _ in range(steps):
        direction = goal_pos - position
        goal_angle = np.arctan2(direction[1], direction[0])
        angle_diff = goal_angle - orientation
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize the angle

        orientation += 0.5 * angle_diff  # Simulate turning
        dx = np.cos(orientation) * step_size
        dy = np.sin(orientation) * step_size
        new_position = position + np.array([dx, dy])

        # Draw the projected segment
        sim.addDrawingObjectItem(path_container, position.tolist() + [0.05] + new_position.tolist() + [0.05])
        position = new_position

if __name__ == '__main__':
    main()
