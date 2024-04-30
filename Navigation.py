# Path Planning with obstacles for the PioneerP3DX car in the CoppeliaSim Simulation
# Put a dummy called "Goal" in Coppeliasim which represents the end goal
# Run the code and the car will calculate a path and move towards the path automatically

# First Created: 24/04/2024 by JJ

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

# Detect obstacles function
def detect_obstacles(sim, robot):
    
    return sim.readProximitySensor(robot)


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')
    robot = sim.getObject('/PioneerP3DX')
    goal = sim.getObject('/Goal')
           
    # Sensing
    left_sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(0, 3)]
    right_sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(4, 7)]
    
    noDetectionDist = 0.25  # Set the detection distance

    # Start the simulation
    sim.startSimulation()

    # Initialize drawing (the line handle)
    navigated_path_line = sim.addDrawingObject(sim.drawing_lines, 2, 0.0, -1, 10000, [1,0,0])
    previous_position = np.array(sim.getObjectPosition(robot, -1)[:2])

    try:
        while True:
            
            left_detected = False
            right_detected = False
            
            # Check each sensor in front_sensors
            for sensor in left_sensors:
                res, dist, _, _, _ = sim.readProximitySensor(sensor)
                if res == 1 and dist < noDetectionDist:
                    left_detected = True
                    break
            
            # Check each sensor in front_sensors
            for sensor in right_sensors:
                res, dist, _, _, _ = sim.readProximitySensor(sensor)
                if res == 1 and dist < noDetectionDist:
                    right_detected = True
                    break    

            if left_detected:
                # If an obstacle is detected, turn right
                sim.setJointTargetVelocity(motorLeft, 1)     # Slow down or stop the left motor
                sim.setJointTargetVelocity(motorRight, -0.5)   # Keep the right motor moving to turn left
                time.sleep(0.1)
                
            elif right_detected:
                # If an obstacle is detected, turn left
                sim.setJointTargetVelocity(motorLeft, -0.5)     # Slow down or stop the left motor
                sim.setJointTargetVelocity(motorRight, 1)   # Keep the right motor moving to turn left
                time.sleep(0.1)
                
            else:
                # If no obstacle is detected, move towards goal
                           
                current_position = np.array(sim.getObjectPosition(robot, -1)[:2])
                goal_position = np.array(sim.getObjectPosition(goal, -1)[:2])
                robot_orientation = sim.getObjectOrientation(robot, -1)[2]  # Get yaw (z-axis rotation)
                
                # Draw the path
                sim.addDrawingObjectItem(navigated_path_line, previous_position.tolist() + [0.05] + current_position.tolist() + [0.05])
                previous_position = current_position

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
        sim.removeDrawingObject(navigated_path_line)
        sim.stopSimulation()
        print("Simulation stopped and cleaned up.")

if __name__ == '__main__':
    main()
