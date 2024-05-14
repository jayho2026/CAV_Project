from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np
import matplotlib.pyplot as plt

def detect_obstacles(sim, robot, sensors, noDetectionDist):
    obstacles = []
    robot_position = sim.getObjectPosition(robot, -1)[:2]
    robot_orientation = sim.getObjectOrientation(robot, -1)[2]  # Get yaw (z-axis rotation)
    
    for sensor in sensors:
        res, dist, detectedPoint, _, _ = sim.readProximitySensor(sensor)
        if res == 1 and dist < noDetectionDist:
            detectedPoint = np.array(detectedPoint[:2])
            
            # Transform detected point based on robot position and orientation
            rotation_matrix = np.array([[np.cos(robot_orientation), -np.sin(robot_orientation)],
                                        [np.sin(robot_orientation), np.cos(robot_orientation)]])
            detected_point_transformed = rotation_matrix @ detectedPoint + robot_position
            obstacles.append(detected_point_transformed)
    
    return obstacles

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')
    robot = sim.getObject('/PioneerP3DX')
    goal = sim.getObject('/Goal')
    
    # Sensing
    sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(16)]  # All 16 sensors
    
    noDetectionDist = 2.5  # Set the detection distance

    # Start the simulation
    sim.startSimulation()

    # Initialize a list to store accumulated obstacle points
    accumulated_obstacles = []

    try:
        plt.ion()
        fig, ax = plt.subplots()
        scatter = ax.scatter([], [])
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        
        while True:
            # Sample the environment
            obstacles = detect_obstacles(sim, robot, sensors, noDetectionDist)
            accumulated_obstacles.extend(obstacles)
            
            # Print obstacles for debugging
            print("Obstacles detected: ", obstacles)

            # Update the scatter plot
            scatter.set_offsets(accumulated_obstacles)
            plt.draw()
            plt.pause(0.1)

            # Simulate robot behavior (placeholder for now)
            #sim.setJointTargetVelocity(motorLeft, 1.0)
            #sim.setJointTargetVelocity(motorRight, 1.0)
            
            time.sleep(0.5)  # Sample every 0.5 seconds

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")

    finally:
        # Cleanup
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        sim.stopSimulation()
        print("Simulation stopped and cleaned up.")

if __name__ == '__main__':
    main()
