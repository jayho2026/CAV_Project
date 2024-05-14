from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

def detect_obstacles(sim, sensors, noDetectionDist):
    obstacles = []
    for sensor in sensors:
        res, dist, detectedPoint, _, _ = sim.readProximitySensor(sensor)
        if res == 1 and dist < noDetectionDist:
            obstacles.append(detectedPoint[:2])  # Store the x, y position of the detected obstacle
    return obstacles

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')
    robot = sim.getObject('/PioneerP3DX')
    goal = sim.getObject('/Goal')
    
    # Sensing
    sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(16)]  # All 8 sensors
    
    noDetectionDist = 0.8  # Set the detection distance

    # Start the simulation
    sim.startSimulation()

    try:
        while True:
            # Sample the environment
            obstacles = detect_obstacles(sim, sensors, noDetectionDist)
            
            # Print obstacles for debugging
            print("Obstacles detected: ", obstacles)

            # Simulate robot behavior (placeholder for now)
            # e.g., move randomly or follow a simple pattern
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
