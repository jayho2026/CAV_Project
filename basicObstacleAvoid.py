# Simple obstacle avoidance test with proximity or ultrasonic sensors
# Run the code and the car will turn left when it detects a wall

# First Created: 24/04/2024 by JJ

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')

    # Assuming sensors are labeled from 1 to 16, and sensors 4 to 6 cover the front
    #front_sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor{i}') for i in range(2, 5)]
    front_sensors = sim.getObject('/PioneerP3DX/ultrasonicSensor[2]')
    
    noDetectionDist = 0.6  # Set the detection threshold distance

    # Start the simulation
    sim.startSimulation()

    try:
        while True:
            obstacle_detected = False
            
            res, dist, _, _, _ = sim.readProximitySensor(front_sensors)
            if res == 1 and dist < noDetectionDist:
                obstacle_detected = True
                

            if obstacle_detected:
                # If an obstacle is detected, turn left
                sim.setJointTargetVelocity(motorLeft, -2)     # Slow down or stop the left motor
                sim.setJointTargetVelocity(motorRight, 2)   # Keep the right motor moving to turn left
                time.sleep(0.3)
            else:
                # If no obstacle is detected, move forward
                sim.setJointTargetVelocity(motorLeft, 2)
                sim.setJointTargetVelocity(motorRight, 2)

            time.sleep(0.1)  # Short delay to prevent high CPU usage

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
