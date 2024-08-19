# Get started controlling the PioneerP3DX car in the CoppeliaSim Simulation
# Drag the car into the coppelia simulation
# Delete the Lua script in the simulation
# Run the code and the car will move according to the program below automatically

# First Created: 22/04/2024 by JJ

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

def main():
    client = RemoteAPIClient()  # Create a client to connect to zmqRemoteApi server
    sim = client.require('sim')  # Get the simulation object

    # Ensure to use the correct object names as per your scene in CoppeliaSim
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')  # Get the left motor handle
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')  # Get the right motor handle

    speed = 2.0  # Speed at which the motors should rotate

    # Start the simulation
    sim.startSimulation()

    # Set the motor velocities
    sim.setJointTargetVelocity(motorLeft, 10.5)
    sim.setJointTargetVelocity(motorRight, -10.5)

    # Let the robot move forward for 5 seconds
    time.sleep(1)

    # Stop the motors
    sim.setJointTargetVelocity(motorLeft, 0)
    sim.setJointTargetVelocity(motorRight, 0)

    # Stop the simulation
    sim.stopSimulation()

if __name__ == '__main__':
    main()
