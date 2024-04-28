


from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    motorLeft = sim.getObject('/PioneerP3DX/leftMotor')
    motorRight = sim.getObject('/PioneerP3DX/rightMotor')
    robot = sim.getObject('/PioneerP3DX')

    # These are the group of waypoints 
    waypoints = [
        sim.getObject('/Waypoint1'),
        sim.getObject('/Waypoint2'),
        sim.getObject('/Waypoint3'),
        sim.getObject('/Goal')
    ]

    # Start the simulation
    sim.startSimulation()
    


    # Process each waypoint
    for goal in waypoints:
        navigate_to_goal(sim, robot, goal, motorLeft, motorRight)

    # Cleanup
    sim.setJointTargetVelocity(motorLeft, 0)
    sim.setJointTargetVelocity(motorRight, 0)
    sim.stopSimulation()

def navigate_to_goal(sim, robot, goal, motorLeft, motorRight):
    previous_position = np.array(sim.getObjectPosition(robot, -1)[:2])

    while True:

        current_position = np.array(sim.getObjectPosition(robot, -1)[:2])
        goal_position = np.array(sim.getObjectPosition(goal, -1)[:2])
        robot_orientation = sim.getObjectOrientation(robot, -1)[2]  # Get yaw (z-axis rotation)

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

if __name__ == '__main__':
    main()
