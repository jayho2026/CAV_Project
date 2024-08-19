from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

def move_to_config_callback(config, velocity, accel, aux_data):
    sim = aux_data['sim']
    for i, joint in enumerate(aux_data['joints']):
        if sim.isDynamicallyEnabled(joint):
            sim.setJointTargetPosition(joint, config[i])
        else:
            sim.setJointPosition(joint, config[i])

def move_to_config_via_fk(sim, max_velocity, max_acceleration, max_jerk, goal_config, aux_data):
    start_config = [sim.getJointPosition(joint) for joint in aux_data['joints']]
    sim.moveToConfig(-1, start_config, None, None, max_velocity, max_acceleration, max_jerk, goal_config, None, move_to_config_callback, aux_data, None)

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    sim.startSimulation()

    # Initialize handles for the 6 joints of the UR5
    sim_joints = [sim.getObject('./joint', {'index': i}) for i in range(6)]

    # FK movement data
    vel = 180  # degrees per second
    accel = 40  # degrees per second^2
    jerk = 80   # degrees per second^3
    max_vel = [vel * math.pi / 180] * 6
    max_accel = [accel * math.pi / 180] * 6
    max_jerk = [jerk * math.pi / 180] * 6

    # Hardcoded joint configuration (in radians)
    # This represents [0, -90, 90, -90, -90, 0] in degrees
    goal_config = [0, -90 * math.pi / 180, 0, 0, 0, 0]

    aux_data = {
        'sim': sim,
        'joints': sim_joints
    }

    # Move to the goal configuration
    move_to_config_via_fk(sim, max_vel, max_accel, max_jerk, goal_config, aux_data)

    # Wait for the movement to complete
    sim.wait(5)

    sim.stopSimulation()

if __name__ == "__main__":
    main()