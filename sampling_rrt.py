import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class RRT:
    def __init__(self, start, goal, obstacles, x_range, y_range, step_len, goal_sample_rate, max_iter, sim):
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacles = obstacles
        self.x_range = x_range
        self.y_range = y_range
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.sim = sim
        self.dummy_handles = []

        # Create a dummy object for the start and goal nodes
        self.start_handle = self.create_dummy(self.start.x, self.start.y)
        self.goal_handle = self.create_dummy(self.goal.x, self.goal.y)

    def create_dummy(self, x, y):
        handle = self.sim.createDummy(0.05, [0, 0, 1])
        self.set_dummy_position(handle, x, y)
        return handle

    def set_dummy_position(self, dummy_handle, x, y):
        self.sim.setObjectPosition(dummy_handle, -1, [x, y, 0])

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0], self.x_range[1]),
                         np.random.uniform(self.y_range[0], self.y_range[1])))
        return self.goal

    def nearest_node(self, node):
        return self.nodes[int(np.argmin([np.hypot(nd.x - node.x, nd.y - node.y) for nd in self.nodes]))]

    def is_collision(self, node):
        for ox, oy, w, h in self.obstacles:
            if ox <= node.x <= ox + w and oy <= node.y <= oy + h:
                return True
        return False

    def new_state(self, from_node, to_node):
        dist, theta = self.get_distance_and_angle(from_node, to_node)
        dist = min(self.step_len, dist)
        new_node = Node((from_node.x + dist * np.cos(theta), from_node.y + dist * np.sin(theta)))
        new_node.parent = from_node
        return new_node

    def get_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return np.hypot(dx, dy), np.arctan2(dy, dx)

    def planning(self):
        for i in range(self.max_iter):
            if len(self.nodes) >= 50:  # Limit the number of nodes to 20
                break
            node_rand = self.generate_random_node()
            node_near = self.nearest_node(node_rand)
            node_new = self.new_state(node_near, node_rand)

            if not self.is_collision(node_new):
                self.nodes.append(node_new)
                dummy_handle = self.create_dummy(node_new.x, node_new.y)
                self.set_dummy_position(dummy_handle, node_new.x, node_new.y)
                self.dummy_handles.append(dummy_handle)

                if self.get_distance_and_angle(node_new, self.goal)[0] <= self.step_len:
                    final_node = self.new_state(node_new, self.goal)
                    if not self.is_collision(final_node):
                        self.nodes.append(final_node)
                        dummy_handle = self.create_dummy(final_node.x, final_node.y)
                        self.set_dummy_position(dummy_handle, final_node.x, final_node.y)
                        self.dummy_handles.append(dummy_handle)
                        return self.extract_path(final_node)
                    
            time.sleep(0.2)

        return None

    def extract_path(self, node):
        path = [(self.goal.x, self.goal.y)]
        while node.parent is not None:
            node = node.parent
            path.append((node.x, node.y))
        path.append((self.start.x, self.start.y))
        return path[::-1]

def get_obstacles_positions(sim):
    obstacles = []
    for i in range(2):  # Assuming there are 2 obstacle objects
        obstacle_handle = sim.getObject('/Cuboid[0]')
        obstacle_position = sim.getObjectPosition(obstacle_handle, sim.handle_world)[:2]
        obstacle_size = [1, 1]  # Assuming each obstacle occupies a 1x1 area
        obstacles.append((*obstacle_position, *obstacle_size))
    return obstacles

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    # Get positions of start, goal, and obstacles
    robot_handle = sim.getObject('/PioneerP3DX')
    goal_handle = sim.getObject('/Goal')
    
    start_position = sim.getObjectPosition(robot_handle, sim.handle_world)[:2]
    goal_position = sim.getObjectPosition(goal_handle, sim.handle_world)[:2]
    
    obstacles = get_obstacles_positions(sim)
    
    x_range = (0, 10)
    y_range = (0, 10)
    step_len = 0.5
    goal_sample_rate = 0.1
    max_iter = 500
    
    sim.startSimulation()
    
    rrt = RRT(start_position, goal_position, obstacles, x_range, y_range, step_len, goal_sample_rate, max_iter, sim)
    path = rrt.planning()

    if path:
        print("Path found!")
        for x, y in path:
            print(f"Path: x={x}, y={y}")
    else:
        print("Path not found.")
    
    if path:
        # Move the robot along the path
        motorLeft = sim.getObject('/PioneerP3DX/leftMotor')
        motorRight = sim.getObject('/PioneerP3DX/rightMotor')
        robot = sim.getObject('/PioneerP3DX')

        try:
            for x, y in path:
                current_position = np.array(sim.getObjectPosition(robot, -1)[:2])
                target_position = np.array([x, y])
                while np.linalg.norm(target_position - current_position) > 0.1:
                    current_position = np.array(sim.getObjectPosition(robot, -1)[:2])
                    direction = target_position - current_position
                    distance = np.linalg.norm(direction)
                    
                    robot_orientation = sim.getObjectOrientation(robot, -1)[2]
                    goal_angle = np.arctan2(direction[1], direction[0])
                    angle_diff = goal_angle - robot_orientation
                    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
                    
                    base_speed = 2.0
                    turn_speed = 2.5 * angle_diff
                    
                    left_speed = base_speed - turn_speed
                    right_speed = base_speed + turn_speed
                    
                    sim.setJointTargetVelocity(motorLeft, left_speed)
                    sim.setJointTargetVelocity(motorRight, right_speed)
                    
                    time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("Simulation interrupted by user.")
        
        finally:
            sim.setJointTargetVelocity(motorLeft, 0)
            sim.setJointTargetVelocity(motorRight, 0)
            sim.stopSimulation()
            print("Simulation stopped and cleaned up.")
    else:
        print("No valid path found. Adjust the environment or parameters and try again.")
        sim.stopSimulation()

if __name__ == '__main__':
    main()
