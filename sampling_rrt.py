import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import json


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
        self.line_handle = self.create_line_drawing_handle()

        # Create a dummy object for the start and goal nodes
        self.start_handle = self.create_dummy(self.start.x, self.start.y)
        self.goal_handle = self.create_dummy(self.goal.x, self.goal.y)

    def create_dummy(self, x, y):
        handle = self.sim.createDummy(0.05, [0, 0, 1])
        self.set_dummy_position(handle, x, y)
        return handle

    def set_dummy_position(self, dummy_handle, x, y):
        self.sim.setObjectPosition(dummy_handle, -1, [x, y, 0])

    # Goal biasness
    # probability of sampling either a random space near the latest node or just follow the goal
    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            if np.random.random() < 0.5:  # 50% chance to generate a completely random node
                return Node((np.random.uniform(self.x_range[0], self.x_range[1]),
                            np.random.uniform(self.y_range[0], self.y_range[1])))
            else:  # 50% chance to expand from a random existing node
                base_node = np.random.choice(self.nodes)
                perturbation = np.random.normal(0, 1, 2)  # small Gaussian noise
                new_x = max(self.x_range[0], min(self.x_range[1], base_node.x + perturbation[0]))
                new_y = max(self.y_range[0], min(self.y_range[1], base_node.y + perturbation[1]))
                return Node((new_x, new_y))
        return self.goal

    def nearest_node(self, node):
        return self.nodes[int(np.argmin([np.hypot(nd.x - node.x, nd.y - node.y) for nd in self.nodes]))]
    
    def new_state(self, from_node, to_node):
        dist, theta = self.get_distance_and_angle(from_node, to_node)
        dist = min(self.step_len, dist)
        new_node = Node((from_node.x + dist * np.cos(theta), from_node.y + dist * np.sin(theta)))
        new_node.parent = from_node
        return new_node

    def is_collision(self, node):
        for ox, oy, radius in self.obstacles:
            if np.hypot(node.x - ox, node.y - oy) <= radius:
                return True
        return False

    def is_collision_on_path(self, from_node, to_node):
        def point_to_line_distance(px, py, x1, y1, x2, y2):
            norm = np.hypot(x2 - x1, y2 - y1)
            if norm == 0:
                return np.hypot(px - x1, py - y1)
            u = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (norm * norm)
            u = max(0, min(1, u))
            nearest_x = x1 + u * (x2 - x1)
            nearest_y = y1 + u * (y2 - y1)
            return np.hypot(px - nearest_x, py - nearest_y)

        for ox, oy, radius in self.obstacles:
            if point_to_line_distance(ox, oy, from_node.x, from_node.y, to_node.x, to_node.y) <= radius:
                return True
        return False

    def line_intersects_rect(self, x1, y1, x2, y2, rx, ry, rw, rh):
        # Check line intersection with rectangle
        # Calculate all four sides of the rectangle
        left = rx
        right = rx + rw
        top = ry
        bottom = ry + rh

        # Use the Cohen-Sutherland algorithm or Liang-Barsky algorithm for line clipping
        # For simplicity, here is a basic approach using separation axis theorem
        if (min(x1, x2) > right or max(x1, x2) < left or
            min(y1, y2) > bottom or max(y1, y2) < top):
            return False

        # Check line intersection with each side of the rectangle
        if self.segment_intersects(x1, y1, x2, y2, left, top, right, top) or \
        self.segment_intersects(x1, y1, x2, y2, left, bottom, right, bottom) or \
        self.segment_intersects(x1, y1, x2, y2, left, top, left, bottom) or \
        self.segment_intersects(x1, y1, x2, y2, right, top, right, bottom):
            return True

        return False

    def segment_intersects(self, Ax, Ay, Bx, By, Cx, Cy, Dx, Dy):
        # Determine if two line segments intersect using vector cross products
        def ccw(Ax, Ay, Bx, By, Cx, Cy):
            return (Cy - Ay) * (Bx - Ax) > (By - Ay) * (Cx - Ax)

        return (ccw(Ax, Ay, Cx, Cy, Dx, Dy) != ccw(Bx, By, Cx, Cy, Dx, Dy) and
                ccw(Ax, Ay, Bx, By, Cx, Cy) != ccw(Ax, Ay, Bx, By, Dx, Dy))



    
    def get_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return np.hypot(dx, dy), np.arctan2(dy, dx)

    def planning(self):
        for i in range(self.max_iter):
            if len(self.nodes) >= 1500:  # Limit the number of nodes to 20
                break
            node_rand = self.generate_random_node()
            node_near = self.nearest_node(node_rand)
            node_new = self.new_state(node_near, node_rand)

            if not self.is_collision(node_new):
                self.nodes.append(node_new)
                dummy_handle = self.create_dummy(node_new.x, node_new.y)
                self.set_dummy_position(dummy_handle, node_new.x, node_new.y)
                self.dummy_handles.append(dummy_handle)
                self.add_line(node_near, node_new)  # Draw line from nearest node to new node


                if self.get_distance_and_angle(node_new, self.goal)[0] <= self.step_len:
                    final_node = self.new_state(node_new, self.goal)
                    if not self.is_collision(final_node):
                        self.nodes.append(final_node)
                        dummy_handle = self.create_dummy(final_node.x, final_node.y)
                        self.set_dummy_position(dummy_handle, final_node.x, final_node.y)
                        self.dummy_handles.append(dummy_handle)
                        return self.extract_path(final_node)
                    
            else: continue
                    
            time.sleep(0.2)

        return None

    def extract_path(self, node):
        path = [(self.goal.x, self.goal.y)]
        while node.parent is not None:
            node = node.parent
            path.append((node.x, node.y))
        path.append((self.start.x, self.start.y))
        return path[::-1]
    
    
    def create_line_drawing_handle(self):
        # Create a line drawing handle with red color and a thickness of 2 pixels
        return self.sim.addDrawingObject(self.sim.drawing_lines, 2, 0, -1, 9999, [1, 0, 0], [0, 0, 0], [0, 0, 0], [1, 0, 0])
    
    def clear_lines(self):
        # Clear all lines
        self.sim.addDrawingObjectItem(self.line_handle, None)
        
    

    def add_line(self, node1, node2):
        # Add a line segment between node1 and node2
        line_data = [node1.x, node1.y, 0, node2.x, node2.y, 0]
        self.sim.addDrawingObjectItem(self.line_handle, line_data)


def load_obstacles_from_json(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    obstacles = []
    for item in data:
        x, y, diameter = item['x'], item['y'], item['size']
        radius = diameter / 2
        obstacles.append((x, y, radius))  # Store center and radius
    return obstacles


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    # Get positions of start, goal, and obstacles
    robot_handle = sim.getObject('/PioneerP3DX')
    goal_handle = sim.getObject('/Goal')
    
    start_position = sim.getObjectPosition(robot_handle, sim.handle_world)[:2]
    goal_position = sim.getObjectPosition(goal_handle, sim.handle_world)[:2]
    
        
    # Load obstacles from the JSON file
    obstacles = load_obstacles_from_json('grid.json')
   
    
    
    
    x_range = (-10, 20)
    y_range = (-10, 20)
    step_len = 0.25
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
            rrt.clear_lines()
            print("Simulation stopped and cleaned up.")
    else:
        print("No valid path found. Adjust the environment or parameters and try again.")
        rrt.clear_lines()
        sim.stopSimulation()

if __name__ == '__main__':
    main()
