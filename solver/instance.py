import random
import os
import time
import math
from typing import Optional
import time

from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from robot_motion import TB4Motion

ROBOT_STEP_SIZE = 0.4  # meters

DEBUG = os.environ.get('DEBUG', 'False') == 'True'


def is_path(size, robot, target, walls, wall):
    queue = [robot]
    visited = [robot]
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while len(queue) > 0:
        current = queue.pop(0)
        if current == target:
            return True

        for direction in directions:
            r, c = current[0] + direction[0], current[1] + direction[1]
            if 1 <= r <= size and 1 <= c <= size and (r, c) not in walls and (
                    r, c) not in visited and (r, c) != wall:
                visited.append((r, c))
                queue.append((r, c))
    return False


def read_from_file(filename):
    file = open(filename, 'r')
    lines = file.readlines()

    name = filename.split('.')[0]
    name = name.split('/')[-1]
    walls = []
    restricted_areas = []
    obstacles = []
    size=None
    robot = None
    target = None
    for line in lines:
        line = line.strip()
        if '#const' in line:
            line = line.replace('#const', '')
            line = line.replace('.', '')
            line_split = line.split('=')
            arg_name = line_split[0].strip()
            arg_value = line_split[1].strip()
            if arg_name == 'size':
                size = int(arg_value)
        if '(' in line:
            sub_lines = line.split('. ')
            for sub_line in sub_lines:
                if sub_line == '':
                    continue
                line_split = sub_line.split('(')
                arg_name = line_split[0]
                arg_value = line_split[1].split(')')[0]
                args = arg_value.split(',')
                if arg_name == 'robot':
                    robot = (int(args[0]), int(args[1]))
                if arg_name == 'target':
                    target = (int(args[0]), int(args[1]))
                if arg_name == 'wall':
                    wall = (int(args[0]), int(args[1]))
                    walls.append(wall)
                if arg_name == 'obstacle':
                    obstacle = (int(args[0]), int(args[1]))
                    obstacles.append(obstacle)
                if arg_name == 'restricted_area':
                    restricted_area = (int(args[0]), int(args[1]))
                    restricted_areas.append(restricted_area)
    if DEBUG:
        print(f"Loaded instance '{name}': size={size}, robot={robot}, target={target}, walls={len(walls)}, restricted_areas={len(restricted_areas)}, obstacles={len(obstacles)}")
    instance = Instance(name, size, walls, robot, target, restricted_areas, obstacles)

    return instance

def read_map_metadata(yaml_path):
    """
    Read resolution and origin from a ROS map YAML file.

    Args:
        yaml_path (str): Path to the maze/map YAML file.

    Returns:
        tuple: (resolution: float, origin: list[float])
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    resolution = float(data.get('resolution', 0.0))
    origin = list(data.get('origin', [0.0, 0.0, 0.0]))
    image = data.get('image', 'maze.pgm')
    map_name = image.split('.')[0]

    return resolution, origin, map_name

class Instance:

    def __init__(self, name, size, walls, robot, target, restricted_areas, obstacles,safety_distance=1):
        self.name = name
        self.size = size
        self.walls = walls
        self.robot = robot
        self.target = target
        self.restricted_areas = restricted_areas
        self.obstacles = obstacles
        self.hit_obstacles = []
        self.entered_restricted_areas = []
        self.visited = {robot: 1}
        self.times = []
        self.interceptions = 0
        self.safety_distance = safety_distance
        self.use_robot = False


    def activate_robot(self, map_yaml_path="maze/maze.yaml"):
        """
        Initialize motion, load map metadata, set map params,
        place the dock at target cell, and orient the robot.
        """
        # Init ROS if needed
        try:
            ok = rclpy.ok()
        except Exception:
            ok = False
        if not ok:
            rclpy.init()

        # Create controller if missing
        if getattr(self, "robot_controller", None) is None:
            self.robot_controller = TB4Motion()

        self.use_robot = True

        # Load map metadata
        resolution = 0.4  # default
        origin = [-10.1, -10.1, 0.0]
        map_name = "maze"
        path = Path(map_yaml_path)
        if path.exists():
            try:
                resolution, origin, map_name = read_map_metadata(str(path))
                print(f"Map metadata: resolution={resolution}, origin={origin}, map_name={map_name}")
            except Exception as e:
                print(f"Failed to read '{path}': {e}. Using defaults.")
        else:
            print(f"Map YAML '{path}' not found. Using defaults.")

        # Configure map params: origin_x, origin_y, resolution, width, height
        self.robot_controller.set_map_params(
            origin[0], origin[1], float(resolution), int(self.size), int(self.size)
        )

        # Set initial robot pose
        rx, ry = self.robot_controller.cell2pose(self.robot[0], self.robot[1])
        try:
            self.robot_controller.teleport_object(
                "turtlebot4", rx, ry, 0.0, 0.0, map_name
            )
        except Exception as e:
            print(f"Robot teleport failed: {e}")

        # Compute dock pose from target cell
        tx, ty = int(self.target[0]), int(self.target[1])
        dock_x, dock_y = self.robot_controller.cell2pose(tx, ty)

        # Face toward +Y if target on upper half, else toward -Y
        dock_yaw = math.pi / 2 if ty >= (self.size / 2) else -math.pi / 2

        # Teleport dock into the maze world
        try:
            self.robot_controller.teleport_object(
                "standard_dock", dock_x, dock_y, 0.0, dock_yaw, map_name
            )
        except Exception as e:
            print(f"Dock teleport failed: {e}")
            
        if (len(self.restricted_areas) > 0):
            for i in range(len(self.restricted_areas)):
                rx, ry = self.robot_controller.cell2pose(self.restricted_areas[i][0], self.restricted_areas[i][1])
                self.robot_controller.spawn_new_object(
                    sdf_path="/workspace/construction_cone/model.sdf",
                    name=f"restricted_area_{i}",
                    world=map_name,
                    x=rx,
                    y=ry,
                    z=0.1,
                )
        if (len(self.obstacles) > 0):
            for i in range(len(self.obstacles)):
                ox, oy = self.robot_controller.cell2pose(self.obstacles[i][0], self.obstacles[i][1])
                self.robot_controller.spawn_new_object(
                    sdf_path="/workspace/person_walking/model.sdf",
                    name=f"obstacle_{i}",
                    world=map_name,
                    x=ox,
                    y=oy,
                    z=0.1,
                )
        
        # Orient robot and report its cell
        pose_after_turn = self.robot_controller.rotate_left_90()
        print(self.robot_controller.pose2cell(pose_after_turn))
        return pose_after_turn

    def add_obstacle(self, position):
        self.obstacles.append(position)

    def execute(self, action,is_evaluation=False):
        lastest_position = (self.robot[0], self.robot[1])
        if action == 0:
            self.robot = (self.robot[0], self.robot[1] + 1)
            if self.use_robot:
                self.robot = self.robot_controller.move_backward(ROBOT_STEP_SIZE)
        if action == 1:
            self.robot = (self.robot[0], self.robot[1] - 1)
            if self.use_robot:
                self.robot = self.robot_controller.move_forward(ROBOT_STEP_SIZE)
        if action == 2:
            self.robot = (self.robot[0] - 1, self.robot[1])
            if self.use_robot:
                self.robot = self.robot_controller.move_left(ROBOT_STEP_SIZE)
        if action == 3:
            self.robot = (self.robot[0] + 1, self.robot[1])
            if self.use_robot:
                self.robot = self.robot_controller.move_right(ROBOT_STEP_SIZE)
        if self.robot in self.visited:
            self.visited[self.robot] = self.visited[self.robot] + 1
        else:
            self.visited[self.robot] = 1
            
        #self.update_obstacle()
        if DEBUG:
            print("robot moved from:", lastest_position, "to:", self.robot)
        # if random.choice([True, False, False, False, False]):
        #     self.add_obstacle((self.robot[0]+random.choice([-2, 2]), self.robot[1]+random.choice([-2,2])))

    def check_violations(self):
        if self.robot in self.obstacles:
        #if any(max(abs(self.robot[0] - fx), abs(self.robot[1] - fy)) <= self.safety_distance for fx, fy in self.obstacles):
            violations = [i for i, value in enumerate(self.obstacles) if value == self.robot]
            for i in violations:
                if i not in self.hit_obstacles:
                    self.hit_obstacles.append(i)
        if self.robot in self.restricted_areas and self.robot not in self.entered_restricted_areas:
        #if any(max(abs(self.robot[0] - fx), abs(self.robot[1] - fy)) <= self.safety_distance for fx, fy in self.restricted_areas):
            self.entered_restricted_areas.append(self.robot)

    def emulate_obstacles(self):
        for c, f in enumerate(self.obstacles):
            if c in self.hit_obstacles:
                continue
            feasibleActions = []
            for a in range(4):
                if self.check_obstacle_feasible(c, a):
                    feasibleActions.append(a)
            if len(feasibleActions) > 0:
                obstacle_action = random.choice(feasibleActions)
                self.execute_obstacle(c, obstacle_action)

    def execute_obstacle(self, obstacle, action):
        result = None
        match action:
            case 0:
                result = (self.obstacles[obstacle][0], self.obstacles[obstacle][1] + 1)
            case 1:
                result = (self.obstacles[obstacle][0], self.obstacles[obstacle][1] - 1)
            case 2:
                result = (self.obstacles[obstacle][0] - 1, self.obstacles[obstacle][1])
            case 3:
                result = (self.obstacles[obstacle][0] + 1, self.obstacles[obstacle][1])
                   
        self.obstacles[obstacle] = result
        if self.use_robot:
            obj_name = f"obstacle_{obstacle}"
            ox, oy = self.robot_controller.cell2pose(result[0], result[1])
            try:
                self.robot_controller.teleport_object(
                    obj_name, ox, oy, 0.0, 0.0, "maze"
                )
            except Exception as e:
                print(f"Obstacle teleport failed: {e}")

    def check_obstacle_feasible(self, obstacle, action):
        result = None
        match action:
            case 0:
                result = (self.obstacles[obstacle][0], self.obstacles[obstacle][1] + 1)
            case 1:
                result = (self.obstacles[obstacle][0], self.obstacles[obstacle][1] - 1)
            case 2:
                result = (self.obstacles[obstacle][0] - 1, self.obstacles[obstacle][1])
            case 3:
                result = (self.obstacles[obstacle][0] + 1, self.obstacles[obstacle][1])
        if self.is_feasible(result):
            return True
        else:
            return False

    def is_feasible(self, position):
        if position[0] < 1:
            return False
        if position[0] > self.size:
            return False
        if position[1] < 1:
            return False
        if position[1] > self.size:
            return False
        if position in self.walls:
            return False
        return True

    def print_report(self, folder):
        import os
        text = ''
        text += '{:03d}'.format(
            len(self.entered_restricted_areas)) + ' / ' + '{:03d}'.format(
            len(self.restricted_areas)) + ' restricted_areas killed\n'
        text += '{:03d}'.format(
            len(self.hit_obstacles)) + ' / ' + '{:03d}'.format(
            len(self.obstacles)) + ' obstacles killed\n'
        text += str(len(self.times)) + ' steps taken\n'
        if len(self.times[1:]) > 0:
            text += str(sum(self.times[1:]) / len(self.times[1:])) + ' average time per step\n'
            text += str(max(self.times[1:])) + ' max time per step\n'
            text += str(min(self.times[1:])) + ' min time per step\n'
        else:
            text += str(0) + ' average time per step\n'
            text += str(0) + ' max time per step\n'
            text += str(0) + ' min time per step\n'
        text += str(self.times[0]) + ' startup time\n'
        text += str(self.interceptions) + ' interceptions from ASP'

        print(text)

    def save(self):
        import os
        text = ''
        text += '#const size =' + str(self.size) + '.\n'
        text += '\n'
        text += 'robot' + str(self.robot) + '.\n'
        text += 'target' + str(self.target) + '.\n'
        text += '\n'
        for wall in self.walls:
            text += 'wall' + str(wall) + '. '
        text += '\n'
        text += '\n'
        for restricted_area in self.restricted_areas:
            text += 'restricted_area' + str(restricted_area) + '. '
        text += '\n'
        text += '\n'
        for obstacle in self.obstacles:
            text += 'obstacle' + str(obstacle) + '. '

        os.makedirs(
            os.path.dirname('instances/%s.lp' % self.name),
            exist_ok=True)
        file = open('instances/%s.lp' % self.name, 'w')
        file.write(text)
        file.close()

    def update_obstacle(self):
        list_of_obstacles = self.robot_controller.get_objects()
        print(list_of_obstacles)
        '''for i in range(len(list_of_obstacles)):
            pos = list_of_obstacles[i]['cell']
            self.add_obstacle(pos)'''