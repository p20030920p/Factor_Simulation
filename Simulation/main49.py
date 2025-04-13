import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import time
import os
import datetime
from matplotlib.animation import FuncAnimation
from collections import deque, defaultdict
import random
import math


class Material:
    def __init__(self, material_id, material_type):
        self.id = material_id
        self.type = material_type  # "empty", "green", "yellow", "red"
        self.location = None
        self.status = "waiting"  # waiting, processing, finished
        self.process_start_time = 0
        self.process_end_time = 0
        self.creation_time = 0
        self.completion_time = 0
        self.path = []
        self.current_machine = None

    def __repr__(self):
        return f"Material({self.id}, {self.type}, at {self.location})"


class Machine:
    def __init__(self, machine_id, machine_type, position, processing_time, capacity=1):
        self.id = machine_id
        self.type = machine_type
        self.position = position
        self.processing_time = processing_time
        self.capacity = capacity
        self.materials = []
        self.status = "idle"  # idle, processing, finished
        self.process_start_time = 0
        self.process_end_time = 0
        self.total_processing_time = 0
        self.processed_materials = 0

    def is_available(self):
        return len(self.materials) < self.capacity and self.status != "processing"

    def start_processing(self, current_time):
        if len(self.materials) > 0 and self.status == "idle":
            self.status = "processing"
            self.process_start_time = current_time
            self.process_end_time = current_time + self.processing_time
            for material in self.materials:
                material.status = "processing"
                material.process_start_time = current_time
                material.process_end_time = current_time + self.processing_time
            return True
        return False

    def check_process_completion(self, current_time):
        if self.status == "processing" and current_time >= self.process_end_time:
            self.status = "finished"
            self.total_processing_time += (self.process_end_time - self.process_start_time)
            self.processed_materials += len(self.materials)
            for material in self.materials:
                material.status = "finished"
                material.completion_time = current_time
            return True
        return False

    def __repr__(self):
        return f"Machine({self.id}, {self.type}, at {self.position}, status={self.status})"


class Area:
    def __init__(self, area_id, area_type, top_left, bottom_right, capacity=float('inf')):
        self.id = area_id
        self.type = area_type
        self.top_left = top_left
        self.bottom_right = bottom_right
        self.capacity = capacity
        self.materials = []

    def is_full(self):
        return len(self.materials) >= self.capacity

    def is_empty(self):
        return len(self.materials) == 0

    def add_material(self, material):
        if not self.is_full():
            self.materials.append(material)
            # Assign location within the area
            x = random.uniform(self.top_left[0], self.bottom_right[0])
            y = random.uniform(self.top_left[1], self.bottom_right[1])
            material.location = (x, y)
            return True
        return False

    def remove_material(self, material):
        if material in self.materials:
            self.materials.remove(material)
            return True
        return False

    def get_center(self):
        return ((self.top_left[0] + self.bottom_right[0]) / 2,
                (self.top_left[1] + self.bottom_right[1]) / 2)

    def __repr__(self):
        return f"Area({self.id}, {self.type}, materials={len(self.materials)}/{self.capacity})"


class Robot:
    def __init__(self, robot_id, position, speed=1.0, capacity=1):
        self.id = robot_id
        self.position = position
        self.speed = speed
        self.capacity = capacity
        self.carrying = []  # List of materials being carried
        self.status = "idle"  # idle, moving, loading, unloading
        self.destination = None
        self.path = []
        self.current_task = None
        self.total_distance = 0
        self.completed_tasks = 0
        self.busy_time = 0
        self.last_status_change = 0

    def is_available(self):
        return self.status == "idle" and len(self.carrying) < self.capacity

    def move_to(self, destination, current_time, grid_map):
        self.destination = destination
        self.status = "moving"
        self.last_status_change = current_time
        # Calculate path using A* algorithm
        self.path = astar_path(grid_map, self.position, destination)
        return len(self.path) > 0

    def update_position(self, current_time, time_step=1):
        if self.status == "moving" and len(self.path) > 0:
            # Calculate how many steps we can move in this time step
            distance_per_step = self.speed * time_step
            total_distance_moved = 0

            while self.path and total_distance_moved < distance_per_step:
                next_pos = self.path[0]
                # Calculate distance to next point
                dx = next_pos[0] - self.position[0]
                dy = next_pos[1] - self.position[1]
                distance_to_next = math.sqrt(dx * dx + dy * dy)

                if distance_to_next <= distance_per_step - total_distance_moved:
                    # We can reach the next point
                    self.position = next_pos
                    self.path.pop(0)
                    total_distance_moved += distance_to_next
                    self.total_distance += distance_to_next
                else:
                    # Move partially towards the next point
                    ratio = (distance_per_step - total_distance_moved) / distance_to_next
                    self.position = (
                        self.position[0] + dx * ratio,
                        self.position[1] + dy * ratio
                    )
                    self.total_distance += (distance_per_step - total_distance_moved)
                    total_distance_moved = distance_per_step

            # Check if we've reached the destination
            if not self.path:
                self.status = "idle"
                self.busy_time += (current_time - self.last_status_change)
                self.last_status_change = current_time
                return True
        return False

    def pick_material(self, material, current_time):
        if len(self.carrying) < self.capacity:
            self.carrying.append(material)
            self.status = "loading"
            self.busy_time += (current_time - self.last_status_change)
            self.last_status_change = current_time
            return True
        return False

    def drop_material(self, area, current_time):
        if self.carrying and not area.is_full():
            for material in self.carrying:
                area.add_material(material)
            self.completed_tasks += len(self.carrying)
            self.carrying = []
            self.status = "idle"
            self.busy_time += (current_time - self.last_status_change)
            self.last_status_change = current_time
            return True
        return False

    def __repr__(self):
        return f"Robot({self.id}, at {self.position}, carrying={len(self.carrying)}, status={self.status})"


class Task:
    def __init__(self, task_id, task_type, priority, source, destination, materials):
        self.id = task_id
        self.type = task_type  # "transport", "process"
        self.priority = priority
        self.source = source
        self.destination = destination
        self.materials = materials
        self.status = "pending"  # pending, assigned, completed
        self.assigned_robot = None
        self.creation_time = 0
        self.assignment_time = 0
        self.completion_time = 0

    def __lt__(self, other):
        return self.priority < other.priority

    def __repr__(self):
        return f"Task({self.id}, {self.type}, priority={self.priority}, status={self.status})"


def create_grid_map(width, height, areas, machines, obstacles=None):
    grid = np.ones((width, height), dtype=bool)  # True means traversable

    # Mark areas as traversable
    for area in areas:
        x_min, y_min = int(area.top_left[0]), int(area.top_left[1])
        x_max, y_max = int(area.bottom_right[0]), int(area.bottom_right[1])
        grid[x_min:x_max + 1, y_min:y_max + 1] = True

    # Mark machines as obstacles
    for machine in machines:
        x, y = int(machine.position[0]), int(machine.position[1])
        grid[max(0, x - 1):min(width, x + 2), max(0, y - 1):min(height, y + 2)] = False

    # Add additional obstacles if provided
    if obstacles:
        for obstacle in obstacles:
            x_min, y_min = int(obstacle[0][0]), int(obstacle[0][1])
            x_max, y_max = int(obstacle[1][0]), int(obstacle[1][1])
            grid[x_min:x_max + 1, y_min:y_max + 1] = False

    return grid


def heuristic(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def astar_path(grid, start, goal):
    # Discretize start and goal positions
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    # If start or goal is not traversable, find nearest traversable point
    if not grid[start] or not grid[goal]:
        # Find nearest traversable point for start
        if not grid[start]:
            min_dist = float('inf')
            for i in range(max(0, start[0] - 10), min(grid.shape[0], start[0] + 11)):
                for j in range(max(0, start[1] - 10), min(grid.shape[1], start[1] + 11)):
                    if grid[i, j]:
                        dist = heuristic((i, j), start)
                        if dist < min_dist:
                            min_dist = dist
                            start = (i, j)

        # Find nearest traversable point for goal
        if not grid[goal]:
            min_dist = float('inf')
            for i in range(max(0, goal[0] - 10), min(grid.shape[0], goal[0] + 11)):
                for j in range(max(0, goal[1] - 10), min(grid.shape[1], goal[1] + 11)):
                    if grid[i, j]:
                        dist = heuristic((i, j), goal)
                        if dist < min_dist:
                            min_dist = dist
                            goal = (i, j)

    # A* algorithm
    closed_set = set()
    open_set = {start}
    came_from = {}

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    open_heap = [(f_score[start], start)]
    heapq.heapify(open_heap)

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]

    while open_set:
        current = heapq.heappop(open_heap)[1]

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            # Convert back to float for smooth movement
            return [(float(x), float(y)) for x, y in path]

        open_set.remove(current)
        closed_set.add(current)

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if (neighbor[0] < 0 or neighbor[0] >= grid.shape[0] or
                    neighbor[1] < 0 or neighbor[1] >= grid.shape[1]):
                continue

            if not grid[neighbor] or neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + ((dx * dx + dy * dy) ** 0.5)

            if neighbor not in open_set or tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)

                if neighbor not in open_set:
                    open_set.add(neighbor)
                    heapq.heappush(open_heap, (f_score[neighbor], neighbor))

    # No path found
    return []


class FactorySimulation:
    def __init__(self, width=1000, height=1000):
        self.width = width
        self.height = height
        self.current_time = 0
        self.robots = []
        self.machines = []
        self.areas = []
        self.materials = []
        self.tasks = []
        self.task_queue = []
        self.grid_map = None
        self.deadlock_count = 0
        self.last_progress_time = 0
        self.deadlock_threshold = 5000  # 5000 seconds without progress considered deadlock

        # Statistics
        self.stats = {
            'red_buckets_produced': 0,
            'green_buckets_produced': 0,
            'yellow_buckets_produced': 0,
            'lead_times': {
                'green': [],
                'yellow': [],
                'red': []
            },
            'robot_distances': {},
            'robot_tasks': {},
            'machine_utilization': {},
            'material_states': []
        }

        # Visualization data
        self.vis_data = {
            'time': [],
            'red_count': [],
            'robot_positions': defaultdict(list),
            'material_positions': defaultdict(list)
        }

        # Initialize factory layout
        self.setup_factory()

    def setup_factory(self):
        # Create areas
        # Empty bucket storage (leftmost area)
        self.areas.append(Area(0, "empty_storage", (50, 50), (100, 950), 540))

        # Carding areas
        for i in range(10):
            y_pos = 50 + i * 90
            self.areas.append(Area(1 + i * 2, "carding_waiting", (150, y_pos), (200, y_pos + 80), 2))
            self.areas.append(Area(2 + i * 2, "carding_finished", (250, y_pos), (300, y_pos + 80), 2))

        # First drawing areas
        for i in range(5):
            y_pos = 50 + i * 180
            self.areas.append(Area(21 + i * 2, "drawing1_waiting", (350, y_pos), (400, y_pos + 160), 12))
            self.areas.append(Area(22 + i * 2, "drawing1_finished", (450, y_pos), (500, y_pos + 160), 2))

        # Second drawing areas
        for i in range(5):
            y_pos = 50 + i * 180
            self.areas.append(Area(31 + i * 2, "drawing2_waiting", (550, y_pos), (600, y_pos + 160), 12))
            self.areas.append(Area(32 + i * 2, "drawing2_finished", (650, y_pos), (700, y_pos + 160), 2))

        # Roving areas
        for i in range(5):
            y_pos = 50 + i * 180
            self.areas.append(Area(41 + i * 2, "roving_waiting", (750, y_pos), (800, y_pos + 160), 6))
            self.areas.append(Area(42 + i * 2, "roving_finished", (850, y_pos), (900, y_pos + 160), 3))

        # Create machines
        # Carding machines
        for i in range(10):
            y_pos = 90 + i * 90
            self.machines.append(Machine(i, "carding", (225, y_pos), 4225.94))

        # First drawing machines
        for i in range(5):
            y_pos = 140 + i * 180
            self.machines.append(Machine(10 + i, "drawing1", (425, y_pos), 2594.75, 12))

        # Second drawing machines
        for i in range(5):
            y_pos = 140 + i * 180
            self.machines.append(Machine(15 + i, "drawing2", (625, y_pos), 2594.75, 12))

        # Roving machines
        for i in range(5):
            y_pos = 140 + i * 180
            self.machines.append(Machine(20 + i, "roving", (825, y_pos), 2594.75, 6))

        # Create robots
        robot_positions = [
            (75, 500),  # Near empty bucket storage
            (175, 200),  # Near carding waiting
            (275, 400),  # Near carding finished
            (375, 600),  # Near drawing1 waiting
            (475, 800),  # Near drawing1 finished
            (575, 200),  # Near drawing2 waiting
            (675, 400),  # Near drawing2 finished
            (775, 600),  # Near roving waiting
            (875, 800),  # Near roving finished
            (500, 500)  # Middle of the factory
        ]

        for i, pos in enumerate(robot_positions):
            self.robots.append(Robot(i, pos))
            self.stats['robot_distances'][i] = 0
            self.stats['robot_tasks'][i] = 0

        # Create empty buckets
        for i in range(540):
            material = Material(i, "empty")
            self.materials.append(material)
            self.areas[0].add_material(material)
            material.creation_time = 0

        # Create grid map for pathfinding
        self.grid_map = create_grid_map(self.width, self.height, self.areas, self.machines)

    def generate_tasks(self):
        # Generate transport tasks based on current state

        # 1. Empty buckets to carding machines
        for area in self.areas:
            if area.type == "carding_waiting" and not area.is_full():
                # Find empty buckets
                empty_area = self.areas[0]
                if not empty_area.is_empty():
                    empty_materials = [m for m in empty_area.materials if m.type == "empty"][
                                      :area.capacity - len(area.materials)]
                    if empty_materials:
                        task = Task(len(self.tasks), "transport", 1, empty_area, area, empty_materials)
                        task.creation_time = self.current_time
                        self.tasks.append(task)
                        heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # 2. Carding finished to drawing1 waiting
        for area in self.areas:
            if area.type == "drawing1_waiting" and not area.is_full():
                # Find green buckets from carding finished areas
                available_green = []
                for source_area in self.areas:
                    if source_area.type == "carding_finished" and not source_area.is_empty():
                        green_materials = [m for m in source_area.materials if m.type == "green"]
                        available_green.extend(green_materials)

                # We need 6 green buckets for drawing1 (per batch)
                needed_count = min(12, area.capacity - len(area.materials))
                needed_count = needed_count - (needed_count % 6)  # Ensure multiple of 6

                if needed_count > 0 and len(available_green) >= needed_count:
                    materials_to_move = available_green[:needed_count]
                    task = Task(len(self.tasks), "transport", 2, None, area, materials_to_move)
                    task.creation_time = self.current_time
                    self.tasks.append(task)
                    heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # 3. Drawing1 finished to drawing2 waiting
        for area in self.areas:
            if area.type == "drawing2_waiting" and not area.is_full():
                # Find yellow buckets from drawing1 finished areas
                available_yellow = []
                for source_area in self.areas:
                    if source_area.type == "drawing1_finished" and not source_area.is_empty():
                        yellow_materials = [m for m in source_area.materials if m.type == "yellow"]
                        available_yellow.extend(yellow_materials)

                # We need 6 yellow buckets for drawing2 (per batch)
                needed_count = min(12, area.capacity - len(area.materials))
                needed_count = needed_count - (needed_count % 6)  # Ensure multiple of 6

                if needed_count > 0 and len(available_yellow) >= needed_count:
                    materials_to_move = available_yellow[:needed_count]
                    task = Task(len(self.tasks), "transport", 3, None, area, materials_to_move)
                    task.creation_time = self.current_time
                    self.tasks.append(task)
                    heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # 4. Drawing2 finished to roving waiting
        for area in self.areas:
            if area.type == "roving_waiting" and not area.is_full():
                # Find yellow buckets from drawing2 finished areas
                available_yellow = []
                for source_area in self.areas:
                    if source_area.type == "drawing2_finished" and not source_area.is_empty():
                        yellow_materials = [m for m in source_area.materials if m.type == "yellow"]
                        available_yellow.extend(yellow_materials)

                # We need 6 yellow buckets for roving (per batch)
                needed_count = min(6, area.capacity - len(area.materials))
                needed_count = needed_count - (needed_count % 6)  # Ensure multiple of 6

                if needed_count > 0 and len(available_yellow) >= needed_count:
                    materials_to_move = available_yellow[:needed_count]
                    task = Task(len(self.tasks), "transport", 4, None, area, materials_to_move)
                    task.creation_time = self.current_time
                    self.tasks.append(task)
                    heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # Generate processing tasks

        # 1. Process empty buckets in carding machines
        for machine in self.machines:
            if machine.type == "carding" and machine.status == "idle":
                # Find carding waiting area for this machine
                machine_idx = machine.id
                waiting_area = self.areas[1 + machine_idx * 2]
                finished_area = self.areas[2 + machine_idx * 2]

                if not waiting_area.is_empty() and not finished_area.is_full():
                    empty_materials = [m for m in waiting_area.materials if m.type == "empty"]
                    if empty_materials:
                        task = Task(len(self.tasks), "process", 5, waiting_area, machine, [empty_materials[0]])
                        task.creation_time = self.current_time
                        self.tasks.append(task)
                        heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # 2. Process green buckets in drawing1 machines
        for machine in self.machines:
            if machine.type == "drawing1" and machine.status == "idle":
                # Find drawing1 waiting area for this machine
                machine_idx = machine.id - 10
                waiting_area = self.areas[21 + machine_idx * 2]
                finished_area = self.areas[22 + machine_idx * 2]

                if len(waiting_area.materials) >= 6 and not finished_area.is_full():
                    green_materials = [m for m in waiting_area.materials if m.type == "green"]
                    if len(green_materials) >= 6:
                        task = Task(len(self.tasks), "process", 6, waiting_area, machine, green_materials[:6])
                        task.creation_time = self.current_time
                        self.tasks.append(task)
                        heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # 3. Process yellow buckets in drawing2 machines
        for machine in self.machines:
            if machine.type == "drawing2" and machine.status == "idle":
                # Find drawing2 waiting area for this machine
                machine_idx = machine.id - 15
                waiting_area = self.areas[31 + machine_idx * 2]
                finished_area = self.areas[32 + machine_idx * 2]

                if len(waiting_area.materials) >= 6 and not finished_area.is_full():
                    yellow_materials = [m for m in waiting_area.materials if m.type == "yellow"]
                    if len(yellow_materials) >= 6:
                        task = Task(len(self.tasks), "process", 7, waiting_area, machine, yellow_materials[:6])
                        task.creation_time = self.current_time
                        self.tasks.append(task)
                        heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # 4. Process yellow buckets in roving machines
        for machine in self.machines:
            if machine.type == "roving" and machine.status == "idle":
                # Find roving waiting area for this machine
                machine_idx = machine.id - 20
                waiting_area = self.areas[41 + machine_idx * 2]
                finished_area = self.areas[42 + machine_idx * 2]

                if len(waiting_area.materials) >= 6 and not finished_area.is_full():
                    yellow_materials = [m for m in waiting_area.materials if m.type == "yellow"]
                    if len(yellow_materials) >= 6:
                        task = Task(len(self.tasks), "process", 8, waiting_area, machine, yellow_materials[:6])
                        task.creation_time = self.current_time
                        self.tasks.append(task)
                        heapq.heappush(self.task_queue, (task.priority, task.id, task))

        # 5. Move processed materials from machines to finished areas
        for machine in self.machines:
            if machine.status == "finished" and machine.materials:
                # Find appropriate finished area
                finished_area = None
                if machine.type == "carding":
                    machine_idx = machine.id
                    finished_area = self.areas[2 + machine_idx * 2]
                elif machine.type == "drawing1":
                    machine_idx = machine.id - 10
                    finished_area = self.areas[22 + machine_idx * 2]
                elif machine.type == "drawing2":
                    machine_idx = machine.id - 15
                    finished_area = self.areas[32 + machine_idx * 2]
                elif machine.type == "roving":
                    machine_idx = machine.id - 20
                    finished_area = self.areas[42 + machine_idx * 2]

                if finished_area and not finished_area.is_full():
                    task = Task(len(self.tasks), "transport", 0, machine, finished_area, machine.materials)
                    task.creation_time = self.current_time
                    self.tasks.append(task)
                    heapq.heappush(self.task_queue, (task.priority, task.id, task))

    def assign_tasks(self):
        # Assign tasks to available robots
        available_robots = [robot for robot in self.robots if robot.status == "idle"]

        if not available_robots or not self.task_queue:
            return

        task_assignments = []
        for robot in available_robots:
            if not self.task_queue:
                break

            _, _, task = heapq.heappop(self.task_queue)

            if task.status == "pending":
                task.status = "assigned"
                task.assigned_robot = robot
                task.assignment_time = self.current_time
                task_assignments.append((robot, task))

        # Process assignments
        for robot, task in task_assignments:
            if task.type == "transport":
                # Calculate source position
                if task.source is None:
                    # Multiple sources, find the closest one containing materials
                    materials_by_area = {}
                    for material in task.materials:
                        for area in self.areas:
                            if material in area.materials:
                                if area not in materials_by_area:
                                    materials_by_area[area] = []
                                materials_by_area[area].append(material)
                                break

                    if not materials_by_area:
                        # Materials not found, skip task
                        task.status = "pending"
                        heapq.heappush(self.task_queue, (task.priority, task.id, task))
                        continue

                    # Find closest area with materials
                    closest_area = min(materials_by_area.keys(),
                                       key=lambda a: heuristic(robot.position, a.get_center()))

                    task.source = closest_area
                    task.materials = materials_by_area[closest_area]

                source_pos = task.source.get_center()
                robot.move_to(source_pos, self.current_time, self.grid_map)
                robot.current_task = task

            elif task.type == "process":
                # Move materials from waiting area to machine
                source_pos = task.source.get_center()
                robot.move_to(source_pos, self.current_time, self.grid_map)
                robot.current_task = task

    def process_robot_actions(self):
        for robot in self.robots:
            if robot.status == "moving":
                # Update robot position
                reached_destination = robot.update_position(self.current_time)

                if reached_destination and robot.current_task:
                    task = robot.current_task

                    if task.type == "transport":
                        if not robot.carrying:
                            # At source, pick up materials
                            all_picked = True
                            for material in task.materials:
                                if task.source.remove_material(material):
                                    robot.pick_material(material, self.current_time)
                                else:
                                    all_picked = False

                            if not all_picked:
                                # Some materials were not found, retry later
                                task.status = "pending"
                                heapq.heappush(self.task_queue, (task.priority, task.id, task))
                                robot.current_task = None
                                continue

                            # Head to destination
                            dest_pos = task.destination.get_center()
                            robot.move_to(dest_pos, self.current_time, self.grid_map)
                        else:
                            # At destination, drop materials
                            success = robot.drop_material(task.destination, self.current_time)
                            if success:
                                task.status = "completed"
                                task.completion_time = self.current_time
                                robot.current_task = None

                                # If materials were dropped at a processing area, transform them
                                if isinstance(task.destination, Area) and "waiting" in task.destination.type:
                                    continue  # Just leave materials in waiting area

                                # If materials were dropped in finished area from a machine, handle transformation
                                elif isinstance(task.source, Machine) and isinstance(task.destination, Area):
                                    machine = task.source

                                    # Remove materials from machine
                                    machine.materials = []
                                    machine.status = "idle"

                                    # Transform materials based on machine type
                                    if machine.type == "carding":
                                        for material in task.materials:
                                            material.type = "green"
                                            self.stats['green_buckets_produced'] += 1
                                            lead_time = self.current_time - material.creation_time
                                            self.stats['lead_times']['green'].append(lead_time)

                                    elif machine.type == "drawing1":
                                        # 6 green buckets -> 1 yellow bucket
                                        yellow_id = len(self.materials)
                                        yellow_material = Material(yellow_id, "yellow")
                                        yellow_material.creation_time = self.current_time
                                        yellow_material.location = task.destination.get_center()
                                        self.materials.append(yellow_material)
                                        task.destination.add_material(yellow_material)
                                        self.stats['yellow_buckets_produced'] += 1

                                    elif machine.type == "drawing2":
                                        # 6 yellow buckets -> 1 yellow bucket (same ratio but for second drawing)
                                        yellow_id = len(self.materials)
                                        yellow_material = Material(yellow_id, "yellow")
                                        yellow_material.creation_time = self.current_time
                                        yellow_material.location = task.destination.get_center()
                                        self.materials.append(yellow_material)
                                        task.destination.add_material(yellow_material)
                                        self.stats['yellow_buckets_produced'] += 1

                                    elif machine.type == "roving":
                                        # 6 yellow buckets -> 1 red bucket
                                        red_id = len(self.materials)
                                        red_material = Material(red_id, "red")
                                        red_material.creation_time = self.current_time
                                        red_material.location = task.destination.get_center()
                                        self.materials.append(red_material)
                                        task.destination.add_material(red_material)
                                        self.stats['red_buckets_produced'] += 1
                                        lead_time = self.current_time - red_material.creation_time
                                        self.stats['lead_times']['red'].append(lead_time)
                            else:
                                # Destination is full, retry later
                                task.status = "pending"
                                heapq.heappush(self.task_queue, (task.priority, task.id, task))
                                robot.current_task = None

                    elif task.type == "process":
                        if not robot.carrying:
                            # At source, pick up materials for processing
                            all_picked = True
                            for material in task.materials:
                                if task.source.remove_material(material):
                                    robot.pick_material(material, self.current_time)
                                else:
                                    all_picked = False

                            if not all_picked:
                                # Some materials were not found, retry later
                                task.status = "pending"
                                heapq.heappush(self.task_queue, (task.priority, task.id, task))
                                robot.current_task = None
                                continue

                            # Head to machine
                            machine_pos = task.destination.position
                            robot.move_to(machine_pos, self.current_time, self.grid_map)
                        else:
                            # At machine, start processing
                            machine = task.destination

                            if machine.is_available():
                                # Load materials into machine
                                for material in robot.carrying:
                                    machine.materials.append(material)
                                robot.carrying = []

                                # Start processing
                                machine.start_processing(self.current_time)

                                task.status = "completed"
                                task.completion_time = self.current_time
                                robot.current_task = None
                            else:
                                # Machine is not available, retry later
                                task.status = "pending"
                                heapq.heappush(self.task_queue, (task.priority, task.id, task))
                                robot.current_task = None

    def update_machines(self):
        for machine in self.machines:
            if machine.status == "processing":
                machine.check_process_completion(self.current_time)

    def check_for_deadlocks(self):
        red_count = sum(1 for m in self.materials if m.type == "red")

        # Check if there has been progress
        if red_count > 0 or any(robot.status != "idle" for robot in self.robots) or any(
                machine.status != "idle" for machine in self.machines):
            self.last_progress_time = self.current_time

        # If no progress for deadlock_threshold seconds, attempt to resolve
        if self.current_time - self.last_progress_time > self.deadlock_threshold:
            self.deadlock_count += 1
            self.last_progress_time = self.current_time

            # Reset all processing machines
            for machine in self.machines:
                if machine.status == "processing":
                    machine.status = "finished"
                    machine.check_process_completion(self.current_time)

            # Reset all stuck robots
            for robot in self.robots:
                if robot.status == "moving" and robot.current_task:
                    robot.status = "idle"
                    task = robot.current_task
                    task.status = "pending"
                    heapq.heappush(self.task_queue, (task.priority, task.id, task))
                    robot.current_task = None

            print(f"Deadlock detected and resolved at time {self.current_time}")

    def update_statistics(self):
        # Update robot statistics
        for robot in self.robots:
            self.stats['robot_distances'][robot.id] = robot.total_distance
            self.stats['robot_tasks'][robot.id] = robot.completed_tasks

        # Update machine utilization
        for machine in self.machines:
            self.stats['machine_utilization'][machine.id] = machine.total_processing_time / max(1, self.current_time)

        # Record material counts
        empty_count = sum(1 for m in self.materials if m.type == "empty")
        green_count = sum(1 for m in self.materials if m.type == "green")
        yellow_count = sum(1 for m in self.materials if m.type == "yellow")
        red_count = sum(1 for m in self.materials if m.type == "red")

        self.stats['material_states'].append({
            'time': self.current_time,
            'empty': empty_count,
            'green': green_count,
            'yellow': yellow_count,
            'red': red_count
        })

        # Update visualization data
        self.vis_data['time'].append(self.current_time)
        self.vis_data['red_count'].append(red_count)

        for robot in self.robots:
            self.vis_data['robot_positions'][robot.id].append(robot.position)

        for material in self.materials:
            if material.location:
                self.vis_data['material_positions'][material.id].append(material.location)

    def run(self, max_time=100000, target_red_buckets=15, update_interval=1000):
        print("启动工厂自动化生产模拟系统...")
        start_time = time.time()

        self.current_time = 0
        self.last_progress_time = 0

        next_update = update_interval

        while self.current_time < max_time:
            # Generate and assign tasks
            self.generate_tasks()
            self.assign_tasks()

            # Process robot actions
            self.process_robot_actions()

            # Update machines
            self.update_machines()

            # Check for deadlocks
            self.check_for_deadlocks()

            # Update statistics
            self.update_statistics()

            # Check if we've reached the target
            red_count = sum(1 for m in self.materials if m.type == "red")
            if red_count >= target_red_buckets:
                print(f"目标已达成！生产了 {red_count} 个红桶，用时 {self.current_time} 秒")
                break

            # Print progress update
            if self.current_time >= next_update:
                print(f"时间: {self.current_time}秒, 红桶数量: {red_count}/{target_red_buckets}")
                next_update += update_interval

            # Increment time
            self.current_time += 1

        # Final status update
        red_count = sum(1 for m in self.materials if m.type == "red")
        print(f"时间: {self.current_time}秒, 红桶数量: {red_count}/{target_red_buckets}")

        # Generate final report
        self.generate_report(time.time() - start_time)

        # Visualize results
        try:
            self.visualize_results()
        except Exception as e:
            print(f"可视化更新错误: {str(e)}")

        return self.stats

    def generate_report(self, computation_time):
        # Calculate statistics
        red_count = sum(1 for m in self.materials if m.type == "red")
        empty_count = sum(1 for m in self.materials if m.type == "empty")
        green_count = sum(1 for m in self.materials if m.type == "green")
        yellow_count = sum(1 for m in self.materials if m.type == "yellow")

        # Theoretical vs actual time
        theoretical_time = 0
        if red_count > 0:
            # Time to produce one red bucket:
            # 1 empty -> 1 green: 4225.94s
            # 6 green -> 1 yellow: 2594.75s
            # 6 yellow -> 1 red: 2594.75s
            theoretical_time = 4225.94 + 2594.75 + 2594.75
            theoretical_efficiency = (theoretical_time / (self.current_time / red_count)) * 100
        else:
            theoretical_efficiency = 0.0

        # Find bottleneck
        machine_utilizations = {}
        for machine in self.machines:
            machine_type = machine.type
            if machine_type not in machine_utilizations:
                machine_utilizations[machine_type] = []
            utilization = machine.total_processing_time / max(1, self.current_time) * 100
            machine_utilizations[machine_type].append(utilization)

        bottleneck = "none"
        bottleneck_utilization = 0
        for machine_type, utilizations in machine_utilizations.items():
            avg_utilization = sum(utilizations) / len(utilizations)
            if avg_utilization > bottleneck_utilization:
                bottleneck_utilization = avg_utilization
                bottleneck = machine_type

        # Calculate lead times
        lead_times = self.stats['lead_times']
        avg_lead_times = {}
        for color, times in lead_times.items():
            if times:
                avg_lead_times[color] = sum(times) / len(times)
            else:
                avg_lead_times[color] = "未生产"

        # Print report
        print("\n===== 工厂模拟性能报告 =====")
        print(f"总模拟时间: {self.current_time}秒 ({self.current_time / 3600:.2f}小时)")
        print(f"物料数量: 空桶={empty_count}, 绿桶={green_count}, 黄桶={yellow_count}, 红桶={red_count}")
        print(f"红桶生产率: {red_count / (self.current_time / 3600):.2f}个/小时")
        print(f"流水线效率: {theoretical_efficiency:.2f}% (理论时间/实际时间)")
        print(f"生产瓶颈: {bottleneck}, 利用率: {bottleneck_utilization:.2f}%")
        print(f"总死锁次数: {self.deadlock_count}")

        print("\n生产提前期:")
        for color, avg_time in avg_lead_times.items():
            if isinstance(avg_time, (int, float)):
                print(f"  {color}: {avg_time:.2f}秒")
            else:
                print(f"  {color}: {avg_time}")

        print("\n机器利用率:")
        for machine_type, utilizations in machine_utilizations.items():
            avg_utilization = sum(utilizations) / len(utilizations)
            print(f"  {machine_type}: {avg_utilization:.2f}%")

        print("\n机器人统计:")
        for robot in self.robots:
            print(f"  机器人{robot.id}:")
            print(f"    总移动距离: {robot.total_distance:.2f}")
            print(f"    完成任务数: {robot.completed_tasks}")
            print(f"    利用率: {(robot.busy_time / max(1, self.current_time)) * 100:.2f}%")

        # Save output to directory
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"output_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)
        print(f"所有图表和数据已保存到目录: {output_dir}")

        # Save final summary
        print("\n===== 模拟完成 =====")
        print(f"总模拟时间: {self.current_time}秒")
        print(
            f"物料状态: {{'empty': {empty_count}, 'green': {green_count}, 'yellow': {yellow_count}, 'red': {red_count}}}")
        print(f"完成任务数: {sum(robot.completed_tasks for robot in self.robots)}")
        print(f"死锁次数: {self.deadlock_count}")
        print(f"效率比(理论/实际): {theoretical_efficiency:.2f}%")
        print(f"实际计算时间: {computation_time:.2f}秒")

        print("\n注: 详细报告已保存到log文件和输出目录中")

    def visualize_results(self):
        # Create output directory
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"output_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # 1. Plot red bucket production over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.vis_data['time'], self.vis_data['red_count'])
        plt.xlabel('Time (s)')
        plt.ylabel('Red Buckets Produced')
        plt.title('Red Bucket Production Over Time')
        plt.grid(True)
        plt.savefig(f"{output_dir}/red_bucket_production.png")

        # 2. Plot material counts over time
        times = [state['time'] for state in self.stats['material_states']]
        empty_counts = [state['empty'] for state in self.stats['material_states']]
        green_counts = [state['green'] for state in self.stats['material_states']]
        yellow_counts = [state['yellow'] for state in self.stats['material_states']]
        red_counts = [state['red'] for state in self.stats['material_states']]

        plt.figure(figsize=(10, 6))
        plt.plot(times, empty_counts, label='Empty')
        plt.plot(times, green_counts, label='Green')
        plt.plot(times, yellow_counts, label='Yellow')
        plt.plot(times, red_counts, label='Red')
        plt.xlabel('Time (s)')
        plt.ylabel('Count')
        plt.title('Material Counts Over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(f"{output_dir}/material_counts.png")

        # 3. Plot robot utilization
        robot_ids = list(self.stats['robot_tasks'].keys())
        tasks_completed = [self.stats['robot_tasks'][r_id] for r_id in robot_ids]
        distances_moved = [self.stats['robot_distances'][r_id] for r_id in robot_ids]

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

        ax1.bar(robot_ids, tasks_completed)
        ax1.set_xlabel('Robot ID')
        ax1.set_ylabel('Tasks Completed')
        ax1.set_title('Tasks Completed by Robot')

        ax2.bar(robot_ids, distances_moved)
        ax2.set_xlabel('Robot ID')
        ax2.set_ylabel('Distance Moved')
        ax2.set_title('Distance Moved by Robot')

        plt.tight_layout()
        plt.savefig(f"{output_dir}/robot_performance.png")

        # 4. Plot machine utilization
        machine_types = ['carding', 'drawing1', 'drawing2', 'roving']
        utilization_data = []

        for machine in self.machines:
            machine_type = machine.type
            utilization = machine.total_processing_time / max(1, self.current_time) * 100
            utilization_data.append((machine.id, machine_type, utilization))

        # Group by machine type
        utilization_by_type = {}
        for _, machine_type, util in utilization_data:
            if machine_type not in utilization_by_type:
                utilization_by_type[machine_type] = []
            utilization_by_type[machine_type].append(util)

        # Calculate average utilization by type
        avg_utilization = [sum(utilization_by_type.get(t, [0])) / len(utilization_by_type.get(t, [1])) for t in
                           machine_types]

        plt.figure(figsize=(10, 6))
        plt.bar(machine_types, avg_utilization)
        plt.xlabel('Machine Type')
        plt.ylabel('Average Utilization (%)')
        plt.title('Machine Utilization by Type')
        plt.grid(True, axis='y')
        plt.savefig(f"{output_dir}/machine_utilization.png")

        # 5. Create a layout visualization of the factory
        plt.figure(figsize=(12, 12))

        # Plot areas
        for area in self.areas:
            x, y = area.top_left
            width = area.bottom_right[0] - area.top_left[0]
            height = area.bottom_right[1] - area.top_left[1]

            color = 'lightgray'
            if 'empty' in area.type:
                color = 'white'
            elif 'carding' in area.type:
                color = 'lightblue'
            elif 'drawing1' in area.type:
                color = 'lightgreen'
            elif 'drawing2' in area.type:
                color = 'lightyellow'
            elif 'roving' in area.type:
                color = 'salmon'

            rect = patches.Rectangle((x, y), width, height, linewidth=1,
                                     edgecolor='black', facecolor=color, alpha=0.5)
            plt.gca().add_patch(rect)
            plt.text(x + width / 2, y + height / 2, area.type,
                     horizontalalignment='center', verticalalignment='center')

        # Plot machines
        for machine in self.machines:
            x, y = machine.position
            if machine.type == 'carding':
                color = 'blue'
            elif machine.type == 'drawing1':
                color = 'green'
            elif machine.type == 'drawing2':
                color = 'yellow'
            elif machine.type == 'roving':
                color = 'red'

            plt.scatter(x, y, c=color, s=100, edgecolor='black', zorder=2)
            plt.text(x, y - 10, f"{machine.type} {machine.id}",
                     horizontalalignment='center', verticalalignment='center')

        # Plot robots final positions
        for robot in self.robots:
            x, y = robot.position
            plt.scatter(x, y, c='black', marker='s', s=50, zorder=3)
            plt.text(x, y + 10, f"R{robot.id}", color='black',
                     horizontalalignment='center', verticalalignment='center')

        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.title('Factory Layout')
        plt.savefig(f"{output_dir}/factory_layout.png")

        # Save statistics to CSV
        with open(f"{output_dir}/simulation_stats.txt", 'w') as f:
            f.write("===== Factory Simulation Statistics =====\n")
            f.write(f"Total simulation time: {self.current_time} seconds\n")
            f.write(f"Red buckets produced: {sum(1 for m in self.materials if m.type == 'red')}\n")
            f.write(f"Deadlocks detected: {self.deadlock_count}\n")

            f.write("\nMaterial counts:\n")
            f.write(f"  Empty: {sum(1 for m in self.materials if m.type == 'empty')}\n")
            f.write(f"  Green: {sum(1 for m in self.materials if m.type == 'green')}\n")
            f.write(f"  Yellow: {sum(1 for m in self.materials if m.type == 'yellow')}\n")
            f.write(f"  Red: {sum(1 for m in self.materials if m.type == 'red')}\n")

            f.write("\nRobot statistics:\n")
            for robot in self.robots:
                f.write(f"  Robot {robot.id}:\n")
                f.write(f"    Tasks completed: {robot.completed_tasks}\n")
                f.write(f"    Distance moved: {robot.total_distance:.2f}\n")
                f.write(f"    Utilization: {(robot.busy_time / max(1, self.current_time)) * 100:.2f}%\n")

            f.write("\nMachine utilization:\n")
            for machine in self.machines:
                utilization = machine.total_processing_time / max(1, self.current_time) * 100
                f.write(f"  {machine.type} {machine.id}: {utilization:.2f}%\n")


def main():
    # Create and run the simulation
    simulation = FactorySimulation()
    simulation.run(max_time=1000000, target_red_buckets=15, update_interval=1000)


if __name__ == "__main__":
    main()