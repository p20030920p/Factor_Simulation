import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import time
import os
import datetime
from collections import deque, defaultdict
import random
import math
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class Material:
    def __init__(self, material_id, material_type):
        self.id = material_id
        self.type = material_type  # "empty", "green", "yellow", "red"
        self.location = None
        self.status = "waiting"  # waiting, processing, finished, in_transit
        self.process_start_time = 0
        self.process_end_time = 0
        self.creation_time = 0
        self.completion_time = 0
        self.batch_id = None  # Track which batch this material belongs to
        self.waiting_time = 0  # Accumulated time material waits for processing
        self.task_priority_boost = 0  # Coefficient to increase material task priority

    def __repr__(self):
        return f"Material({self.id}, {self.type}, at {self.location}, status={self.status})"

    def update_waiting_time(self, time_increment):
        """Update material waiting time, used for dynamic task priority adjustment"""
        if self.status == "waiting":
            self.waiting_time += time_increment
            # Increase priority as waiting time increases
            self.task_priority_boost = self.waiting_time / 1000  # Control boost amount


class Machine:
    def __init__(self, machine_id, machine_type, position, processing_time, capacity=1):
        self.id = machine_id
        self.type = machine_type
        self.position = position
        self.processing_time = processing_time
        self.capacity = capacity
        self.materials = []
        self.status = "idle"  # idle, processing, finished, waiting_for_pickup
        self.process_start_time = 0
        self.process_end_time = 0
        self.total_processing_time = 0
        self.processed_materials = 0
        self.idle_time = 0  # Machine idle time
        self.waiting_for_pickup_time = 0  # Time waiting for pickup
        self.last_status_change = 0
        self.consecutive_failures = 0  # Count of consecutive processing failures
        self.maintenance_time = 0  # Maintenance time
        self.current_batch_id = None  # Track which batch is being processed
        self.last_emergency_task = 0  # Track when last emergency task was created
        self.has_pickup_task = False  # Flag to indicate if a pickup task exists
        self.stuck_since = 0  # Track when machine became stuck

    def is_available(self):
        """Determine if machine is available"""
        return (self.status == "idle" and
                len(self.materials) < self.capacity and
                self.maintenance_time == 0)

    def start_processing(self, current_time):
        """Start processing materials"""
        if len(self.materials) > 0 and self.status == "idle":
            self.status = "processing"
            self.process_start_time = current_time
            self.process_end_time = current_time + self.processing_time
            self.last_status_change = current_time
            self.consecutive_failures = 0  # Reset failure count

            # Set batch_id for tracking
            if self.materials and self.materials[0].batch_id:
                self.current_batch_id = self.materials[0].batch_id

            for material in self.materials:
                material.status = "processing"
                material.process_start_time = current_time
                material.process_end_time = current_time + self.processing_time
                material.waiting_time = 0  # Reset waiting time

            logger.info(
                f"Machine {self.id} ({self.type}) started processing {len(self.materials)} materials, batch {self.current_batch_id}")
            return True
        else:
            self.consecutive_failures += 1
            return False

    def check_process_completion(self, current_time):
        """Check if processing is complete"""
        if self.status == "processing" and current_time >= self.process_end_time:
            self.status = "waiting_for_pickup"
            self.total_processing_time += (self.process_end_time - self.process_start_time)
            self.processed_materials += len(self.materials)
            self.last_status_change = current_time
            self.has_pickup_task = False  # Reset pickup task flag
            self.stuck_since = current_time  # Start tracking stuck time

            for material in self.materials:
                material.status = "finished"
                material.completion_time = current_time

            logger.info(
                f"Machine {self.id} ({self.type}) completed processing {len(self.materials)} materials, batch {self.current_batch_id}")
            return True
        return False

    def update_machine_status(self, current_time, time_step):
        """Update machine status and time statistics"""
        if self.status == "idle":
            self.idle_time += time_step
        elif self.status == "waiting_for_pickup":
            self.waiting_for_pickup_time += time_step

            # Auto-reset if waiting too long with no materials
            if self.waiting_for_pickup_time > 2000 and not self.materials:
                self.status = "idle"
                self.waiting_for_pickup_time = 0
                self.has_pickup_task = False
                logger.warning(f"Machine {self.id} auto-reset to idle state after waiting too long")

        # Update maintenance time
        if self.maintenance_time > 0:
            actual_maintenance = min(time_step, self.maintenance_time)
            self.maintenance_time -= actual_maintenance
            if self.maintenance_time == 0:
                logger.info(f"Machine {self.id} ({self.type}) completed maintenance")

    def require_maintenance(self, duration=300):
        """Machine needs maintenance"""
        if self.status == "idle" or self.consecutive_failures > 5:
            self.status = "idle"
            self.maintenance_time = duration
            self.materials = []  # Clear current materials
            self.consecutive_failures = 0
            self.has_pickup_task = False
            logger.info(f"Machine {self.id} ({self.type}) entering maintenance mode for {duration} seconds")
            return True
        return False

    def is_stuck(self, current_time, threshold=3000):
        """Check if machine is stuck in waiting_for_pickup state"""
        if self.status == "waiting_for_pickup" and current_time - self.stuck_since > threshold:
            return True
        return False

    def __repr__(self):
        status_str = self.status
        if self.maintenance_time > 0:
            status_str += f"(maintenance:{self.maintenance_time}s)"
        pickup_str = " [has pickup task]" if self.has_pickup_task else ""
        return f"Machine({self.id}, {self.type}, at {self.position}, status={status_str}{pickup_str}, materials={len(self.materials)}/{self.capacity})"


class Area:
    def __init__(self, area_id, area_type, top_left, bottom_right, capacity=float('inf')):
        self.id = area_id
        self.type = area_type
        self.top_left = top_left
        self.bottom_right = bottom_right
        self.capacity = capacity
        self.materials = []
        self.busy_factor = 0  # Area congestion factor
        self.batches = {}  # Track materials by batch_id
        self.last_material_change = 0  # Track when materials last changed

    def is_full(self):
        return len(self.materials) >= self.capacity

    def is_empty(self):
        return len(self.materials) == 0

    def add_material(self, material):
        """Add material to area"""
        if not self.is_full():
            self.materials.append(material)
            # Assign random position within area
            margin = 5  # Edge margin
            x = random.uniform(self.top_left[0] + margin, self.bottom_right[0] - margin)
            y = random.uniform(self.top_left[1] + margin, self.bottom_right[1] - margin)
            material.location = (x, y)

            # Track batch
            if material.batch_id:
                if material.batch_id not in self.batches:
                    self.batches[material.batch_id] = []
                self.batches[material.batch_id].append(material)

            # Update area congestion factor
            self.busy_factor = len(self.materials) / self.capacity if self.capacity > 0 else 0
            return True
        return False

    def remove_material(self, material):
        """Remove material from area"""
        if material in self.materials:
            self.materials.remove(material)

            # Update batch tracking
            if material.batch_id and material.batch_id in self.batches:
                if material in self.batches[material.batch_id]:
                    self.batches[material.batch_id].remove(material)
                if not self.batches[material.batch_id]:
                    del self.batches[material.batch_id]

            # Update area congestion factor
            self.busy_factor = len(self.materials) / self.capacity if self.capacity > 0 else 0
            return True
        return False

    def get_material_by_type(self, material_type, count=1, batch_id=None):
        """Get materials of specified type"""
        if batch_id and batch_id in self.batches:
            # If batch_id specified, get materials from that batch
            available_materials = [m for m in self.batches[batch_id] if m.type == material_type]
        else:
            # Otherwise get any materials of the specified type
            available_materials = [m for m in self.materials if m.type == material_type]

        return available_materials[:count]

    def get_batch_materials(self, batch_id, count=None):
        """Get materials belonging to a specific batch"""
        if batch_id in self.batches:
            if count:
                return self.batches[batch_id][:count]
            return self.batches[batch_id][:]
        return []

    def get_center(self):
        """Return center coordinates of the area"""
        return ((self.top_left[0] + self.bottom_right[0]) / 2,
                (self.top_left[1] + self.bottom_right[1]) / 2)

    def update_materials_status(self, current_time, time_step):
        """Update status of all materials in the area"""
        for material in self.materials:
            material.update_waiting_time(time_step)

        # Update last material change time
        if self.materials:
            self.last_material_change = current_time

    def __repr__(self):
        batch_info = f", batches={len(self.batches)}" if self.batches else ""
        return f"Area({self.id}, {self.type}, materials={len(self.materials)}/{self.capacity}{batch_info})"


class Robot:
    def __init__(self, robot_id, position, speed=1.5, capacity=1):
        self.id = robot_id
        self.position = position
        self.speed = speed
        self.capacity = capacity
        self.carrying = []  # Materials being transported
        self.status = "idle"  # idle, moving, loading, unloading
        self.destination = None
        self.path = []
        self.current_task = None
        self.total_distance = 0
        self.completed_tasks = 0
        self.busy_time = 0
        self.idle_time = 0
        self.last_status_change = 0
        self.assigned_region = None  # Region robot is responsible for
        self.consecutive_failures = 0
        self.maintenance_time = 0
        self.last_move_time = 0
        self.monitoring_task = None  # Current monitoring task
        self.stuck_count = 0  # Count of times robot got stuck

    def is_available(self):
        """Check if robot is available for new tasks"""
        return (self.status == "idle" and
                len(self.carrying) < self.capacity and
                self.maintenance_time == 0)

    def is_available_for_monitoring(self):
        """Check if robot can perform monitoring tasks"""
        return (self.maintenance_time == 0 and
                self.monitoring_task is None and
                self.status != "loading" and self.status != "unloading")

    def move_to(self, destination, current_time, grid_map):
        """Move to destination"""
        if self.maintenance_time > 0:
            return False

        # Use direct path for close destinations
        dx = destination[0] - self.position[0]
        dy = destination[1] - self.position[1]
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 50:  # Small distance, use direct path
            self.destination = destination
            self.status = "moving"
            self.last_status_change = current_time
            self.last_move_time = current_time
            self.path = [destination]
            return True

        self.destination = destination
        self.status = "moving"
        self.last_status_change = current_time
        self.last_move_time = current_time

        # Check path cache
        path_key = (self.position, destination)
        if grid_map[2] and path_key in grid_map[2]:
            self.path = grid_map[2][path_key].copy()
            self.consecutive_failures = 0
            return True

        # Calculate path using A*
        self.path = astar_path(grid_map, self.position, destination, path_cache=grid_map[2])

        if self.path:
            self.consecutive_failures = 0
            return True
        else:
            # Create direct path if planning fails
            logger.warning(f"Robot {self.id} couldn't find path, using direct path to {destination}")
            num_segments = 10
            self.path = [(self.position[0] + (destination[0] - self.position[0]) * i / num_segments,
                          self.position[1] + (destination[1] - self.position[1]) * i / num_segments)
                         for i in range(1, num_segments + 1)]
            return len(self.path) > 0

    def update_position(self, current_time, time_step=1):
        """Update robot position"""
        if self.maintenance_time > 0:
            # Reduce maintenance time
            actual_maintenance = min(time_step, self.maintenance_time)
            self.maintenance_time -= actual_maintenance
            if self.maintenance_time == 0:
                logger.info(f"Robot {self.id} completed maintenance")
            return False

        if self.status == "moving" and len(self.path) > 0:
            # Update last move time
            self.last_move_time = current_time

            # Calculate distance movable in this time step
            distance_per_step = self.speed * time_step
            total_distance_moved = 0

            while self.path and total_distance_moved < distance_per_step:
                next_pos = self.path[0]
                # Calculate distance to next point
                dx = next_pos[0] - self.position[0]
                dy = next_pos[1] - self.position[1]
                distance_to_next = math.sqrt(dx * dx + dy * dy)

                if distance_to_next <= (distance_per_step - total_distance_moved):
                    # Can reach next point
                    self.position = next_pos
                    self.path.pop(0)
                    total_distance_moved += distance_to_next
                    self.total_distance += distance_to_next
                else:
                    # Only partial movement
                    ratio = (distance_per_step - total_distance_moved) / distance_to_next
                    self.position = (
                        self.position[0] + dx * ratio,
                        self.position[1] + dy * ratio
                    )
                    self.total_distance += (distance_per_step - total_distance_moved)
                    total_distance_moved = distance_per_step

            # Check if destination reached
            if not self.path:
                self.status = "idle"
                self.busy_time += (current_time - self.last_status_change)
                self.last_status_change = current_time
                return True
        elif self.status == "idle":
            self.idle_time += time_step

        return False

    def pick_material(self, material, current_time):
        """Pick up material"""
        if self.maintenance_time > 0:
            return False

        if len(self.carrying) < self.capacity:
            self.carrying.append(material)
            material.status = "in_transit"
            self.status = "loading"
            self.busy_time += (current_time - self.last_status_change)
            self.last_status_change = current_time
            logger.debug(f"Robot {self.id} picked up material {material.id} ({material.type})")
            return True
        return False

    def drop_material(self, area, current_time):
        """Drop material"""
        if self.maintenance_time > 0:
            return False

        if self.carrying and not area.is_full():
            success = True
            for material in self.carrying:
                if not area.add_material(material):
                    success = False
                    break

            if success:
                logger.debug(f"Robot {self.id} dropped {len(self.carrying)} materials in area {area.id}")
                self.completed_tasks += len(self.carrying)
                self.carrying = []
                self.status = "idle"
                self.busy_time += (current_time - self.last_status_change)
                self.last_status_change = current_time
                return True
            else:
                # Drop failed, area full
                logger.warning(f"Robot {self.id} couldn't drop materials, area {area.id} is full")
                self.consecutive_failures += 1
                return False
        return False

    def start_monitoring(self, machine, current_time):
        """Start monitoring machine"""
        if self.is_available_for_monitoring():
            self.monitoring_task = {
                'type': 'monitor',
                'target': machine,
                'start_time': current_time
            }
            logger.debug(f"Robot {self.id} started monitoring machine {machine.id} ({machine.type})")
            return True
        return False

    def stop_monitoring(self):
        """Stop monitoring"""
        if self.monitoring_task:
            logger.debug(f"Robot {self.id} stopped monitoring machine {self.monitoring_task['target'].id}")
            self.monitoring_task = None
            return True
        return False

    def require_maintenance(self, duration=300):
        """Robot needs maintenance"""
        if self.status == "idle" or self.consecutive_failures > 5 or self.stuck_count > 3:
            prev_status = self.status
            self.status = "idle"
            self.maintenance_time = duration
            self.consecutive_failures = 0
            self.stuck_count = 0

            # Return materials being transported
            if self.carrying and prev_status != "idle":
                self.carrying = []

            # Stop any monitoring tasks
            if self.monitoring_task:
                self.stop_monitoring()

            logger.info(f"Robot {self.id} entering maintenance mode for {duration} seconds")
            return True
        return False

    def is_stuck(self, current_time, threshold=100):
        """Check if robot is stuck"""
        if self.status == "moving" and len(self.path) > 0:
            if current_time - self.last_move_time > threshold:
                self.stuck_count += 1
                return True
        return False

    def __repr__(self):
        status_str = self.status
        if self.maintenance_time > 0:
            status_str += f"(maintenance:{self.maintenance_time}s)"
        monitoring_str = f", monitoring:{self.monitoring_task['target'].id}" if self.monitoring_task else ""
        carrying_info = f", carrying={len(self.carrying)}"
        return f"Robot({self.id}, at {self.position}, {carrying_info}, status={status_str}{monitoring_str})"


class Task:
    def __init__(self, task_id, task_type, priority, source, destination, materials, batch_id=None):
        self.id = task_id
        self.type = task_type  # "transport", "process", "maintenance", "monitor"
        self.priority = priority  # Base priority
        self.source = source
        self.destination = destination
        self.materials = materials
        self.status = "pending"  # pending, assigned, in_progress, completed, failed
        self.assigned_robot = None
        self.creation_time = 0
        self.assignment_time = 0
        self.start_time = 0
        self.completion_time = 0
        self.attempt_count = 0
        self.timeout = None  # Task timeout
        self.dynamic_priority = priority  # Dynamic priority, changes over time
        self.category = self._determine_category()  # Task category for statistics
        self.batch_id = batch_id  # Batch ID for tracking workflow
        self.source_id = None  # For tracking when source is a machine
        self.is_force_task = False  # Flag for forced tasks

        # Store source ID for later checking
        if hasattr(source, 'id'):
            self.source_id = source.id

        # Track ideal time for this task (for efficiency calculation)
        self.ideal_time = self._calculate_ideal_time()

    def _calculate_ideal_time(self):
        """Calculate ideal time for this task execution (without travel)"""
        if self.type == "transport":
            return 60  # Base transport time
        elif self.type == "process":
            if isinstance(self.destination, Machine):
                return self.destination.processing_time
        return 30  # Default time for other tasks

    def _determine_category(self):
        """Determine task category"""
        if self.type == "transport":
            if isinstance(self.source, Machine) and isinstance(self.destination, Area):
                return "machine_to_area"
            elif isinstance(self.source, Area) and isinstance(self.destination, Machine):
                return "area_to_machine"
            elif isinstance(self.source, Area) and isinstance(self.destination, Area):
                return "area_to_area"
            else:
                return "other_transport"
        return self.type

    def update_priority(self, current_time):
        """Update task dynamic priority"""
        # Force tasks always have highest priority
        if self.is_force_task:
            self.dynamic_priority = 100
            return

        # Base priority
        self.dynamic_priority = self.priority

        # Waiting time increases priority
        waiting_time = current_time - self.creation_time
        time_factor = min(5, waiting_time / 1000)  # Control maximum boost

        # Material boost
        material_boost = 0
        for material in self.materials:
            material_boost = max(material_boost, material.task_priority_boost)

        # Attempt penalty
        attempt_penalty = self.attempt_count * 0.5

        # Calculate final dynamic priority
        self.dynamic_priority += time_factor + material_boost - attempt_penalty

        # Special adjustments
        if self.type == "transport" and isinstance(self.source, Machine) and self.source.status == "waiting_for_pickup":
            self.dynamic_priority += 10  # High priority for machine pickup

        # Batch progression priority
        if self.batch_id:
            if any(m.type == "red" for m in self.materials):
                self.dynamic_priority += 15  # Highest priority for red buckets
            elif any(m.type == "yellow" for m in self.materials):
                self.dynamic_priority += 8  # High priority for yellow buckets
            elif any(m.type == "green" for m in self.materials):
                self.dynamic_priority += 3  # Increased priority for green buckets

        # Give higher priority to tasks with fewer materials needed
        # (helps prevent resource starvation)
        if self.materials and len(self.materials) <= 2:
            self.dynamic_priority += 2

    def is_expired(self, current_time):
        """Check if task is expired"""
        if self.timeout and current_time > self.timeout:
            return True
        return False

    def __lt__(self, other):
        return self.dynamic_priority > other.dynamic_priority  # Higher priority first

    def __repr__(self):
        batch_info = f", batch={self.batch_id}" if self.batch_id else ""
        force_info = " [FORCE]" if self.is_force_task else ""
        return f"Task({self.id}, {self.type}, priority={self.priority:.1f}→{self.dynamic_priority:.1f}, status={self.status}{batch_info}{force_info})"


def create_grid_map(width, height, areas, machines, resolution=10.0):
    """Create grid map for path planning"""
    # Use lower resolution for performance
    scaled_width = int(width / resolution)
    scaled_height = int(height / resolution)
    grid = np.ones((scaled_width, scaled_height), dtype=bool)  # True = passable

    # Mark areas as passable
    for area in areas:
        x_min, y_min = int(area.top_left[0] / resolution), int(area.top_left[1] / resolution)
        x_max, y_max = int(area.bottom_right[0] / resolution), int(area.bottom_right[1] / resolution)
        x_min = max(0, x_min)
        y_min = max(0, y_min)
        x_max = min(scaled_width - 1, x_max)
        y_max = min(scaled_height - 1, y_max)
        grid[x_min:x_max + 1, y_min:y_max + 1] = True

    # Mark machines as obstacles
    for machine in machines:
        x, y = int(machine.position[0] / resolution), int(machine.position[1] / resolution)
        radius = 1  # Smaller radius for better passability
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < scaled_width and 0 <= ny < scaled_height:
                    grid[nx, ny] = False

    # Add border obstacles
    border_width = 1
    grid[:border_width, :] = False
    grid[-border_width:, :] = False
    grid[:, :border_width] = False
    grid[:, -border_width:] = False

    # Ensure connectivity between areas - create corridors
    corridor_width = 8
    for y in range(0, scaled_height, corridor_width * 2):
        if y + corridor_width < scaled_height:
            grid[:, y:y + corridor_width] = True

    for x in range(0, scaled_width, corridor_width * 2):
        if x + corridor_width < scaled_width:
            grid[x:x + corridor_width, :] = True

    # Create path cache
    path_cache = {}

    return grid, resolution, path_cache


def heuristic(a, b):
    """A* heuristic function: Euclidean distance"""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def generate_direct_path(start, goal, num_points=5):
    """Generate direct path between points"""
    path = []
    for i in range(num_points + 1):
        t = i / num_points
        x = start[0] + t * (goal[0] - start[0])
        y = start[1] + t * (goal[1] - start[1])
        path.append((x, y))
    return path


def astar_path(grid, start, goal, max_iterations=1000, path_cache=None):
    """Use A* algorithm to find shortest path"""
    # Check cache
    cache_key = (start, goal)
    if path_cache is not None and cache_key in path_cache:
        return path_cache[cache_key].copy()

    # Direct path for close destinations
    dist = ((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2) ** 0.5
    if dist < 100:  # Short distance, use direct path
        direct_path = [goal]
        if path_cache is not None:
            path_cache[cache_key] = direct_path.copy()
        return direct_path

    grid_map, resolution = grid[:2]  # Unpack first two elements

    # Discretize coordinates
    start_scaled = (int(start[0] / resolution), int(start[1] / resolution))
    goal_scaled = (int(goal[0] / resolution), int(goal[1] / resolution))

    # A* algorithm
    closed_set = set()
    open_set = {start_scaled}
    came_from = {}

    g_score = {start_scaled: 0}
    f_score = {start_scaled: heuristic(start_scaled, goal_scaled)}

    open_heap = [(f_score[start_scaled], start_scaled)]
    heapq.heapify(open_heap)

    # Movement directions (4-way)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    iterations = 0
    while open_set and iterations < max_iterations:
        iterations += 1
        current = heapq.heappop(open_heap)[1]

        if current == goal_scaled:
            # Rebuild path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()

            # Smooth path - keep only key points
            if len(path) > 2:
                path = [path[0]] + [path[len(path) // 2]] + [path[-1]]

            # Convert back to float coordinates
            result_path = [(float(x * resolution), float(y * resolution)) for x, y in path]
            result_path[0] = start  # Replace with actual start
            result_path[-1] = goal  # Replace with actual goal

            # Cache result
            if path_cache is not None:
                path_cache[cache_key] = result_path.copy()

            return result_path

        open_set.remove(current)
        closed_set.add(current)

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if (neighbor[0] < 0 or neighbor[0] >= grid_map.shape[0] or
                    neighbor[1] < 0 or neighbor[1] >= grid_map.shape[1]):
                continue

            if not grid_map[neighbor] or neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + 1.0

            if neighbor not in open_set or tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal_scaled)

                if neighbor not in open_set:
                    open_set.add(neighbor)
                    heapq.heappush(open_heap, (f_score[neighbor], neighbor))

    # If A* fails, use direct path
    logger.warning(f"No path found from {start} to {goal}, using direct path")
    direct_path = generate_direct_path(start, goal, 5)

    # Cache result
    if path_cache is not None:
        path_cache[cache_key] = direct_path.copy()

    return direct_path


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
        self.task_queue = []  # Priority queue
        self.grid_map = None
        self.deadlock_count = 0
        self.last_progress_time = 0
        self.deadlock_threshold = 1000
        self.completed_task_ids = set()
        self.failed_task_ids = set()
        self.last_task_generation = 0
        self.task_generation_interval = 5
        self.next_batch_id = 1
        self.last_progress_check = 0
        self.progress_check_interval = 500

        # Track task creation to prevent duplicates
        self.active_tasks = set()  # Set of (source_id, destination_id, task_type) tuples
        self.max_task_queue_size = 200  # Max task queue size

        # Monitor task related
        self.monitor_tasks = []
        self.last_monitor_task_generation = 0
        self.monitor_task_interval = 100

        # Production tracking
        self.target_red_buckets = 8
        self.red_buckets_in_completed_area = 0
        self.production_plan = {
            'empty': 0, 'green': 0, 'yellow': 0, 'red': 0
        }
        self.batch_tasks = {}
        self.batch_status = {}
        self.last_red_bucket_production = 0

        # Production flow tracking
        self.flow_trackers = {
            'empty_to_green': 0,
            'green_to_yellow': 0,
            'yellow_to_red': 0
        }

        # Efficiency calculation
        self.total_ideal_time = 0
        self.total_actual_time = 0
        self.completed_tasks_ideal_time = 0

        # Statistics
        self.stats = {
            'red_buckets_produced': 0,
            'green_buckets_produced': 0,
            'yellow_buckets_produced': 0,
            'lead_times': {
                'green': [], 'yellow': [], 'red': []
            },
            'robot_distances': {},
            'robot_tasks': {},
            'machine_utilization': {},
            'material_states': [],
            'task_stats': {
                'created': 0, 'completed': 0, 'failed': 0,
                'by_category': defaultdict(int),
                'by_batch': defaultdict(int)
            },
            'deadlocks': [],
            'forced_actions': 0
        }

        # Visualization data
        self.vis_data = {
            'time': [],
            'red_count': [],
            'robot_positions': defaultdict(list),
            'material_positions': defaultdict(list),
            'task_queue_length': []
        }

        # Flag to indicate if major reset is needed
        self.major_reset_needed = False
        self.last_major_reset = 0

        # Initialize factory layout
        self.setup_factory()

    def setup_factory(self):
        """Initialize factory layout"""
        # Create areas
        # Empty bucket storage
        self.areas.append(Area(0, "empty_storage", (50, 50), (150, 950), 300))

        # Carding areas
        for i in range(4):
            y_pos = 100 + i * 200
            self.areas.append(Area(1 + i * 2, "carding_waiting", (250, y_pos), (300, y_pos + 100), 5))
            self.areas.append(Area(2 + i * 2, "carding_finished", (350, y_pos), (400, y_pos + 100), 20))

        # Drawing1 areas
        for i in range(2):
            y_pos = 150 + i * 300
            self.areas.append(Area(9 + i * 2, "drawing1_waiting", (500, y_pos), (550, y_pos + 100), 24))
            self.areas.append(Area(10 + i * 2, "drawing1_finished", (600, y_pos), (650, y_pos + 100), 8))

        # Drawing2 areas
        for i in range(2):
            y_pos = 150 + i * 300
            self.areas.append(Area(13 + i * 2, "drawing2_waiting", (700, y_pos), (750, y_pos + 100), 24))
            self.areas.append(Area(14 + i * 2, "drawing2_finished", (800, y_pos), (850, y_pos + 100), 8))

        # Roving areas
        for i in range(2):
            y_pos = 150 + i * 300
            self.areas.append(Area(17 + i * 2, "roving_waiting", (900, y_pos), (950, y_pos + 100), 24))
            self.areas.append(Area(18 + i * 2, "roving_finished", (950, y_pos), (980, y_pos + 100), 8))

        # Completed products area
        self.areas.append(Area(21, "completed_products", (900, 700), (980, 950), 50))

        # Create machines
        # Carding machines
        for i in range(4):
            y_pos = 150 + i * 200
            self.machines.append(Machine(i, "carding", (325, y_pos), 4000))

        # Drawing1 machines
        for i in range(2):
            y_pos = 200 + i * 300
            self.machines.append(Machine(4 + i, "drawing1", (575, y_pos), 2500, 6))

        # Drawing2 machines
        for i in range(2):
            y_pos = 200 + i * 300
            self.machines.append(Machine(6 + i, "drawing2", (775, y_pos), 2500, 6))

        # Roving machines
        for i in range(2):
            y_pos = 200 + i * 300
            self.machines.append(Machine(8 + i, "roving", (925, y_pos), 2500, 6))

        # Create robots
        robot_positions = [
            (100, 100), (100, 300), (100, 500), (100, 700),  # Near empty storage
            (300, 200), (300, 600),  # Near carding
            (500, 200), (700, 200), (900, 200), (900, 600)  # Near other areas
        ]

        for i, pos in enumerate(robot_positions):
            self.robots.append(Robot(i, pos, speed=3.0))
            self.stats['robot_distances'][i] = 0
            self.stats['robot_tasks'][i] = 0

        # Assign regions to robots
        for i in range(4):
            self.robots[i].assigned_region = "empty_storage"
        self.robots[4].assigned_region = "carding"
        self.robots[5].assigned_region = "carding"
        self.robots[6].assigned_region = "drawing1"
        self.robots[7].assigned_region = "drawing2"
        self.robots[8].assigned_region = "roving"
        self.robots[9].assigned_region = "completed_products"

        # Create grid map for path planning
        self.grid_map = create_grid_map(self.width, self.height, self.areas, self.machines)

        logger.info("Factory layout initialized")

    def generate_initial_materials(self, target_red_buckets=8):
        """Generate initial empty buckets based on production targets"""
        self.target_red_buckets = target_red_buckets

        # Calculate required materials
        empty_buckets_needed = target_red_buckets * 6 * 6  # Each red needs 6 yellows, each yellow needs 6 greens
        green_buckets_needed = target_red_buckets * 6
        yellow_buckets_needed = target_red_buckets * 6

        # Update production plan
        self.production_plan = {
            'empty': empty_buckets_needed,
            'green': green_buckets_needed,
            'yellow': yellow_buckets_needed,
            'red': target_red_buckets
        }

        logger.info(f"Production plan set: {self.production_plan['red']} red buckets requiring "
                    f"{self.production_plan['yellow']} yellows, {self.production_plan['green']} greens, "
                    f"{self.production_plan['empty']} empty buckets")

        # Create empty buckets with batch associations
        for i in range(empty_buckets_needed):
            material = Material(i, "empty")
            material.creation_time = 0

            # Assign batch IDs to track material flow
            batch_id = (i // 6) + 1  # Batches start at 1
            material.batch_id = batch_id

            self.materials.append(material)
            self.areas[0].add_material(material)

            # Initialize batch status tracking
            if batch_id not in self.batch_status:
                self.batch_status[batch_id] = {
                    'empty_created': 0,
                    'green_created': 0,
                    'yellow_created': 0,
                    'red_created': 0,
                    'completed': False
                }
            self.batch_status[batch_id]['empty_created'] += 1

        self.next_batch_id = (empty_buckets_needed // 6) + 1
        logger.info(f"Created {empty_buckets_needed} empty buckets in {self.next_batch_id - 1} batches")

    def _find_corresponding_finished_area(self, machine):
        """Find corresponding finished area for a machine"""
        if machine.type == "carding":
            machine_idx = machine.id
            return self.areas[2 + machine_idx * 2]
        elif machine.type == "drawing1":
            machine_idx = machine.id - 4
            return self.areas[10 + machine_idx * 2]
        elif machine.type == "drawing2":
            machine_idx = machine.id - 6
            return self.areas[14 + machine_idx * 2]
        elif machine.type == "roving":
            machine_idx = machine.id - 8
            return self.areas[18 + machine_idx * 2]
        return None

    def _find_corresponding_waiting_area(self, machine):
        """Find corresponding waiting area for a machine"""
        if machine.type == "carding":
            machine_idx = machine.id
            return self.areas[1 + machine_idx * 2]
        elif machine.type == "drawing1":
            machine_idx = machine.id - 4
            return self.areas[9 + machine_idx * 2]
        elif machine.type == "drawing2":
            machine_idx = machine.id - 6
            return self.areas[13 + machine_idx * 2]
        elif machine.type == "roving":
            machine_idx = machine.id - 8
            return self.areas[17 + machine_idx * 2]
        return None

    def task_queue_too_large(self):
        """Check if task queue is too large"""
        return len(self.task_queue) > self.max_task_queue_size

    def check_if_task_exists(self, task_type, source, destination, materials):
        """Check if similar task already exists"""
        source_id = getattr(source, 'id', None)
        dest_id = getattr(destination, 'id', None)

        # Special handling for machine pickup tasks
        if task_type == "transport" and isinstance(source, Machine) and source.status == "waiting_for_pickup":
            return source.has_pickup_task  # Use the flag to check if pickup task exists

        # For other tasks, check the active_tasks set
        task_key = (task_type, source_id, dest_id)
        return task_key in self.active_tasks

    def mark_task_active(self, task):
        """Mark a task as active to prevent duplicates"""
        source_id = getattr(task.source, 'id', None)
        dest_id = getattr(task.destination, 'id', None)

        # For machine pickup tasks, set the machine flag
        if task.type == "transport" and isinstance(task.source, Machine) and task.source.status == "waiting_for_pickup":
            task.source.has_pickup_task = True

        # Add to active tasks set
        task_key = (task.type, source_id, dest_id)
        self.active_tasks.add(task_key)

    def remove_task_active(self, task):
        """Remove a task from active tasks when completed or failed"""
        source_id = getattr(task.source, 'id', None)
        dest_id = getattr(task.destination, 'id', None)

        # For machine pickup tasks, clear the machine flag
        if task.type == "transport" and isinstance(task.source, Machine):
            task.source.has_pickup_task = False

        # Remove from active tasks set
        task_key = (task.type, source_id, dest_id)
        if task_key in self.active_tasks:
            self.active_tasks.remove(task_key)

    def generate_tasks(self):
        """Generate tasks based on production flow"""
        # If task queue is too large, clean up but don't skip generation
        if self.task_queue_too_large():
            logger.warning(f"Task queue large ({len(self.task_queue)} tasks), cleaning up")
            # Force clean up old tasks
            self._cleanup_task_queue()
            # Don't return, continue creating critical tasks

        # Get material counts by area and type for monitoring
        type_counts = {}
        for area in self.areas:
            type_counts[area.id] = {
                'empty': sum(1 for m in area.materials if m.type == "empty"),
                'green': sum(1 for m in area.materials if m.type == "green"),
                'yellow': sum(1 for m in area.materials if m.type == "yellow"),
                'red': sum(1 for m in area.materials if m.type == "red")
            }

        # Log material distribution periodically for diagnosis
        if self.current_time % 10000 == 0:
            logger.info("=== Material Distribution ===")
            for area in self.areas:
                if sum(type_counts[area.id].values()) > 0:
                    logger.info(f"Area {area.id} ({area.type}): {type_counts[area.id]}")
            for machine in self.machines:
                if machine.materials:
                    material_types = [m.type for m in machine.materials]
                    logger.info(f"Machine {machine.id} ({machine.type}): {material_types}, status={machine.status}")

        new_tasks = 0

        # 1. Handle machines waiting for pickup (highest priority)
        for machine in self.machines:
            if (machine.status == "waiting_for_pickup" and
                    machine.materials and
                    not machine.has_pickup_task and  # Check if a pickup task already exists
                    self.current_time - machine.last_emergency_task > 500):  # Throttle emergency tasks

                finished_area = self._find_corresponding_finished_area(machine)

                if finished_area and not finished_area.is_full():
                    task_id = len(self.tasks)
                    batch_id = machine.current_batch_id

                    # High priority pickup task
                    task = Task(task_id, "transport", 30, machine, finished_area, machine.materials[:],
                                batch_id=batch_id)
                    task.creation_time = self.current_time
                    task.timeout = self.current_time + 2000
                    self.tasks.append(task)
                    self.stats['task_stats']['created'] += 1
                    self.stats['task_stats']['by_category'][task.category] += 1
                    self.total_ideal_time += task.ideal_time

                    # Mark that this machine has a pickup task
                    machine.has_pickup_task = True
                    machine.last_emergency_task = self.current_time
                    self.mark_task_active(task)

                    if batch_id:
                        self.stats['task_stats']['by_batch'][batch_id] += 1
                        if batch_id not in self.batch_tasks:
                            self.batch_tasks[batch_id] = []
                        self.batch_tasks[batch_id].append(task)

                    task.update_priority(self.current_time)
                    heapq.heappush(self.task_queue, task)
                    logger.info(f"Created urgent pickup task: from machine {machine.id} to {finished_area.id}")
                    new_tasks += 1

        # 1.1 Special force progress tasks when no red buckets have been produced
        # Only execute when there are no red buckets in completed area and system has run for a while
        if self.red_buckets_in_completed_area == 0 and self.current_time > 30000:
            # Check if there are any red buckets that need transport
            red_buckets_anywhere = sum(1 for m in self.materials if m.type == "red")
            if red_buckets_anywhere > 0:
                logger.info(f"Found {red_buckets_anywhere} red buckets in system, forcing transport!")

                # Find red buckets in roving_finished areas and force transport tasks
                for area in self.areas:
                    if area.type == "roving_finished":
                        red_materials = [m for m in area.materials if m.type == "red"]
                        if red_materials:
                            completed_area = self.areas[21]  # Completed area
                            task_id = len(self.tasks)
                            task = Task(task_id, "transport", 50, area, completed_area, red_materials[:1],
                                        batch_id=red_materials[0].batch_id)
                            task.creation_time = self.current_time
                            task.timeout = self.current_time + 3000
                            task.is_force_task = True
                            self.tasks.append(task)
                            self.stats['task_stats']['created'] += 1
                            self.total_ideal_time += task.ideal_time
                            self.stats['forced_actions'] += 1

                            task.update_priority(self.current_time)
                            heapq.heappush(self.task_queue, task)
                            logger.info(f"Created FORCE transport task: red bucket to completed area")
                            new_tasks += 1

            # If no red buckets, check if yellow buckets need processing
            if red_buckets_anywhere == 0:
                yellow_ready_for_roving = False
                for area in self.areas:
                    if area.type == "roving_waiting":
                        yellow_count = sum(1 for m in area.materials if m.type == "yellow")
                        if yellow_count >= 6:
                            yellow_ready_for_roving = True
                            logger.info(f"Found {yellow_count} yellow buckets waiting for roving, forcing process!")

                            # Find idle roving machine
                            for machine in self.machines:
                                if machine.type == "roving" and machine.status == "idle" and not machine.materials:
                                    # Create force processing task
                                    yellow_materials = area.get_material_by_type("yellow", 6)
                                    task_id = len(self.tasks)
                                    task = Task(task_id, "process", 40, area, machine, yellow_materials,
                                                batch_id=yellow_materials[0].batch_id)
                                    task.creation_time = self.current_time
                                    task.is_force_task = True
                                    self.tasks.append(task)
                                    self.stats['task_stats']['created'] += 1
                                    self.total_ideal_time += task.ideal_time
                                    self.stats['forced_actions'] += 1

                                    task.update_priority(self.current_time)
                                    heapq.heappush(self.task_queue, task)
                                    logger.info(
                                        f"Created FORCE process task: 6 yellow → red in roving machine {machine.id}")
                                    new_tasks += 1
                                    break
                            break

                # If no suitable yellow buckets for red production, force move yellows to roving_waiting
                if not yellow_ready_for_roving:
                    # Check if there are yellows in any drawing finished areas
                    for area in self.areas:
                        if area.type in ["drawing1_finished", "drawing2_finished"]:
                            yellow_materials = [m for m in area.materials if m.type == "yellow"]
                            if yellow_materials:
                                # Find roving waiting area with space
                                for dest_area in self.areas:
                                    if dest_area.type == "roving_waiting" and not dest_area.is_full():
                                        task_id = len(self.tasks)
                                        task = Task(task_id, "transport", 35, area, dest_area, yellow_materials[:1],
                                                    batch_id=yellow_materials[0].batch_id)
                                        task.creation_time = self.current_time
                                        task.is_force_task = True
                                        self.tasks.append(task)
                                        self.stats['task_stats']['created'] += 1
                                        self.total_ideal_time += task.ideal_time
                                        self.stats['forced_actions'] += 1

                                        task.update_priority(self.current_time)
                                        heapq.heappush(self.task_queue, task)
                                        logger.info(f"Created FORCE yellow transport task: {area.id} to {dest_area.id}")
                                        new_tasks += 1
                                        break
                                break

        # 2. Process tasks
        # 2.1 Carding machines: empty → green
        for machine in self.machines:
            if machine.type == "carding" and machine.status == "idle" and not machine.materials:
                waiting_area = self._find_corresponding_waiting_area(machine)
                finished_area = self._find_corresponding_finished_area(machine)

                if not waiting_area.is_empty() and not finished_area.is_full():
                    # Skip if a similar task already exists
                    if self.check_if_task_exists("process", waiting_area, machine, None):
                        continue

                    # Get materials by batch if possible
                    empty_materials = []
                    for batch_id in sorted(self.batch_status.keys()):
                        batch_materials = waiting_area.get_batch_materials(batch_id)
                        if batch_materials:
                            empty_materials = [batch_materials[0]]
                            break

                    # If no batch materials found, take any empty bucket
                    if not empty_materials:
                        empty_materials = waiting_area.get_material_by_type("empty", 1)

                    if empty_materials:
                        batch_id = empty_materials[0].batch_id
                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 15, waiting_area, machine, empty_materials, batch_id=batch_id)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 5000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        self.total_ideal_time += task.ideal_time
                        self.mark_task_active(task)

                        if batch_id:
                            self.stats['task_stats']['by_batch'][batch_id] += 1
                            if batch_id not in self.batch_tasks:
                                self.batch_tasks[batch_id] = []
                            self.batch_tasks[batch_id].append(task)

                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.debug(f"Created process task: empty → green in carding machine {machine.id}")
                        new_tasks += 1

        # 2.2 Drawing1 machines: 6 green → 1 yellow
        for machine in self.machines:
            if machine.type == "drawing1" and machine.status == "idle" and not machine.materials:
                waiting_area = self._find_corresponding_waiting_area(machine)
                finished_area = self._find_corresponding_finished_area(machine)

                if not finished_area.is_full():
                    # Skip if a similar task already exists
                    if self.check_if_task_exists("process", waiting_area, machine, None):
                        continue

                    # Try to find 6 green buckets of the same batch
                    batch_materials = []
                    for batch_id in sorted(self.batch_status.keys()):
                        green_materials = waiting_area.get_batch_materials(batch_id)
                        green_materials = [m for m in green_materials if m.type == "green"]
                        if len(green_materials) >= 6:
                            batch_materials = green_materials[:6]
                            break

                    # If no complete batch found, try any 6 green buckets
                    if len(batch_materials) < 6:
                        green_materials = waiting_area.get_material_by_type("green", 6)
                        if len(green_materials) == 6:
                            batch_ids = set(m.batch_id for m in green_materials if m.batch_id)
                            if len(batch_ids) > 1:
                                # Create new batch for mixed materials
                                batch_id = self.next_batch_id
                                self.next_batch_id += 1
                                for material in green_materials:
                                    material.batch_id = batch_id

                                # Initialize batch status
                                self.batch_status[batch_id] = {
                                    'empty_created': 0,
                                    'green_created': 6,
                                    'yellow_created': 0,
                                    'red_created': 0,
                                    'completed': False
                                }
                            else:
                                # Use existing batch ID
                                batch_id = next(iter(batch_ids)) if batch_ids else None

                            batch_materials = green_materials

                    if len(batch_materials) == 6:
                        batch_id = batch_materials[0].batch_id
                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 16, waiting_area, machine, batch_materials, batch_id=batch_id)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 5000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        self.total_ideal_time += task.ideal_time
                        self.mark_task_active(task)

                        if batch_id:
                            self.stats['task_stats']['by_batch'][batch_id] += 1
                            if batch_id not in self.batch_tasks:
                                self.batch_tasks[batch_id] = []
                            self.batch_tasks[batch_id].append(task)

                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.info(f"Created process task: 6 green → yellow in drawing1 machine {machine.id}")
                        new_tasks += 1

        # 2.3 Roving machines: 6 yellow → 1 red (higher priority)
        for machine in self.machines:
            if machine.type == "roving" and machine.status == "idle" and not machine.materials:
                waiting_area = self._find_corresponding_waiting_area(machine)
                finished_area = self._find_corresponding_finished_area(machine)

                if not finished_area.is_full():
                    # Skip if a similar task already exists
                    if self.check_if_task_exists("process", waiting_area, machine, None):
                        continue

                    # Try to find 6 yellow buckets
                    yellow_materials = waiting_area.get_material_by_type("yellow", 6)

                    if len(yellow_materials) >= 6:
                        batch_ids = set(m.batch_id for m in yellow_materials if m.batch_id)
                        batch_id = next(iter(batch_ids)) if batch_ids else self.next_batch_id

                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 25, waiting_area, machine, yellow_materials[:6],
                                    batch_id=batch_id)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 4000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        self.total_ideal_time += task.ideal_time
                        self.mark_task_active(task)

                        if batch_id:
                            self.stats['task_stats']['by_batch'][batch_id] += 1
                            if batch_id not in self.batch_tasks:
                                self.batch_tasks[batch_id] = []
                            self.batch_tasks[batch_id].append(task)

                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.info(f"Created process task: 6 yellow → red in roving machine {machine.id}")
                        new_tasks += 1

        # 3. Transport tasks
        # 3.1 Empty storage → Carding waiting areas
        for area in self.areas:
            if area.type == "carding_waiting" and len(area.materials) < 2:
                empty_storage = self.areas[0]
                if not empty_storage.is_empty():
                    # Skip if a similar task already exists
                    if self.check_if_task_exists("transport", empty_storage, area, None):
                        continue

                    # Get empty buckets from batch
                    empty_buckets = None
                    for batch_id in sorted(self.batch_status.keys()):
                        batch_materials = empty_storage.get_batch_materials(batch_id)
                        if batch_materials:
                            empty_buckets = batch_materials[:min(2, len(batch_materials))]
                            break

                    if not empty_buckets:
                        empty_buckets = empty_storage.get_material_by_type("empty", 2)

                    if empty_buckets:
                        batch_id = empty_buckets[0].batch_id
                        task_id = len(self.tasks)
                        task = Task(task_id, "transport", 12, empty_storage, area, empty_buckets, batch_id=batch_id)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 3000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        self.total_ideal_time += task.ideal_time
                        self.mark_task_active(task)

                        if batch_id:
                            self.stats['task_stats']['by_batch'][batch_id] += 1
                            if batch_id not in self.batch_tasks:
                                self.batch_tasks[batch_id] = []
                            self.batch_tasks[batch_id].append(task)

                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.debug(f"Created transport task: {len(empty_buckets)} empty buckets to area {area.id}")
                        new_tasks += 1

        # 3.2 Transport green buckets to drawing1_waiting areas
        for source_area in self.areas:
            if source_area.type == "carding_finished" and not source_area.is_empty():
                green_buckets = [m for m in source_area.materials if m.type == "green"]
                if green_buckets:
                    # Find drawing1_waiting areas with space
                    for dest_area in self.areas:
                        if dest_area.type == "drawing1_waiting" and not dest_area.is_full():
                            # Skip if a similar task already exists
                            if self.check_if_task_exists("transport", source_area, dest_area, None):
                                continue

                            count = min(len(green_buckets), 6, dest_area.capacity - len(dest_area.materials))
                            if count > 0:
                                batch_id = green_buckets[0].batch_id
                                task_id = len(self.tasks)
                                task = Task(task_id, "transport", 14, source_area, dest_area, green_buckets[:count],
                                            batch_id=batch_id)
                                task.creation_time = self.current_time
                                task.timeout = self.current_time + 3000
                                self.tasks.append(task)
                                self.stats['task_stats']['created'] += 1
                                self.stats['task_stats']['by_category'][task.category] += 1
                                self.total_ideal_time += task.ideal_time
                                self.mark_task_active(task)

                                task.update_priority(self.current_time)
                                heapq.heappush(self.task_queue, task)
                                logger.debug(
                                    f"Created transport task: {count} green buckets from {source_area.id} to {dest_area.id}")
                                new_tasks += 1
                                break

        # 3.3 Transport yellow buckets to drawing2_waiting or roving_waiting areas
        for source_area in self.areas:
            if source_area.type in ["drawing1_finished", "drawing2_finished"] and not source_area.is_empty():
                yellow_buckets = [m for m in source_area.materials if m.type == "yellow"]
                if yellow_buckets:
                    # Prioritize transport to roving_waiting
                    dest_areas = [a for a in self.areas if a.type == "roving_waiting" and not a.is_full()]
                    if not dest_areas:
                        dest_areas = [a for a in self.areas if a.type == "drawing2_waiting" and not a.is_full()]

                    for dest_area in dest_areas:
                        # Skip if a similar task already exists
                        if self.check_if_task_exists("transport", source_area, dest_area, None):
                            continue

                        count = min(len(yellow_buckets), 6, dest_area.capacity - len(dest_area.materials))
                        if count > 0:
                            batch_id = yellow_buckets[0].batch_id
                            task_id = len(self.tasks)
                            # Higher priority for yellow→roving_waiting
                            priority = 20 if dest_area.type == "roving_waiting" else 15
                            task = Task(task_id, "transport", priority, source_area, dest_area, yellow_buckets[:count],
                                        batch_id=batch_id)
                            task.creation_time = self.current_time
                            task.timeout = self.current_time + 3000
                            self.tasks.append(task)
                            self.stats['task_stats']['created'] += 1
                            self.stats['task_stats']['by_category'][task.category] += 1
                            self.total_ideal_time += task.ideal_time
                            self.mark_task_active(task)

                            task.update_priority(self.current_time)
                            heapq.heappush(self.task_queue, task)
                            logger.info(
                                f"Created transport task: {count} yellow buckets from {source_area.id} to {dest_area.id}")
                            new_tasks += 1
                            break

        # 3.4 Red buckets to completed products area (highest priority transport)
        completed_area = self.areas[21]  # Completed products area
        for source_area in self.areas:
            if source_area.type == "roving_finished" and not source_area.is_empty():
                if not completed_area.is_full():
                    # Skip if a similar task already exists
                    if self.check_if_task_exists("transport", source_area, completed_area, None):
                        continue

                    red_materials = source_area.get_material_by_type("red")
                    if red_materials:
                        batch_id = red_materials[0].batch_id
                        task_id = len(self.tasks)
                        task = Task(task_id, "transport", 40, source_area, completed_area, red_materials,
                                    batch_id=batch_id)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 2000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        self.total_ideal_time += task.ideal_time
                        self.mark_task_active(task)

                        if batch_id:
                            self.stats['task_stats']['by_batch'][batch_id] += 1
                            if batch_id not in self.batch_tasks:
                                self.batch_tasks[batch_id] = []
                            self.batch_tasks[batch_id].append(task)

                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.info(
                            f"Created high priority transport task: {len(red_materials)} red buckets to completed area")
                        new_tasks += 1

        # Update all task priorities
        self._update_all_task_priorities()

        # Log task generation
        if new_tasks > 0:
            logger.info(f"Generated {new_tasks} new tasks, queue now has {len(self.task_queue)} tasks")

        # Cleanup expired or stale tasks
        self._cleanup_task_queue()

    def _update_all_task_priorities(self):
        """Update priorities of all tasks in queue"""
        updated_queue = []
        while self.task_queue:
            task = heapq.heappop(self.task_queue)
            if task.is_expired(self.current_time):
                task.status = "failed"
                self.failed_task_ids.add(task.id)
                self.stats['task_stats']['failed'] += 1
                self.remove_task_active(task)
                continue

            task.update_priority(self.current_time)
            updated_queue.append(task)

        # Rebuild task queue
        self.task_queue = []
        for task in updated_queue:
            heapq.heappush(self.task_queue, task)

    def _cleanup_task_queue(self):
        """Clean up expired or stale tasks from queue"""
        cutoff_time = self.current_time - 3000  # Reduced from 5000 to 3000
        tasks_to_keep = []

        while self.task_queue:
            task = heapq.heappop(self.task_queue)
            # Force tasks are always kept
            if task.is_force_task:
                tasks_to_keep.append(task)
                continue

            # Check if task is still valid
            if task.status == "pending" and task.creation_time > cutoff_time and not task.is_expired(self.current_time):
                # For transport tasks, check if materials still exist at source
                if task.type == "transport" and isinstance(task.source, Area):
                    materials_valid = all(material in task.source.materials for material in task.materials)
                    if not materials_valid:
                        task.status = "failed"
                        self.failed_task_ids.add(task.id)
                        self.stats['task_stats']['failed'] += 1
                        self.remove_task_active(task)
                        continue

                # For machine pickup tasks, check if machine is still valid
                if task.type == "transport" and isinstance(task.source, Machine):
                    if task.source.status != "waiting_for_pickup" or not task.source.materials:
                        # Machine no longer needs pickup, remove task
                        task.source.has_pickup_task = False
                        task.status = "failed"
                        self.failed_task_ids.add(task.id)
                        self.stats['task_stats']['failed'] += 1
                        self.remove_task_active(task)
                        continue

                # Keep valid task
                tasks_to_keep.append(task)
            else:
                # Mark expired tasks as failed
                if task.status == "pending":
                    task.status = "failed"
                    self.failed_task_ids.add(task.id)
                    self.stats['task_stats']['failed'] += 1
                    self.remove_task_active(task)

        # Rebuild queue with valid tasks
        self.task_queue = []
        for task in tasks_to_keep:
            heapq.heappush(self.task_queue, task)

    def generate_monitor_tasks(self):
        """Generate machine monitoring tasks"""
        for machine in self.machines:
            if machine.status == "processing":
                # Check if there's already a monitoring task
                existing_task = False
                for task in self.monitor_tasks:
                    if task.destination == machine and task.status in ["pending", "assigned", "in_progress"]:
                        existing_task = True
                        break

                if not existing_task:
                    task_id = len(self.tasks)
                    batch_id = machine.current_batch_id
                    task = Task(task_id, "monitor", 5, None, machine, [], batch_id=batch_id)
                    task.creation_time = self.current_time
                    task.timeout = self.current_time + machine.process_end_time - self.current_time + 100
                    self.tasks.append(task)
                    self.monitor_tasks.append(task)
                    self.stats['task_stats']['created'] += 1
                    self.stats['task_stats']['by_category']["monitor"] += 1
                    self.total_ideal_time += task.ideal_time

                    # Find closest available robot for monitoring
                    available_robots = [r for r in self.robots if r.is_available_for_monitoring()]
                    if available_robots:
                        # Sort by distance to machine
                        sorted_robots = sorted(available_robots,
                                               key=lambda r: math.sqrt((r.position[0] - machine.position[0]) ** 2 +
                                                                       (r.position[1] - machine.position[1]) ** 2))

                        # Assign to closest robot
                        if sorted_robots:
                            robot = sorted_robots[0]
                            task.assigned_robot = robot
                            task.status = "assigned"
                            robot.start_monitoring(machine, self.current_time)

        # Clean up completed monitoring tasks
        self.monitor_tasks = [t for t in self.monitor_tasks if t.status not in ["completed", "failed"]]

    def assign_tasks(self):
        """Assign tasks to available robots"""
        available_robots = [robot for robot in self.robots if robot.is_available()]

        if not available_robots or not self.task_queue:
            return

        # Handle critical pickup tasks first
        critical_tasks = []
        for task in self.task_queue:
            if (task.status == "pending" and
                    ((task.type == "transport" and
                      isinstance(task.source, Machine) and
                      task.source.status == "waiting_for_pickup") or
                     task.is_force_task)):
                critical_tasks.append(task)

        # Assign robots to critical tasks
        for task in critical_tasks:
            if not available_robots:
                break

            # Find closest robot
            closest_robot = min(available_robots,
                                key=lambda r: ((r.position[0] - task.source.position[0]) ** 2 +
                                               (r.position[1] - task.source.position[1]) ** 2))

            success = self._try_start_transport_task(closest_robot, task)
            if success:
                available_robots.remove(closest_robot)
                task.status = "assigned"
                task.assigned_robot = closest_robot
                logger.info(
                    f"Assigned critical {'force' if task.is_force_task else 'pickup'} task to robot {closest_robot.id}")

        # Regular task assignments
        temp_queue = self.task_queue.copy()
        heapq.heapify(temp_queue)

        assigned_tasks = []
        while available_robots and temp_queue:
            task = heapq.heappop(temp_queue)

            if task.status != "pending":
                continue

            # Find best robot-task match
            best_robot = None
            best_score = float('-inf')

            for robot in available_robots:
                score = self._calculate_task_robot_match(task, robot)
                if score > best_score:
                    best_score = score
                    best_robot = robot

            if best_robot and best_score > 0:
                # Try to start the task
                success = False
                if task.type == "transport":
                    success = self._try_start_transport_task(best_robot, task)
                elif task.type == "process":
                    success = self._try_start_process_task(best_robot, task)

                if success:
                    task.status = "assigned"
                    task.assigned_robot = best_robot
                    task.assignment_time = self.current_time
                    available_robots.remove(best_robot)
                    assigned_tasks.append(task)
                    logger.debug(f"Assigned {task.type} task {task.id} to robot {best_robot.id}")

        # Update the main task queue
        new_queue = []
        assigned_task_ids = {task.id for task in assigned_tasks}

        while self.task_queue:
            task = heapq.heappop(self.task_queue)
            if task.id not in assigned_task_ids:
                new_queue.append(task)

        self.task_queue = []
        for task in new_queue:
            heapq.heappush(self.task_queue, task)

    def _calculate_task_robot_match(self, task, robot):
        """Calculate task-robot match score"""
        # Base score = task dynamic priority
        score = task.dynamic_priority

        # Robots carrying materials aren't suitable for new tasks
        if robot.carrying:
            return -100

        # Calculate distance to task source
        source_pos = None
        if task.type == "transport" and task.source:
            if isinstance(task.source, Area):
                source_pos = task.source.get_center()
            elif isinstance(task.source, Machine):
                source_pos = task.source.position

        if source_pos:
            distance = math.sqrt((robot.position[0] - source_pos[0]) ** 2 +
                                 (robot.position[1] - source_pos[1]) ** 2)
            # Distance factor: closer is better
            distance_factor = 1000 / (distance + 100)
            score += distance_factor

        # Consider robot's assigned region
        if robot.assigned_region:
            # Check if task is related to robot's region
            task_region = None
            if task.type == "transport":
                if task.source and isinstance(task.source, Area):
                    task_region = task.source.type
                elif task.destination and isinstance(task.destination, Area):
                    task_region = task.destination.type

            # Boost score if task region matches robot's assigned region
            if task_region and robot.assigned_region in task_region:
                score += 5

        # Give higher priority to tasks involving red buckets
        if task.type == "transport" and any(m.type == "red" for m in task.materials):
            score += 15  # Much higher priority for red buckets
            # Even higher priority for moving to completed area
            if isinstance(task.destination, Area) and task.destination.type == "completed_products":
                score += 20

        # Give higher priority to yellow buckets to roving waiting areas
        if (task.type == "transport" and
                any(m.type == "yellow" for m in task.materials) and
                isinstance(task.destination, Area) and
                task.destination.type == "roving_waiting"):
            score += 10

        return score

    def _try_start_transport_task(self, robot, task):
        """Try to start a transport task"""
        source_pos = None
        if isinstance(task.source, Area):
            source_pos = task.source.get_center()
        elif isinstance(task.source, Machine):
            source_pos = task.source.position

        if source_pos:
            started = robot.move_to(source_pos, self.current_time, self.grid_map)
            if started:
                task.status = "in_progress"
                task.start_time = self.current_time
                robot.current_task = task
                logger.debug(f"Robot {robot.id} started transport task {task.id}")
                return True
        return False

    def _try_start_process_task(self, robot, task):
        """Try to start a process task"""
        source_pos = task.source.get_center()

        # Special handling for batch processes
        if isinstance(task.destination, Machine):
            if task.destination.type == "drawing1" and len(task.materials) != 6:
                logger.warning(f"Drawing1 process task needs exactly 6 green buckets")
                return False
            elif task.destination.type in ["drawing2", "roving"] and len(task.materials) != 6:
                logger.warning(f"{task.destination.type} process task needs exactly 6 yellow buckets")
                return False

        # Start robot movement
        started = robot.move_to(source_pos, self.current_time, self.grid_map)
        if started:
            task.status = "in_progress"
            task.start_time = self.current_time
            robot.current_task = task
            logger.info(f"Robot {robot.id} started process task {task.id}")
            return True
        return False

    def process_robot_actions(self):
        """Process all robot actions"""
        for robot in self.robots:
            # Handle stuck robots
            if robot.is_stuck(self.current_time, 100):
                logger.warning(f"Robot {robot.id} appears stuck, resetting state")
                robot.status = "idle"
                robot.path = []
                if robot.current_task:
                    task = robot.current_task
                    # Put critical pickup tasks back on queue with higher priority
                    if (task.type == "transport" and
                            isinstance(task.source, Machine) and
                            task.source.status == "waiting_for_pickup"):
                        task.status = "pending"
                        task.priority += 5  # Increase priority
                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.info(f"Requeued critical pickup task: {task.id}")
                    else:
                        task.status = "pending"
                        task.attempt_count += 1
                        heapq.heappush(self.task_queue, task)
                    robot.current_task = None

                # Move robot to random position to avoid getting stuck
                robot.position = (random.uniform(50, self.width - 50), random.uniform(50, self.height - 50))
                continue

            # Handle monitoring tasks
            if robot.monitoring_task:
                machine = robot.monitoring_task['target']
                # Stop monitoring if machine is no longer processing
                if machine.status != "processing":
                    robot.stop_monitoring()
                    # Complete corresponding monitoring task
                    for task in self.monitor_tasks:
                        if task.destination == machine and task.assigned_robot == robot and task.status == "in_progress":
                            task.status = "completed"
                            task.completion_time = self.current_time
                            self.completed_task_ids.add(task.id)
                            self.stats['task_stats']['completed'] += 1
                            self.completed_tasks_ideal_time += task.ideal_time
                            break

            # Regular movement logic
            if robot.status == "moving" and robot.current_task:
                # Update robot position
                reached_destination = robot.update_position(self.current_time)

                if reached_destination:
                    task = robot.current_task

                    if task.type == "transport":
                        if not robot.carrying:
                            # At source location, pick up materials
                            all_picked = True
                            for material in task.materials:
                                # Check if material is still at source
                                if isinstance(task.source, Area):
                                    found = material in task.source.materials
                                    if found:
                                        task.source.remove_material(material)
                                        robot.pick_material(material, self.current_time)
                                    else:
                                        all_picked = False
                                elif isinstance(task.source, Machine):
                                    found = material in task.source.materials
                                    if found:
                                        task.source.materials.remove(material)
                                        robot.pick_material(material, self.current_time)
                                    else:
                                        all_picked = False

                            if not all_picked or not robot.carrying:
                                # Material not found, task failed
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1
                                # Mark that the machine no longer has a pickup task
                                if isinstance(task.source, Machine):
                                    task.source.has_pickup_task = False
                                self.remove_task_active(task)
                                robot.current_task = None
                                robot.consecutive_failures += 1
                                continue

                            # Move to destination
                            dest_pos = None
                            if isinstance(task.destination, Area):
                                dest_pos = task.destination.get_center()
                            elif isinstance(task.destination, Machine):
                                dest_pos = task.destination.position

                            if dest_pos:
                                robot.move_to(dest_pos, self.current_time, self.grid_map)
                            else:
                                # Invalid destination, task failed
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1
                                self.remove_task_active(task)
                                robot.current_task = None
                        else:
                            # At destination, drop materials
                            if isinstance(task.destination, Area):
                                success = robot.drop_material(task.destination, self.current_time)
                                if success:
                                    # Materials successfully placed, task completed
                                    task.status = "completed"
                                    task.completion_time = self.current_time
                                    self.completed_task_ids.add(task.id)
                                    self.stats['task_stats']['completed'] += 1
                                    self.completed_tasks_ideal_time += task.ideal_time
                                    # Remove from active tasks
                                    self.remove_task_active(task)
                                    robot.current_task = None
                                    robot.consecutive_failures = 0

                                    # Handle special case: Machine to finished area material transformation
                                    if isinstance(task.source, Machine):
                                        machine = task.source
                                        # Reset machine state
                                        machine.status = "idle"
                                        machine.materials = []
                                        machine.has_pickup_task = False

                                        # Material transformation
                                        area = task.destination
                                        material_type = None
                                        batch_id = task.batch_id

                                        if machine.type == "carding":
                                            # empty → green
                                            material_type = "green"
                                            self.stats['green_buckets_produced'] += len(task.materials)
                                            self.flow_trackers['empty_to_green'] += len(task.materials)
                                            logger.info(
                                                f"Produced {len(task.materials)} green buckets, batch {batch_id}")
                                        elif machine.type == "drawing1":
                                            # 6 green → 1 yellow
                                            material_type = "yellow"
                                            self.stats['yellow_buckets_produced'] += 1
                                            self.flow_trackers['green_to_yellow'] += 1
                                            logger.info(f"Produced 1 yellow bucket, batch {batch_id}")
                                        elif machine.type == "drawing2":
                                            # 6 yellow → 1 yellow (keep yellow)
                                            material_type = "yellow"
                                            self.stats['yellow_buckets_produced'] += 1
                                            logger.info(f"Produced 1 yellow bucket (drawing2), batch {batch_id}")
                                        elif machine.type == "roving":
                                            # 6 yellow → 1 red
                                            material_type = "red"
                                            self.stats['red_buckets_produced'] += 1
                                            self.flow_trackers['yellow_to_red'] += 1
                                            self.last_red_bucket_production = self.current_time
                                            logger.info(f"!!! PRODUCED 1 RED BUCKET, batch {batch_id} !!!")

                                            # Create immediate task to transport this red bucket to completed area
                                            red_material = None
                                            for m in area.materials:
                                                if m.type == "red":
                                                    red_material = m
                                                    break

                                            if red_material:
                                                completed_area = self.areas[21]  # Completed area
                                                if not completed_area.is_full():
                                                    transport_task_id = len(self.tasks)
                                                    transport_task = Task(transport_task_id, "transport", 100,
                                                                          area, completed_area, [red_material],
                                                                          batch_id=batch_id)
                                                    transport_task.creation_time = self.current_time
                                                    transport_task.timeout = self.current_time + 3000
                                                    transport_task.is_force_task = True
                                                    self.tasks.append(task)
                                                    self.stats['task_stats']['created'] += 1
                                                    self.total_ideal_time += transport_task.ideal_time

                                                    # Add directly to queue
                                                    transport_task.update_priority(self.current_time)
                                                    heapq.heappush(self.task_queue, transport_task)
                                                    logger.info(
                                                        f"Created immediate RED transport task after production")

                                        # Clear area contents (just dropped)
                                        area.materials = []

                                        # Create new material
                                        if material_type == "green":
                                            # Each empty bucket becomes one green bucket
                                            for i in range(len(task.materials)):
                                                material_id = len(self.materials)
                                                new_material = Material(material_id, material_type)
                                                new_material.creation_time = self.current_time
                                                new_material.batch_id = batch_id
                                                self.materials.append(new_material)
                                                area.add_material(new_material)

                                                # Update batch statistics
                                                if batch_id in self.batch_status:
                                                    self.batch_status[batch_id]['green_created'] += 1
                                        elif material_type in ["yellow", "red"]:
                                            # Batch conversion
                                            material_id = len(self.materials)
                                            new_material = Material(material_id, material_type)
                                            new_material.creation_time = self.current_time
                                            new_material.batch_id = batch_id
                                            self.materials.append(new_material)
                                            area.add_material(new_material)

                                            # Update batch statistics
                                            if batch_id in self.batch_status:
                                                if material_type == "yellow":
                                                    self.batch_status[batch_id]['yellow_created'] += 1
                                                elif material_type == "red":
                                                    self.batch_status[batch_id]['red_created'] += 1
                                                    # Mark batch as completed if a red bucket is produced
                                                    self.batch_status[batch_id]['completed'] = True

                                    # Check if red buckets were delivered to completed area
                                    if isinstance(task.destination,
                                                  Area) and task.destination.type == "completed_products":
                                        # Count how many red buckets were delivered
                                        red_count = sum(1 for m in task.materials if m.type == "red")
                                        if red_count > 0:
                                            self.red_buckets_in_completed_area += red_count
                                            logger.info(
                                                f"Delivered {red_count} red buckets to completed area, total: {self.red_buckets_in_completed_area}/{self.target_red_buckets}")
                                else:
                                    # Drop failed, retry task
                                    task.status = "pending"
                                    task.attempt_count += 1
                                    robot.consecutive_failures += 1
                                    heapq.heappush(self.task_queue, task)
                                    robot.current_task = None
                            elif isinstance(task.destination, Machine):
                                # Put materials into machine
                                machine = task.destination
                                if machine.is_available():
                                    # Add materials to machine
                                    for material in robot.carrying:
                                        machine.materials.append(material)

                                    # Set batch ID for machine processing
                                    if robot.carrying and robot.carrying[0].batch_id:
                                        machine.current_batch_id = robot.carrying[0].batch_id

                                    robot.carrying = []

                                    # Start machine processing
                                    success = machine.start_processing(self.current_time)
                                    if success:
                                        task.status = "completed"
                                        task.completion_time = self.current_time
                                        self.completed_task_ids.add(task.id)
                                        self.stats['task_stats']['completed'] += 1
                                        self.completed_tasks_ideal_time += task.ideal_time
                                        self.remove_task_active(task)
                                        robot.current_task = None
                                        robot.consecutive_failures = 0
                                    else:
                                        task.status = "failed"
                                        self.failed_task_ids.add(task.id)
                                        self.stats['task_stats']['failed'] += 1
                                        self.remove_task_active(task)
                                        robot.current_task = None
                                        robot.consecutive_failures += 1
                                        machine.consecutive_failures += 1
                                else:
                                    # Machine not available, return to source
                                    source_pos = task.source.get_center()
                                    robot.move_to(source_pos, self.current_time, self.grid_map)
                                    task.status = "pending"
                                    task.attempt_count += 1
                                    heapq.heappush(self.task_queue, task)
                    elif task.type == "process":
                        if not robot.carrying:
                            # At process source area, pick up materials
                            materials_to_pick = []
                            for material in task.materials:
                                if material in task.source.materials:
                                    materials_to_pick.append(material)
                                    task.source.remove_material(material)

                            # Pick up materials
                            all_picked = True
                            for material in materials_to_pick:
                                if not robot.pick_material(material, self.current_time):
                                    all_picked = False
                                    break

                            if not all_picked or not robot.carrying:
                                # Pickup failed, return materials to area
                                for material in robot.carrying:
                                    task.source.add_material(material)
                                robot.carrying = []

                                # Task failed
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1
                                self.remove_task_active(task)
                                robot.current_task = None
                                robot.consecutive_failures += 1
                                continue

                            # Move to machine
                            machine_pos = task.destination.position
                            robot.move_to(machine_pos, self.current_time, self.grid_map)
                        else:
                            # At machine, load materials
                            machine = task.destination

                            if machine.is_available():
                                # Load materials into machine
                                batch_id = None
                                for material in robot.carrying:
                                    machine.materials.append(material)
                                    if material.batch_id and not batch_id:
                                        batch_id = material.batch_id

                                # Set batch ID for tracking
                                if batch_id:
                                    machine.current_batch_id = batch_id

                                robot.carrying = []

                                # Start machine processing
                                success = machine.start_processing(self.current_time)
                                if success:
                                    task.status = "completed"
                                    task.completion_time = self.current_time
                                    self.completed_task_ids.add(task.id)
                                    self.stats['task_stats']['completed'] += 1
                                    self.completed_tasks_ideal_time += task.ideal_time
                                    self.remove_task_active(task)
                                    robot.current_task = None
                                    robot.consecutive_failures = 0

                                    # Special tracking for roving machine starting
                                    if machine.type == "roving":
                                        logger.info(
                                            f"Roving machine {machine.id} has started processing 6 yellow buckets, batch {batch_id}")
                                else:
                                    task.status = "failed"
                                    self.failed_task_ids.add(task.id)
                                    self.stats['task_stats']['failed'] += 1
                                    self.remove_task_active(task)
                                    robot.current_task = None
                                    robot.consecutive_failures += 1
                                    machine.consecutive_failures += 1
                            else:
                                # Machine not available, return to source
                                source_pos = task.source.get_center()
                                robot.move_to(source_pos, self.current_time, self.grid_map)
                                task.status = "pending"
                                task.attempt_count += 1
                                heapq.heappush(self.task_queue, task)
            else:
                # Update non-moving robot position
                robot.update_position(self.current_time, 1)

    def update_machines(self):
        """Update all machine states"""
        for machine in self.machines:
            # Check if processing is complete
            if machine.status == "processing":
                machine.check_process_completion(self.current_time)

            # Update machine status statistics
            machine.update_machine_status(self.current_time, 1)

            # Auto-reset machines that have no materials but are in waiting_for_pickup state
            if machine.status == "waiting_for_pickup" and not machine.materials:
                machine.status = "idle"
                machine.has_pickup_task = False
                logger.info(f"Auto-reset machine {machine.id} to idle state")

            # Check for stuck machines
            if machine.is_stuck(self.current_time, 3000):
                logger.warning(f"Machine {machine.id} appears stuck, trying to resolve")

                # If machine is waiting for pickup, force a pickup task
                if machine.status == "waiting_for_pickup" and machine.materials:
                    finished_area = self._find_corresponding_finished_area(machine)
                    if finished_area and not finished_area.is_full():
                        task_id = len(self.tasks)
                        task = Task(task_id, "transport", 50, machine, finished_area, machine.materials[:],
                                    batch_id=machine.current_batch_id)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 2000
                        task.is_force_task = True
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.total_ideal_time += task.ideal_time
                        self.stats['forced_actions'] += 1
                        machine.has_pickup_task = True

                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.info(f"Created FORCE pickup task for stuck machine {machine.id}")

                # If nothing works, reset the machine
                if machine.consecutive_failures > 3:
                    machine.require_maintenance(200)

    def update_areas(self):
        """Update all area states"""
        for area in self.areas:
            area.update_materials_status(self.current_time, 1)

    def check_production_progress(self):
        """Check if production is making progress"""
        if self.current_time - self.last_progress_check < self.progress_check_interval:
            return

        self.last_progress_check = self.current_time

        # Check for stalled production - no red buckets after a long time
        if (self.red_buckets_in_completed_area == 0 and
                self.current_time > 10000 and
                self.current_time - self.last_red_bucket_production > 20000):

            logger.warning(f"Production appears stalled: No red buckets after {self.current_time} seconds")

            # Check for yellow buckets in system
            yellow_count = sum(1 for m in self.materials if m.type == "yellow")
            green_count = sum(1 for m in self.materials if m.type == "green")

            logger.info(f"Current material counts: green={green_count}, yellow={yellow_count}, " +
                        f"red={sum(1 for m in self.materials if m.type == 'red')}")

            # If enough materials for yellow→red conversion, force it
            if yellow_count >= 6:
                # Set flag for major reset next cycle
                self.major_reset_needed = True

        # Check if we need to do a major reset
        if self.major_reset_needed and self.current_time - self.last_major_reset > 10000:
            self.do_major_system_reset()
            self.last_major_reset = self.current_time
            self.major_reset_needed = False

    def do_major_system_reset(self):
        """Perform a major system reset to unblock production"""
        logger.warning(f"=== PERFORMING MAJOR SYSTEM RESET AT TIME {self.current_time} ===")

        # 1. Reset all machine states
        for machine in self.machines:
            if machine.status == "processing":
                # Force completion
                machine.process_end_time = self.current_time
                machine.check_process_completion(self.current_time)
                logger.info(f"Force completed machine {machine.id} ({machine.type})")

            # Forcibly convert materials if machine is waiting for pickup
            if machine.status == "waiting_for_pickup" and machine.materials:
                finished_area = self._find_corresponding_finished_area(machine)
                if finished_area:
                    # Transform materials based on machine type
                    if machine.type == "carding" and any(m.type == "empty" for m in machine.materials):
                        # Convert empties to greens
                        for m in machine.materials[:]:
                            if m.type == "empty":
                                new_material = Material(len(self.materials), "green")
                                new_material.creation_time = self.current_time
                                new_material.batch_id = m.batch_id
                                self.materials.append(new_material)
                                finished_area.add_material(new_material)
                                self.stats['green_buckets_produced'] += 1
                                logger.info(f"FORCE converted empty to green in area {finished_area.id}")

                    elif machine.type == "drawing1" and any(m.type == "green" for m in machine.materials):
                        # Convert 6 greens to 1 yellow
                        green_count = sum(1 for m in machine.materials if m.type == "green")
                        if green_count >= 6:
                            batch_id = next((m.batch_id for m in machine.materials if m.batch_id), 1)
                            new_material = Material(len(self.materials), "yellow")
                            new_material.creation_time = self.current_time
                            new_material.batch_id = batch_id
                            self.materials.append(new_material)
                            finished_area.add_material(new_material)
                            self.stats['yellow_buckets_produced'] += 1
                            logger.info(f"FORCE converted 6 greens to 1 yellow in area {finished_area.id}")

                    elif machine.type in ["drawing2", "roving"] and any(m.type == "yellow" for m in machine.materials):
                        # Convert 6 yellows to 1 red if roving machine
                        yellow_count = sum(1 for m in machine.materials if m.type == "yellow")
                        if yellow_count >= 6 and machine.type == "roving":
                            batch_id = next((m.batch_id for m in machine.materials if m.batch_id), 1)
                            new_material = Material(len(self.materials), "red")
                            new_material.creation_time = self.current_time
                            new_material.batch_id = batch_id
                            self.materials.append(new_material)
                            finished_area.add_material(new_material)
                            self.stats['red_buckets_produced'] += 1
                            self.last_red_bucket_production = self.current_time
                            logger.info(f"!!! FORCE PRODUCED RED BUCKET in area {finished_area.id} !!!")

                            # Immediately create task to transport red to completed area
                            completed_area = self.areas[21]
                            task_id = len(self.tasks)
                            task = Task(task_id, "transport", 100, finished_area, completed_area, [new_material],
                                        batch_id=batch_id)
                            task.creation_time = self.current_time
                            task.is_force_task = True
                            self.tasks.append(task)
                            self.stats['task_stats']['created'] += 1
                            self.total_ideal_time += task.ideal_time
                            self.stats['forced_actions'] += 1

                            task.update_priority(self.current_time)
                            heapq.heappush(self.task_queue, task)

                        # Just clone yellows for drawing2 machine
                        elif machine.type == "drawing2":
                            for m in machine.materials[:]:
                                if m.type == "yellow":
                                    new_material = Material(len(self.materials), "yellow")
                                    new_material.creation_time = self.current_time
                                    new_material.batch_id = m.batch_id
                                    self.materials.append(new_material)
                                    finished_area.add_material(new_material)
                                    logger.info(f"FORCE copied yellow in drawing2 area {finished_area.id}")

            # Reset machine state
            machine.status = "idle"
            machine.materials = []
            machine.has_pickup_task = False
            machine.waiting_for_pickup_time = 0
            machine.consecutive_failures = 0

        # 2. Reset robots
        for robot in self.robots:
            robot.status = "idle"
            robot.path = []
            robot.carrying = []
            robot.current_task = None
            robot.consecutive_failures = 0
            robot.stuck_count = 0

        # 3. Clean up task queue
        self.task_queue = []
        self.active_tasks = set()

        # 4. Check and force distribute materials if needed
        # Make sure drawing1_waiting has enough greens
        green_in_drawing1 = sum(
            sum(1 for m in area.materials if m.type == "green")
            for area in self.areas if area.type == "drawing1_waiting"
        )

        if green_in_drawing1 < 12:  # Need at least 12 greens for 2 yellow conversions
            # Find carding_finished areas with greens
            for source_area in [a for a in self.areas if a.type == "carding_finished"]:
                greens = [m for m in source_area.materials if m.type == "green"]
                if greens:
                    # Move to drawing1_waiting areas
                    for dest_area in [a for a in self.areas if a.type == "drawing1_waiting" and not a.is_full()]:
                        count = min(len(greens), 6, dest_area.capacity - len(dest_area.materials))
                        if count > 0:
                            task_id = len(self.tasks)
                            task = Task(task_id, "transport", 20, source_area, dest_area, greens[:count],
                                        batch_id=greens[0].batch_id if greens[0].batch_id else 1)
                            task.creation_time = self.current_time
                            task.is_force_task = True
                            self.tasks.append(task)
                            self.stats['task_stats']['created'] += 1
                            self.total_ideal_time += task.ideal_time

                            task.update_priority(self.current_time)
                            heapq.heappush(self.task_queue, task)
                            logger.info(f"Created FORCE green transport task: {source_area.id} to {dest_area.id}")

                            # Remove these greens from consideration
                            greens = greens[count:]
                            if not greens:
                                break

        # Make sure roving_waiting has enough yellows
        yellow_in_roving = sum(
            sum(1 for m in area.materials if m.type == "yellow")
            for area in self.areas if area.type == "roving_waiting"
        )

        if yellow_in_roving < 12:  # Need at least 12 yellows for 2 red conversions
            # Find areas with yellows
            for source_area in [a for a in self.areas if a.type in ["drawing1_finished", "drawing2_finished"]]:
                yellows = [m for m in source_area.materials if m.type == "yellow"]
                if yellows:
                    # Move to roving_waiting areas
                    for dest_area in [a for a in self.areas if a.type == "roving_waiting" and not a.is_full()]:
                        count = min(len(yellows), 6, dest_area.capacity - len(dest_area.materials))
                        if count > 0:
                            task_id = len(self.tasks)
                            task = Task(task_id, "transport", 25, source_area, dest_area, yellows[:count],
                                        batch_id=yellows[0].batch_id if yellows[0].batch_id else 1)
                            task.creation_time = self.current_time
                            task.is_force_task = True
                            self.tasks.append(task)
                            self.stats['task_stats']['created'] += 1
                            self.total_ideal_time += task.ideal_time

                            task.update_priority(self.current_time)
                            heapq.heappush(self.task_queue, task)
                            logger.info(f"Created FORCE yellow transport task: {source_area.id} to {dest_area.id}")

                            # Remove these yellows from consideration
                            yellows = yellows[count:]
                            if not yellows:
                                break

        # 5. Force create new red buckets if no reds in system
        red_count = sum(1 for m in self.materials if m.type == "red")
        if red_count == 0:
            # Find roving machines
            for machine in [m for m in self.machines if m.type == "roving" and m.status == "idle"]:
                # Find areas with yellows
                for area in [a for a in self.areas if a.type == "roving_waiting"]:
                    yellows = [m for m in area.materials if m.type == "yellow"]
                    if len(yellows) >= 6:
                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 50, area, machine, yellows[:6],
                                    batch_id=yellows[0].batch_id if yellows[0].batch_id else 1)
                        task.creation_time = self.current_time
                        task.is_force_task = True
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.total_ideal_time += task.ideal_time

                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)
                        logger.info(f"Created FORCE process task: 6 yellows to roving machine {machine.id}")
                        break
                break

        # Record reset in stats
        self.stats['deadlocks'].append({
            'time': self.current_time,
            'type': 'major_system_reset',
            'red_count': self.red_buckets_in_completed_area
        })
        self.stats['forced_actions'] += 1

    def check_for_deadlocks(self):
        """Check for and resolve deadlocks"""
        # Check if we should do a major system reset
        if self.current_time % 10000 == 0 and self.red_buckets_in_completed_area == 0 and self.current_time > 30000:
            logger.warning(f"No progress after {self.current_time} seconds, forcing system reset")
            self.do_major_system_reset()
            return

        # Check for machines waiting for pickup
        machines_waiting_pickup = [m for m in self.machines if
                                   m.status == "waiting_for_pickup" and not m.has_pickup_task]
        idle_robots = [r for r in self.robots if r.status == "idle"]

        # If machines are waiting for pickup but no progress
        if (len(machines_waiting_pickup) > 0 and
                len(idle_robots) > 0 and
                self.current_time - self.last_progress_time > 1000):

            self.deadlock_count += 1
            logger.warning(f"Detected deadlock #{self.deadlock_count}: {len(machines_waiting_pickup)} machines waiting")

            # Create emergency pickup tasks
            for machine in machines_waiting_pickup[:min(3, len(machines_waiting_pickup))]:
                if machine.has_pickup_task:
                    continue  # Skip machines with existing pickup tasks

                finished_area = self._find_corresponding_finished_area(machine)
                if finished_area and not finished_area.is_full():
                    # Find closest idle robot
                    closest_robot = min(idle_robots, key=lambda r:
                    ((r.position[0] - machine.position[0]) ** 2 +
                     (r.position[1] - machine.position[1]) ** 2))

                    # Force robot to move to machine
                    logger.info(f"Forcing robot {closest_robot.id} to perform emergency pickup")
                    closest_robot.status = "moving"
                    closest_robot.destination = machine.position
                    closest_robot.last_move_time = self.current_time
                    closest_robot.path = [machine.position]

                    # Create task
                    task_id = len(self.tasks)
                    batch_id = machine.current_batch_id
                    task = Task(task_id, "transport", 30, machine, finished_area, machine.materials[:],
                                batch_id=batch_id)
                    task.creation_time = self.current_time
                    task.start_time = self.current_time
                    task.status = "in_progress"
                    closest_robot.current_task = task
                    self.tasks.append(task)
                    self.total_ideal_time += task.ideal_time
                    self.stats['forced_actions'] += 1

                    # Mark machine as having a pickup task
                    machine.has_pickup_task = True
                    machine.last_emergency_task = self.current_time
                    task_key = (task.type, machine.id, finished_area.id)
                    self.active_tasks.add(task_key)

                    # Remove assigned robot
                    idle_robots.remove(closest_robot)
                    if not idle_robots:
                        break

            self.last_progress_time = self.current_time
            self.stats['deadlocks'].append({
                'time': self.current_time,
                'type': 'pickup_deadlock',
                'red_count': sum(1 for m in self.materials if m.type == "red"),
                'task_queue_length': len(self.task_queue)
            })
            return

        # Check if any machine has been waiting for pickup too long
        for machine in machines_waiting_pickup:
            if machine.waiting_for_pickup_time > 4000:
                logger.warning(f"Machine {machine.id} waiting too long for pickup, forcing reset")
                machine.status = "idle"
                machine.waiting_for_pickup_time = 0
                machine.has_pickup_task = False
                continue

        # Check for overall progress
        if (self.red_buckets_in_completed_area > 0 or
                sum(1 for r in self.robots if r.status != "idle") > 0 or
                sum(1 for m in self.machines if m.status != "idle") > 0 or
                self.current_time - self.last_progress_time < 500):
            self.last_progress_time = self.current_time
            return

        # If no progress for a long time, resolve deadlock
        if self.current_time - self.last_progress_time > self.deadlock_threshold:
            self.deadlock_count += 1
            self.last_progress_time = self.current_time

            logger.warning(f"Detected general deadlock #{self.deadlock_count}, starting resolution...")
            self.stats['deadlocks'].append({
                'time': self.current_time,
                'type': 'general_deadlock',
                'red_count': self.red_buckets_in_completed_area,
                'task_queue_length': len(self.task_queue)
            })

            # Reset processing machines
            for machine in self.machines:
                if machine.status == "processing":
                    machine.status = "waiting_for_pickup"
                    machine.has_pickup_task = False
                    machine.check_process_completion(self.current_time)
                elif machine.consecutive_failures > 3:
                    machine.require_maintenance(300)

            # Reset stuck robots
            for robot in self.robots:
                if robot.status == "moving" and robot.current_task:
                    if robot.consecutive_failures > 3:
                        robot.require_maintenance(300)
                    else:
                        robot.status = "idle"
                        robot.path = []
                        task = robot.current_task

                        # Return materials
                        if robot.carrying:
                            robot.carrying = []

                        # Reset task
                        if task:
                            task.status = "pending"
                            task.attempt_count += 1
                            if task.attempt_count <= 5:
                                heapq.heappush(self.task_queue, task)
                            else:
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1
                                self.remove_task_active(task)

                        robot.current_task = None

            # Clean task queue
            self._cleanup_task_queue()

            # Reset active tasks tracking
            self.active_tasks = set()
            for machine in self.machines:
                machine.has_pickup_task = False

            # Regenerate grid map
            self.grid_map = create_grid_map(self.width, self.height, self.areas, self.machines)

    def update_statistics(self):
        """Update system statistics"""
        # Update robot statistics
        for robot in self.robots:
            self.stats['robot_distances'][robot.id] = robot.total_distance
            self.stats['robot_tasks'][robot.id] = robot.completed_tasks

        # Update machine utilization
        for machine in self.machines:
            self.stats['machine_utilization'][machine.id] = machine.total_processing_time / max(1,
                                                                                                self.current_time) * 100

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

        # Print status every 1000 seconds
        if self.current_time % 1000 == 0:
            logger.info(
                f"Status at {self.current_time}s: Red={red_count}, Completed red={self.red_buckets_in_completed_area}/{self.target_red_buckets}")

        # Update visualization data
        self.vis_data['time'].append(self.current_time)
        self.vis_data['red_count'].append(self.red_buckets_in_completed_area)
        self.vis_data['task_queue_length'].append(len(self.task_queue))

        for robot in self.robots:
            self.vis_data['robot_positions'][robot.id].append(robot.position)

        for material in self.materials:
            if material.location:
                self.vis_data['material_positions'][material.id].append(material.location)

        # Update efficiency calculation
        self.total_actual_time = self.current_time

        # Detailed system status every 5000 seconds
        if self.current_time % 5000 == 0:
            # Statistics by area
            area_stats = {}
            for area in self.areas:
                area_stats[area.id] = {
                    'empty': sum(1 for m in area.materials if m.type == "empty"),
                    'green': sum(1 for m in area.materials if m.type == "green"),
                    'yellow': sum(1 for m in area.materials if m.type == "yellow"),
                    'red': sum(1 for m in area.materials if m.type == "red"),
                    'total': len(area.materials)
                }

            # Machine status counts
            machine_stats = {
                'idle': sum(1 for m in self.machines if m.status == "idle"),
                'processing': sum(1 for m in self.machines if m.status == "processing"),
                'waiting_pickup': sum(1 for m in self.machines if m.status == "waiting_for_pickup")
            }

            # Output detailed log
            logger.info("========== DETAILED SYSTEM STATUS ==========")
            logger.info(
                f"Time: {self.current_time}, Red buckets in completed area: {self.red_buckets_in_completed_area}/{self.target_red_buckets}")
            logger.info(f"Task queue: {len(self.task_queue)} tasks")
            logger.info(f"Machine status: {machine_stats}")

            # Only output non-empty areas
            logger.info("Areas with materials:")
            for area_id, counts in area_stats.items():
                if counts['total'] > 0:
                    area = next(a for a in self.areas if a.id == area_id)
                    logger.info(f"  Area {area_id} ({area.type}): {counts}")

            # Output machine details
            logger.info("Machines with materials:")
            for machine in self.machines:
                if machine.materials:
                    logger.info(
                        f"  Machine {machine.id} ({machine.type}): {len(machine.materials)} materials, status={machine.status}")

            logger.info("==========================================")

    def generate_report(self, computation_time):
        """Generate simulation report"""
        # Calculate statistics
        red_count = sum(1 for m in self.materials if m.type == "red")
        empty_count = sum(1 for m in self.materials if m.type == "empty")
        green_count = sum(1 for m in self.materials if m.type == "green")
        yellow_count = sum(1 for m in self.materials if m.type == "yellow")

        # Calculate efficiency
        if self.completed_tasks_ideal_time > 0:
            efficiency = (self.completed_tasks_ideal_time / self.total_actual_time) * 100
        else:
            efficiency = 0.0
        # Identify bottlenecks
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

        # Calculate batch completion stats
        total_batches = len(self.batch_status)
        completed_batches = sum(1 for b in self.batch_status.values() if b['completed'])
        batch_completion_rate = completed_batches / max(1, total_batches) * 100

        # Print report
        print("\n===== Factory Simulation Performance Report =====")
        print(f"Total simulation time: {self.current_time} seconds ({self.current_time / 3600:.2f} hours)")
        print(f"Materials: Empty={empty_count}, Green={green_count}, Yellow={yellow_count}, Red={red_count}")
        print(f"Red buckets in completed area: {self.red_buckets_in_completed_area}/{self.target_red_buckets}")
        print(
            f"Red bucket production rate: {self.red_buckets_in_completed_area / (self.current_time / 3600):.2f} per hour")
        print(f"Production efficiency: {efficiency:.2f}% (ideal time/actual time)")
        print(f"Production bottleneck: {bottleneck}, utilization: {bottleneck_utilization:.2f}%")
        print(f"Total deadlocks: {self.deadlock_count}")
        print(f"Forced actions performed: {self.stats['forced_actions']}")
        print(f"Batch completion: {completed_batches}/{total_batches} batches ({batch_completion_rate:.2f}%)")
        print(
            f"Task statistics: {self.stats['task_stats']['completed']} completed, {self.stats['task_stats']['failed']} failed")

        # Save output directory
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"output_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Final summary
        print("\n===== Simulation Complete =====")
        print(f"Total simulation time: {self.current_time} seconds")
        print(
            f"Materials state: {{'empty': {empty_count}, 'green': {green_count}, 'yellow': {yellow_count}, 'red': {red_count}}}")
        print(f"Red buckets in completed area: {self.red_buckets_in_completed_area}/{self.target_red_buckets}")
        print(f"Completed tasks: {sum(robot.completed_tasks for robot in self.robots)}")
        print(f"Deadlocks: {self.deadlock_count}")
        print(f"Efficiency ratio: {efficiency:.2f}%")
        print(f"Actual computation time: {computation_time:.2f} seconds")

        print(f"\nReport and visualizations saved to: {output_dir}")

    def visualize_results(self):
        """Visualize simulation results"""
        # Create output directory
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"output_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Plot red bucket production
        plt.figure(figsize=(10, 6))
        plt.plot(self.vis_data['time'][::100], self.vis_data['red_count'][::100], 'r-', linewidth=2)
        plt.xlabel('Time (seconds)')
        plt.ylabel('Red Buckets in Completed Area')
        plt.title('Red Bucket Production Over Time')
        plt.grid(True)
        plt.savefig(f"{output_dir}/red_bucket_production.png")

        # Plot material counts
        times = [state['time'] for state in self.stats['material_states'][::100]]
        empty_counts = [state['empty'] for state in self.stats['material_states'][::100]]
        green_counts = [state['green'] for state in self.stats['material_states'][::100]]
        yellow_counts = [state['yellow'] for state in self.stats['material_states'][::100]]
        red_counts = [state['red'] for state in self.stats['material_states'][::100]]

        plt.figure(figsize=(12, 6))
        plt.plot(times, empty_counts, 'k-', label='Empty', linewidth=2)
        plt.plot(times, green_counts, 'g-', label='Green', linewidth=2)
        plt.plot(times, yellow_counts, 'y-', label='Yellow', linewidth=2)
        plt.plot(times, red_counts, 'r-', label='Red', linewidth=2)
        plt.xlabel('Time (seconds)')
        plt.ylabel('Count')
        plt.title('Material Count Changes')
        plt.legend()
        plt.grid(True)
        plt.savefig(f"{output_dir}/material_counts.png")

        # Plot factory layout
        plt.figure(figsize=(16, 16))

        # Draw areas
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
            elif 'completed' in area.type:
                color = 'pink'

            rect = patches.Rectangle((x, y), width, height, linewidth=1,
                                     edgecolor='black', facecolor=color, alpha=0.5)
            plt.gca().add_patch(rect)
            plt.text(x + width / 2, y + height / 2, area.type,
                     horizontalalignment='center', verticalalignment='center')

        # Draw machines
        for machine in self.machines:
            x, y = machine.position
            if machine.type == 'carding':
                color = 'blue'
            elif machine.type == 'drawing1':
                color = 'green'
            elif machine.type == 'drawing2':
                color = 'orange'
            elif machine.type == 'roving':
                color = 'red'

            plt.scatter(x, y, c=color, s=150, edgecolor='black', zorder=2)
            plt.text(x, y - 20, f"{machine.type} {machine.id}",
                     horizontalalignment='center', verticalalignment='center', fontsize=8)

        # Draw robot positions
        for robot in self.robots:
            x, y = robot.position
            plt.scatter(x, y, c='black', marker='s', s=80, zorder=3)
            plt.text(x, y + 20, f"R{robot.id}", color='black',
                     horizontalalignment='center', verticalalignment='center', fontsize=10)

        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.title('Factory Layout and Current State')
        plt.savefig(f"{output_dir}/factory_layout.png")

        # Plot task queue length over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.vis_data['time'][::100], self.vis_data['task_queue_length'][::100], 'b-', linewidth=2)
        plt.xlabel('Time (seconds)')
        plt.ylabel('Task Queue Length')
        plt.title('Task Queue Length Over Time')
        plt.grid(True)
        plt.savefig(f"{output_dir}/task_queue_length.png")

        # Save statistics to file
        with open(f"{output_dir}/simulation_stats.txt", 'w') as f:
            f.write("===== Factory Simulation Statistics =====\n")
            f.write(f"Total simulation time: {self.current_time} seconds\n")
            f.write(f"Red buckets in completed area: {self.red_buckets_in_completed_area}/{self.target_red_buckets}\n")
            f.write(f"Deadlock detections: {self.deadlock_count}\n")
            f.write(f"Forced actions performed: {self.stats['forced_actions']}\n")
            f.write(
                f"Task success rate: {self.stats['task_stats']['completed'] / max(1, self.stats['task_stats']['created']) * 100:.2f}%\n")

            # Detailed material distribution
            f.write("\nMaterial distribution by area type:\n")
            area_type_materials = {}
            for area in self.areas:
                if area.type not in area_type_materials:
                    area_type_materials[area.type] = {'empty': 0, 'green': 0, 'yellow': 0, 'red': 0}
                area_type_materials[area.type]['empty'] += sum(1 for m in area.materials if m.type == "empty")
                area_type_materials[area.type]['green'] += sum(1 for m in area.materials if m.type == "green")
                area_type_materials[area.type]['yellow'] += sum(1 for m in area.materials if m.type == "yellow")
                area_type_materials[area.type]['red'] += sum(1 for m in area.materials if m.type == "red")

            for area_type, counts in area_type_materials.items():
                f.write(f"  {area_type}: {counts}\n")

    def check_target_reached(self):
        """Check if target number of red buckets has been reached"""
        if self.red_buckets_in_completed_area >= self.target_red_buckets:
            return True
        return False

    def run(self, max_time=500000, target_red_buckets=8, update_interval=1000):
        """Run factory simulation"""
        print("Starting factory automation production simulation...")
        start_time = time.time()

        self.current_time = 0
        self.last_progress_time = 0
        self.last_task_generation = 0
        self.last_monitor_task_generation = 0
        self.red_buckets_in_completed_area = 0
        self.last_major_reset = 0

        # Initialize materials based on target
        self.generate_initial_materials(target_red_buckets)

        next_update = update_interval

        while self.current_time < max_time:
            try:
                # Generate and assign tasks
                if self.current_time - self.last_task_generation >= self.task_generation_interval:
                    self.generate_tasks()
                    self.assign_tasks()
                    self.last_task_generation = self.current_time

                # Generate monitoring tasks
                if self.current_time - self.last_monitor_task_generation >= self.monitor_task_interval:
                    self.generate_monitor_tasks()
                    self.last_monitor_task_generation = self.current_time

                # Process robot actions
                self.process_robot_actions()

                # Update machine states
                self.update_machines()

                # Update area states
                self.update_areas()

                # Check for production progress
                self.check_production_progress()

                # Check for deadlocks
                self.check_for_deadlocks()

                # Update statistics (less frequently)
                if self.current_time % 10 == 0:
                    self.update_statistics()

                # Check if target reached
                if self.check_target_reached():
                    print(
                        f"Target achieved! Produced {self.red_buckets_in_completed_area} red buckets in {self.current_time} seconds")
                    break

                # Print progress update
                if self.current_time >= next_update:
                    print(
                        f"Time: {self.current_time}s, Red buckets in completed area: {self.red_buckets_in_completed_area}/{target_red_buckets}")
                    next_update += update_interval

                # Increment time
                self.current_time += 1

            except Exception as e:
                logger.error(f"Simulation error: {str(e)}", exc_info=True)
                break

        # Final status update
        print(
            f"Time: {self.current_time}s, Red buckets in completed area: {self.red_buckets_in_completed_area}/{target_red_buckets}")

        # Generate report and visualizations
        self.generate_report(time.time() - start_time)
        try:
            self.visualize_results()
        except Exception as e:
            print(f"Visualization error: {str(e)}")

        return self.stats


def main():
    """Main function to run simulation"""
    # Create and run simulation
    simulation = FactorySimulation()
    simulation.run(max_time=500000, target_red_buckets=8, update_interval=1000)


if __name__ == "__main__":
    main()