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
import logging

# 配置日志
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
        self.path = []
        self.current_machine = None
        self.creation_history = []  # 记录材料转换历史
        self.task_priority_boost = 0  # 提升材料任务优先级的系数
        self.waiting_time = 0  # 材料等待处理的累计时间

    def __repr__(self):
        return f"Material({self.id}, {self.type}, at {self.location}, status={self.status})"

    def update_waiting_time(self, time_increment):
        """更新材料等待时间，用于动态调整任务优先级"""
        if self.status == "waiting":
            self.waiting_time += time_increment
            # 等待时间增加时提升优先级
            self.task_priority_boost = self.waiting_time / 1000  # 控制提升幅度


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
        self.idle_time = 0  # 机器空闲时间
        self.waiting_for_pickup_time = 0  # 等待取件时间
        self.last_status_change = 0
        self.consecutive_failures = 0  # 连续处理失败计数，用于死锁探测
        self.maintenance_time = 0  # 维护时间

    def is_available(self):
        """改进判断逻辑，更精确地判断机器是否可用"""
        return (self.status == "idle" and
                len(self.materials) < self.capacity and
                self.maintenance_time == 0)

    def start_processing(self, current_time):
        """开始处理材料，返回是否成功启动处理"""
        if len(self.materials) > 0 and self.status == "idle":
            self.status = "processing"
            self.process_start_time = current_time
            self.process_end_time = current_time + self.processing_time
            self.last_status_change = current_time
            self.consecutive_failures = 0  # 重置失败计数

            for material in self.materials:
                material.status = "processing"
                material.process_start_time = current_time
                material.process_end_time = current_time + self.processing_time
                material.waiting_time = 0  # 重置等待时间

            logger.info(
                f"机器 {self.id} ({self.type}) 开始处理 {len(self.materials)} 个材料，预计完成时间: {self.process_end_time}")
            return True
        else:
            self.consecutive_failures += 1
            return False

    def check_process_completion(self, current_time):
        """检查处理是否完成，返回是否完成处理"""
        if self.status == "processing" and current_time >= self.process_end_time:
            self.status = "waiting_for_pickup"
            self.total_processing_time += (self.process_end_time - self.process_start_time)
            self.processed_materials += len(self.materials)
            self.last_status_change = current_time

            for material in self.materials:
                material.status = "finished"
                material.completion_time = current_time

            logger.info(f"机器 {self.id} ({self.type}) 完成处理 {len(self.materials)} 个材料，等待取件")
            return True
        return False

    def update_machine_status(self, current_time, time_step):
        """更新机器状态和时间统计"""
        if self.status == "idle":
            self.idle_time += time_step
        elif self.status == "waiting_for_pickup":
            self.waiting_for_pickup_time += time_step

        # 更新维护时间
        if self.maintenance_time > 0:
            actual_maintenance = min(time_step, self.maintenance_time)
            self.maintenance_time -= actual_maintenance
            if self.maintenance_time == 0:
                logger.info(f"机器 {self.id} ({self.type}) 完成维护，恢复工作")

    def require_maintenance(self, duration=300):
        """机器需要维护"""
        if self.status == "idle" or self.consecutive_failures > 5:
            self.status = "idle"
            self.maintenance_time = duration
            self.materials = []  # 清空当前材料
            self.consecutive_failures = 0
            logger.info(f"机器 {self.id} ({self.type}) 进入维护状态，持续时间 {duration} 秒")
            return True
        return False

    def __repr__(self):
        status_str = self.status
        if self.maintenance_time > 0:
            status_str += f"(维护中:{self.maintenance_time}s)"
        return f"Machine({self.id}, {self.type}, at {self.position}, status={status_str}, materials={len(self.materials)}/{self.capacity})"


class Area:
    def __init__(self, area_id, area_type, top_left, bottom_right, capacity=float('inf')):
        self.id = area_id
        self.type = area_type
        self.top_left = top_left
        self.bottom_right = bottom_right
        self.capacity = capacity
        self.materials = []
        self.busy_factor = 0  # 区域拥堵因子，用于任务调度
        self.last_update = 0

    def is_full(self):
        return len(self.materials) >= self.capacity

    def is_empty(self):
        return len(self.materials) == 0

    def get_available_capacity(self):
        """返回区域剩余容量"""
        return max(0, self.capacity - len(self.materials))

    def add_material(self, material):
        """添加材料到区域，返回是否成功添加"""
        if not self.is_full():
            self.materials.append(material)
            # 分配区域内随机位置，避免机器人路径冲突
            margin = 5  # 边缘留白
            x = random.uniform(self.top_left[0] + margin, self.bottom_right[0] - margin)
            y = random.uniform(self.top_left[1] + margin, self.bottom_right[1] - margin)
            material.location = (x, y)

            # 更新区域拥堵因子
            self.busy_factor = len(self.materials) / self.capacity
            return True
        return False

    def remove_material(self, material):
        """从区域移除材料，返回是否成功移除"""
        if material in self.materials:
            self.materials.remove(material)
            # 更新区域拥堵因子
            self.busy_factor = len(self.materials) / self.capacity if self.capacity > 0 else 0
            return True
        return False

    def get_material_by_type(self, material_type, count=1):
        """获取指定类型的材料，返回材料列表"""
        available_materials = [m for m in self.materials if m.type == material_type]
        return available_materials[:count]

    def get_center(self):
        """返回区域中心点坐标"""
        return ((self.top_left[0] + self.bottom_right[0]) / 2,
                (self.top_left[1] + self.bottom_right[1]) / 2)

    def update_materials_status(self, current_time, time_step):
        """更新区域内所有材料状态"""
        for material in self.materials:
            material.update_waiting_time(time_step)
        self.last_update = current_time

    def __repr__(self):
        return f"Area({self.id}, {self.type}, materials={len(self.materials)}/{self.capacity}, busy={self.busy_factor:.2f})"


class Robot:
    def __init__(self, robot_id, position, speed=1.5, capacity=1):
        self.id = robot_id
        self.position = position
        self.speed = speed
        self.capacity = capacity
        self.carrying = []  # 正在运输的材料列表
        self.status = "idle"  # idle, moving, loading, unloading, waiting
        self.destination = None
        self.path = []
        self.current_task = None
        self.total_distance = 0
        self.completed_tasks = 0
        self.busy_time = 0
        self.idle_time = 0
        self.last_status_change = 0
        self.assigned_region = None  # 指定机器人负责的区域
        self.task_history = []  # 任务历史记录
        self.path_efficiency = 1.0  # 路径效率系数，用于评估和调整路径规划
        self.consecutive_failures = 0  # 连续失败次数，用于死锁探测
        self.maintenance_time = 0  # 维护时间

    def is_available(self):
        """检查机器人是否可用于新任务"""
        return (self.status == "idle" and
                len(self.carrying) < self.capacity and
                self.maintenance_time == 0)

    def move_to(self, destination, current_time, grid_map, optimize=True):
        """移动到目标位置，返回是否成功启动移动"""
        if self.maintenance_time > 0:
            return False

        self.destination = destination
        self.status = "moving"
        self.last_status_change = current_time

        # 使用A*算法计算路径
        self.path = astar_path(grid_map, self.position, destination, optimize=optimize)

        if len(self.path) > 0:
            logger.debug(f"机器人 {self.id} 开始移动，目标: {destination}, 路径长度: {len(self.path)}")
            self.consecutive_failures = 0
            return True
        else:
            logger.warning(f"机器人 {self.id} 无法找到到达 {destination} 的路径")
            self.status = "idle"
            self.consecutive_failures += 1
            return False

    def update_position(self, current_time, time_step=1):
        """更新机器人位置，返回是否到达目的地"""
        if self.maintenance_time > 0:
            # 减少维护时间
            actual_maintenance = min(time_step, self.maintenance_time)
            self.maintenance_time -= actual_maintenance
            if self.maintenance_time == 0:
                logger.info(f"机器人 {self.id} 完成维护，恢复工作")
            return False

        if self.status == "moving" and len(self.path) > 0:
            # 计算本次时间步可移动的距离
            distance_per_step = self.speed * time_step
            total_distance_moved = 0

            while self.path and total_distance_moved < distance_per_step:
                next_pos = self.path[0]
                # 计算到下一点的距离
                dx = next_pos[0] - self.position[0]
                dy = next_pos[1] - self.position[1]
                distance_to_next = math.sqrt(dx * dx + dy * dy)

                if distance_to_next <= (distance_per_step - total_distance_moved):
                    # 可以到达下一点
                    self.position = next_pos
                    self.path.pop(0)
                    total_distance_moved += distance_to_next
                    self.total_distance += distance_to_next
                else:
                    # 只能部分前进
                    ratio = (distance_per_step - total_distance_moved) / distance_to_next
                    self.position = (
                        self.position[0] + dx * ratio,
                        self.position[1] + dy * ratio
                    )
                    self.total_distance += (distance_per_step - total_distance_moved)
                    total_distance_moved = distance_per_step

            # 检查是否到达目的地
            if not self.path:
                self.status = "idle"
                self.busy_time += (current_time - self.last_status_change)
                self.last_status_change = current_time
                logger.debug(f"机器人 {self.id} 到达目的地 {self.destination}")
                return True
        elif self.status == "idle":
            self.idle_time += time_step

        return False

    def pick_material(self, material, current_time):
        """拾取材料，返回是否成功拾取"""
        if self.maintenance_time > 0:
            return False

        if len(self.carrying) < self.capacity:
            self.carrying.append(material)
            material.status = "in_transit"
            self.status = "loading"
            self.busy_time += (current_time - self.last_status_change)
            self.last_status_change = current_time
            logger.debug(f"机器人 {self.id} 拾取材料 {material.id} ({material.type})")
            return True
        return False

    def drop_material(self, area, current_time):
        """放下材料，返回是否成功放下"""
        if self.maintenance_time > 0:
            return False

        if self.carrying and not area.is_full():
            success = True
            for material in self.carrying:
                if not area.add_material(material):
                    success = False
                    break

            if success:
                logger.debug(f"机器人 {self.id} 放下 {len(self.carrying)} 个材料到区域 {area.id} ({area.type})")
                self.completed_tasks += len(self.carrying)
                self.carrying = []
                self.status = "idle"
                self.busy_time += (current_time - self.last_status_change)
                self.last_status_change = current_time
                return True
            else:
                # 放置失败，区域满了
                logger.warning(f"机器人 {self.id} 无法放下材料，区域 {area.id} ({area.type}) 已满")
                self.consecutive_failures += 1
                return False
        return False

    def require_maintenance(self, duration=300):
        """机器人需要维护"""
        if self.status == "idle" or self.consecutive_failures > 5:
            prev_status = self.status
            self.status = "idle"
            self.maintenance_time = duration
            self.consecutive_failures = 0

            # 如果正在运输材料，返回材料到原位
            if self.carrying and prev_status != "idle":
                logger.info(f"机器人 {self.id} 进入维护，返还 {len(self.carrying)} 个正在运输的材料")
                self.carrying = []

            logger.info(f"机器人 {self.id} 进入维护状态，持续时间 {duration} 秒")
            return True
        return False

    def __repr__(self):
        status_str = self.status
        if self.maintenance_time > 0:
            status_str += f"(维护中:{self.maintenance_time}s)"
        return f"Robot({self.id}, at {self.position}, carrying={len(self.carrying)}, status={status_str})"


class Task:
    def __init__(self, task_id, task_type, priority, source, destination, materials, dependencies=None):
        self.id = task_id
        self.type = task_type  # "transport", "process", "maintenance"
        self.priority = priority  # 基础优先级
        self.source = source
        self.destination = destination
        self.materials = materials
        self.status = "pending"  # pending, assigned, in_progress, completed, failed
        self.assigned_robot = None
        self.creation_time = 0
        self.assignment_time = 0
        self.start_time = 0
        self.completion_time = 0
        self.dependencies = dependencies or []  # 依赖任务ID列表
        self.dependents = []  # 依赖此任务的任务ID列表
        self.attempt_count = 0  # 尝试执行次数
        self.timeout = None  # 任务超时时间
        self.dynamic_priority = priority  # 动态优先级，会随时间变化
        self.category = self._determine_category()  # 任务类别，用于分类统计

    def _determine_category(self):
        """确定任务类别"""
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
        """更新任务动态优先级"""
        # 基础优先级
        self.dynamic_priority = self.priority

        # 等待时间提升优先级
        waiting_time = current_time - self.creation_time
        time_factor = min(5, waiting_time / 1000)  # 控制最大提升幅度

        # 材料的额外优先级提升
        material_boost = 0
        for material in self.materials:
            material_boost = max(material_boost, material.task_priority_boost)

        # 任务尝试次数减少优先级，防止无限尝试失败任务
        attempt_penalty = self.attempt_count * 0.5

        # 任务依赖关系提升优先级
        dependency_boost = 0
        if self.dependents:
            dependency_boost = len(self.dependents) * 0.5

        # 计算最终动态优先级
        self.dynamic_priority += time_factor + material_boost + dependency_boost - attempt_penalty

        # 特定类型任务的额外优先级调整
        if self.type == "process" and any(m.type == "yellow" for m in self.materials):
            self.dynamic_priority += 1  # 提高黄桶处理任务的优先级

        if self.type == "transport" and any(m.type == "red" for m in self.materials):
            self.dynamic_priority += 2  # 提高红桶运输任务的优先级

    def check_dependencies_completed(self, completed_tasks):
        """检查依赖任务是否都已完成"""
        for dep_id in self.dependencies:
            if dep_id not in completed_tasks:
                return False
        return True

    def is_expired(self, current_time):
        """检查任务是否超时"""
        if self.timeout and current_time > self.timeout:
            return True
        return False

    def __lt__(self, other):
        return self.dynamic_priority > other.dynamic_priority  # 注意这里是倒序，高优先级先执行

    def __repr__(self):
        return f"Task({self.id}, {self.type}, priority={self.priority:.1f}→{self.dynamic_priority:.1f}, status={self.status}, attempts={self.attempt_count})"


def create_grid_map(width, height, areas, machines, obstacles=None, resolution=1.0):
    """创建网格地图，用于路径规划"""
    # 使用较低分辨率以提高性能
    scaled_width = int(width / resolution)
    scaled_height = int(height / resolution)
    grid = np.ones((scaled_width, scaled_height), dtype=bool)  # True表示可通行

    # 标记区域为可通行
    for area in areas:
        x_min, y_min = int(area.top_left[0] / resolution), int(area.top_left[1] / resolution)
        x_max, y_max = int(area.bottom_right[0] / resolution), int(area.bottom_right[1] / resolution)
        x_min = max(0, x_min)
        y_min = max(0, y_min)
        x_max = min(scaled_width - 1, x_max)
        y_max = min(scaled_height - 1, y_max)
        grid[x_min:x_max + 1, y_min:y_max + 1] = True

    # 标记机器为障碍物
    for machine in machines:
        x, y = int(machine.position[0] / resolution), int(machine.position[1] / resolution)
        radius = 2  # 机器占用半径
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < scaled_width and 0 <= ny < scaled_height:
                    grid[nx, ny] = False

    # 添加额外障碍物
    if obstacles:
        for obstacle in obstacles:
            x_min, y_min = int(obstacle[0][0] / resolution), int(obstacle[0][1] / resolution)
            x_max, y_max = int(obstacle[1][0] / resolution), int(obstacle[1][1] / resolution)
            x_min = max(0, x_min)
            y_min = max(0, y_min)
            x_max = min(scaled_width - 1, x_max)
            y_max = min(scaled_height - 1, y_max)
            grid[x_min:x_max + 1, y_min:y_max + 1] = False

    # 添加边界障碍
    border_width = 2
    grid[:border_width, :] = False
    grid[-border_width:, :] = False
    grid[:, :border_width] = False
    grid[:, -border_width:] = False

    return grid, resolution


def heuristic(a, b):
    """A*算法启发式函数：欧氏距离"""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def astar_path(grid, start, goal, optimize=True, max_iterations=10000):
    """使用A*算法找到最短路径"""
    grid_map, resolution = grid

    # 离散化起点和终点位置
    start_scaled = (int(start[0] / resolution), int(start[1] / resolution))
    goal_scaled = (int(goal[0] / resolution), int(goal[1] / resolution))

    # 检查起点和终点是否可通行，如果不可通行，找附近可通行点
    if not grid_map[start_scaled] or not grid_map[goal_scaled]:
        # 找最近的可通行点作为起点
        if not grid_map[start_scaled]:
            min_dist = float('inf')
            search_radius = 10
            for i in range(max(0, start_scaled[0] - search_radius),
                           min(grid_map.shape[0], start_scaled[0] + search_radius + 1)):
                for j in range(max(0, start_scaled[1] - search_radius),
                               min(grid_map.shape[1], start_scaled[1] + search_radius + 1)):
                    if grid_map[i, j]:
                        dist = heuristic((i, j), start_scaled)
                        if dist < min_dist:
                            min_dist = dist
                            start_scaled = (i, j)

        # 找最近的可通行点作为终点
        if not grid_map[goal_scaled]:
            min_dist = float('inf')
            search_radius = 10
            for i in range(max(0, goal_scaled[0] - search_radius),
                           min(grid_map.shape[0], goal_scaled[0] + search_radius + 1)):
                for j in range(max(0, goal_scaled[1] - search_radius),
                               min(grid_map.shape[1], goal_scaled[1] + search_radius + 1)):
                    if grid_map[i, j]:
                        dist = heuristic((i, j), goal_scaled)
                        if dist < min_dist:
                            min_dist = dist
                            goal_scaled = (i, j)

    # A*算法
    closed_set = set()
    open_set = {start_scaled}
    came_from = {}

    g_score = {start_scaled: 0}
    f_score = {start_scaled: heuristic(start_scaled, goal_scaled)}

    open_heap = [(f_score[start_scaled], start_scaled)]
    heapq.heapify(open_heap)

    # 定义移动方向：四向或八向移动
    if optimize:
        # 八向移动
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    else:
        # 四向移动，更保守
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    iterations = 0
    while open_set and iterations < max_iterations:
        iterations += 1
        current = heapq.heappop(open_heap)[1]

        if current == goal_scaled:
            # 重建路径
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()

            # 路径平滑化
            if optimize and len(path) > 2:
                path = smooth_path(path, grid_map)

            # 转换回浮点坐标
            return [(float(x * resolution), float(y * resolution)) for x, y in path]

        open_set.remove(current)
        closed_set.add(current)

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if (neighbor[0] < 0 or neighbor[0] >= grid_map.shape[0] or
                    neighbor[1] < 0 or neighbor[1] >= grid_map.shape[1]):
                continue

            if not grid_map[neighbor] or neighbor in closed_set:
                continue

            # 计算移动代价
            if dx != 0 and dy != 0:
                # 对角线移动代价为sqrt(2)
                move_cost = 1.414
            else:
                move_cost = 1.0

            tentative_g_score = g_score[current] + move_cost

            if neighbor not in open_set or tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal_scaled)

                if neighbor not in open_set:
                    open_set.add(neighbor)
                    heapq.heappush(open_heap, (f_score[neighbor], neighbor))

    # 未找到路径，返回空列表
    logger.warning(f"未找到从 {start} 到 {goal} 的路径，迭代次数: {iterations}")
    return []


def smooth_path(path, grid_map):
    """平滑路径，减少转向点"""
    if len(path) <= 2:
        return path

    smoothed_path = [path[0]]
    i = 0

    while i < len(path) - 1:
        current = smoothed_path[-1]

        # 尝试寻找可以直接到达的最远点
        farthest_visible = None
        for j in range(len(path) - 1, i, -1):
            if is_line_of_sight(current, path[j], grid_map):
                farthest_visible = path[j]
                i = j
                break

        if farthest_visible:
            smoothed_path.append(farthest_visible)
        else:
            i += 1
            if i < len(path):
                smoothed_path.append(path[i])

    return smoothed_path


def is_line_of_sight(start, end, grid_map):
    """检查两点之间是否有直线视野"""
    x0, y0 = start
    x1, y1 = end

    # 使用布莱森汉姆直线算法
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while x0 != x1 or y0 != y1:
        if not grid_map[x0, y0]:
            return False

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

        # 边界检查
        if x0 < 0 or x0 >= grid_map.shape[0] or y0 < 0 or y0 >= grid_map.shape[1]:
            return False

    return True


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
        self.task_queue = []  # 优先队列
        self.grid_map = None
        self.deadlock_count = 0
        self.last_progress_time = 0
        self.deadlock_threshold = 3000  # 3000秒无进展视为死锁
        self.completed_task_ids = set()  # 已完成任务ID集合
        self.failed_task_ids = set()  # 失败任务ID集合
        self.task_dependencies = {}  # 任务依赖关系图

        # 统计数据
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
            'material_states': [],
            'task_stats': {
                'created': 0,
                'completed': 0,
                'failed': 0,
                'by_category': defaultdict(int)
            },
            'deadlocks': [],
            'performance_snapshots': []
        }

        # 可视化数据
        self.vis_data = {
            'time': [],
            'red_count': [],
            'robot_positions': defaultdict(list),
            'material_positions': defaultdict(list),
            'task_queue_length': []
        }

        # 初始化工厂布局
        self.setup_factory()

    def setup_factory(self):
        """初始化工厂布局、机器、机器人和初始材料"""
        # 创建区域
        # 空桶存储区（最左侧区域）
        self.areas.append(Area(0, "empty_storage", (50, 50), (150, 950), 540))

        # 梳棉区
        for i in range(10):
            y_pos = 50 + i * 90
            self.areas.append(Area(1 + i * 2, "carding_waiting", (200, y_pos), (250, y_pos + 80), 2))
            self.areas.append(Area(2 + i * 2, "carding_finished", (300, y_pos), (350, y_pos + 80), 2))

        # 一并区
        for i in range(5):
            y_pos = 50 + i * 180
            self.areas.append(Area(21 + i * 2, "drawing1_waiting", (400, y_pos), (450, y_pos + 160), 12))
            self.areas.append(Area(22 + i * 2, "drawing1_finished", (500, y_pos), (550, y_pos + 160), 2))

        # 二并区
        for i in range(5):
            y_pos = 50 + i * 180
            self.areas.append(Area(31 + i * 2, "drawing2_waiting", (600, y_pos), (650, y_pos + 160), 12))
            self.areas.append(Area(32 + i * 2, "drawing2_finished", (700, y_pos), (750, y_pos + 160), 2))

        # 粗纱区
        for i in range(5):
            y_pos = 50 + i * 180
            self.areas.append(Area(41 + i * 2, "roving_waiting", (800, y_pos), (850, y_pos + 160), 6))
            self.areas.append(Area(42 + i * 2, "roving_finished", (900, y_pos), (950, y_pos + 160), 3))

        # 创建机器
        # 梳棉机
        for i in range(10):
            y_pos = 90 + i * 90
            self.machines.append(Machine(i, "carding", (275, y_pos), 4225.94))

        # 一并机
        for i in range(5):
            y_pos = 140 + i * 180
            self.machines.append(Machine(10 + i, "drawing1", (475, y_pos), 2594.75, 12))

        # 二并机
        for i in range(5):
            y_pos = 140 + i * 180
            self.machines.append(Machine(15 + i, "drawing2", (675, y_pos), 2594.75, 12))

        # 粗纱机
        for i in range(5):
            y_pos = 140 + i * 180
            self.machines.append(Machine(20 + i, "roving", (875, y_pos), 2594.75, 6))

        # 创建机器人
        robot_positions = [
            (100, 100),  # 靠近空桶存储区
            (100, 300),  # 靠近空桶存储区
            (100, 500),  # 靠近空桶存储区
            (100, 700),  # 靠近空桶存储区
            (100, 900),  # 靠近空桶存储区
            (300, 200),  # 靠近梳棉区
            (300, 600),  # 靠近梳棉区
            (500, 400),  # 靠近一并区
            (700, 600),  # 靠近二并区
            (900, 500)  # 靠近粗纱区
        ]

        for i, pos in enumerate(robot_positions):
            self.robots.append(Robot(i, pos, speed=2.0))
            self.stats['robot_distances'][i] = 0
            self.stats['robot_tasks'][i] = 0

        # 为机器人分配负责区域
        for i in range(5):
            self.robots[i].assigned_region = "empty_storage"  # 前5个机器人负责空桶存储区
        self.robots[5].assigned_region = "carding"
        self.robots[6].assigned_region = "carding"
        self.robots[7].assigned_region = "drawing1"
        self.robots[8].assigned_region = "drawing2"
        self.robots[9].assigned_region = "roving"

        # 创建初始空桶
        for i in range(540):
            material = Material(i, "empty")
            material.creation_time = 0
            self.materials.append(material)
            self.areas[0].add_material(material)

        # 创建网格地图，用于路径规划
        self.grid_map = create_grid_map(self.width, self.height, self.areas, self.machines, resolution=2.0)

        logger.info("工厂布局初始化完成")
        logger.info(
            f"创建了 {len(self.areas)} 个区域, {len(self.machines)} 台机器, {len(self.robots)} 个机器人, {len(self.materials)} 个初始材料")

    def generate_tasks(self):
        """生成各类任务，优化版本，更智能地创建和优化任务"""
        start_count = len(self.task_queue)

        # 1. 空桶到梳棉待装区任务
        for area in self.areas:
            if area.type == "carding_waiting" and not area.is_full():
                empty_area = self.areas[0]
                if not empty_area.is_empty():
                    # 计算需要的空桶数量
                    needed_count = min(area.capacity - len(area.materials), 2)  # 最多2个，避免过度填充

                    # 只有当区域几乎空了才补充
                    if len(area.materials) < area.capacity * 0.5 and needed_count > 0:
                        empty_materials = empty_area.get_material_by_type("empty", needed_count)
                        if empty_materials:
                            task_id = len(self.tasks)
                            task = Task(task_id, "transport", 10, empty_area, area, empty_materials)
                            task.creation_time = self.current_time
                            task.timeout = self.current_time + 10000  # 设置超时时间
                            self.tasks.append(task)
                            self.stats['task_stats']['created'] += 1
                            self.stats['task_stats']['by_category'][task.category] += 1
                            task.update_priority(self.current_time)
                            heapq.heappush(self.task_queue, task)

        # 2. 梳棉完成区到一并待装区任务
        for area in self.areas:
            if area.type == "drawing1_waiting" and not area.is_full():
                # 找出所有梳棉完成区的绿桶
                available_green = []
                for source_area in self.areas:
                    if source_area.type == "carding_finished" and not source_area.is_empty():
                        green_materials = source_area.get_material_by_type("green")
                        available_green.extend(green_materials)

                # 计算需要的绿桶数量，必须是6的倍数
                current_green_count = len([m for m in area.materials if m.type == "green"])
                needed_count = min(12, area.capacity - current_green_count)
                needed_count = needed_count - (needed_count % 6)  # 确保是6的倍数

                # 仅当区域空间足够且有足够的绿桶时创建任务
                if needed_count > 0 and len(available_green) >= 6:
                    # 如果有足够多的绿桶，优先移动整批次(6个)
                    batch_size = min(needed_count, len(available_green))
                    batch_size = batch_size - (batch_size % 6)  # 确保是6的倍数

                    if batch_size >= 6:
                        materials_to_move = available_green[:batch_size]
                        # 找出材料所在的源区域
                        materials_by_area = defaultdict(list)
                        for material in materials_to_move:
                            for source_area in self.areas:
                                if material in source_area.materials:
                                    materials_by_area[source_area].append(material)
                                    break

                        # 为每个源区域创建单独的任务
                        for source_area, materials in materials_by_area.items():
                            if materials:
                                task_id = len(self.tasks)
                                task = Task(task_id, "transport", 9, source_area, area, materials)
                                task.creation_time = self.current_time
                                task.timeout = self.current_time + 10000
                                self.tasks.append(task)
                                self.stats['task_stats']['created'] += 1
                                self.stats['task_stats']['by_category'][task.category] += 1
                                task.update_priority(self.current_time)
                                heapq.heappush(self.task_queue, task)

        # 3. 一并完成区到二并待装区任务
        for area in self.areas:
            if area.type == "drawing2_waiting" and not area.is_full():
                # 找出所有一并完成区的黄桶
                available_yellow = []
                for source_area in self.areas:
                    if source_area.type == "drawing1_finished" and not source_area.is_empty():
                        yellow_materials = source_area.get_material_by_type("yellow")
                        available_yellow.extend(yellow_materials)

                # 计算需要的黄桶数量，必须是6的倍数
                current_yellow_count = len([m for m in area.materials if m.type == "yellow"])
                needed_count = min(12, area.capacity - current_yellow_count)
                needed_count = needed_count - (needed_count % 6)  # 确保是6的倍数

                # 仅当区域空间足够且有足够的黄桶时创建任务
                if needed_count > 0 and len(available_yellow) >= 6:
                    # 如果有足够多的黄桶，优先移动整批次(6个)
                    batch_size = min(needed_count, len(available_yellow))
                    batch_size = batch_size - (batch_size % 6)  # 确保是6的倍数

                    if batch_size >= 6:
                        materials_to_move = available_yellow[:batch_size]
                        # 找出材料所在的源区域
                        materials_by_area = defaultdict(list)
                        for material in materials_to_move:
                            for source_area in self.areas:
                                if material in source_area.materials:
                                    materials_by_area[source_area].append(material)
                                    break

                        # 为每个源区域创建单独的任务
                        for source_area, materials in materials_by_area.items():
                            if materials:
                                task_id = len(self.tasks)
                                task = Task(task_id, "transport", 8, source_area, area, materials)
                                task.creation_time = self.current_time
                                task.timeout = self.current_time + 10000
                                self.tasks.append(task)
                                self.stats['task_stats']['created'] += 1
                                self.stats['task_stats']['by_category'][task.category] += 1
                                task.update_priority(self.current_time)
                                heapq.heappush(self.task_queue, task)

        # 4. 二并完成区到粗纱待装区任务
        for area in self.areas:
            if area.type == "roving_waiting" and not area.is_full():
                # 找出所有二并完成区的黄桶
                available_yellow = []
                for source_area in self.areas:
                    if source_area.type == "drawing2_finished" and not source_area.is_empty():
                        yellow_materials = source_area.get_material_by_type("yellow")
                        available_yellow.extend(yellow_materials)

                # 计算需要的黄桶数量，必须是6的倍数
                current_yellow_count = len([m for m in area.materials if m.type == "yellow"])
                needed_count = min(6, area.capacity - current_yellow_count)
                needed_count = needed_count - (needed_count % 6)  # 确保是6的倍数

                # 仅当区域空间足够且有足够的黄桶时创建任务
                if needed_count > 0 and len(available_yellow) >= 6:
                    # 如果有足够多的黄桶，优先移动整批次(6个)
                    batch_size = min(needed_count, len(available_yellow))
                    batch_size = batch_size - (batch_size % 6)  # 确保是6的倍数

                    if batch_size >= 6:
                        materials_to_move = available_yellow[:batch_size]
                        # 找出材料所在的源区域
                        materials_by_area = defaultdict(list)
                        for material in materials_to_move:
                            for source_area in self.areas:
                                if material in source_area.materials:
                                    materials_by_area[source_area].append(material)
                                    break

                        # 为每个源区域创建单独的任务
                        for source_area, materials in materials_by_area.items():
                            if materials:
                                task_id = len(self.tasks)
                                task = Task(task_id, "transport", 7, source_area, area, materials)
                                task.creation_time = self.current_time
                                task.timeout = self.current_time + 10000
                                self.tasks.append(task)
                                self.stats['task_stats']['created'] += 1
                                self.stats['task_stats']['by_category'][task.category] += 1
                                task.update_priority(self.current_time)
                                heapq.heappush(self.task_queue, task)

        # 5. 处理任务：梳棉机处理空桶
        for machine in self.machines:
            if machine.type == "carding" and machine.status == "idle":
                # 找出对应的梳棉待装区和完成区
                machine_idx = machine.id
                waiting_area = self.areas[1 + machine_idx * 2]
                finished_area = self.areas[2 + machine_idx * 2]

                # 如果有空桶待处理且完成区有空间，则创建处理任务
                if not waiting_area.is_empty() and not finished_area.is_full():
                    empty_materials = waiting_area.get_material_by_type("empty", 1)
                    if empty_materials:
                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 12, waiting_area, machine, empty_materials)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 10000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)

        # 6. 处理任务：一并机处理绿桶
        for machine in self.machines:
            if machine.type == "drawing1" and machine.status == "idle":
                # 找出对应的一并待装区和完成区
                machine_idx = machine.id - 10
                waiting_area = self.areas[21 + machine_idx * 2]
                finished_area = self.areas[22 + machine_idx * 2]

                # 一并处理需要6个绿桶，完成区需要有空间容纳1个黄桶
                if len(waiting_area.materials) >= 6 and not finished_area.is_full():
                    green_materials = waiting_area.get_material_by_type("green", 6)
                    if len(green_materials) == 6:
                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 11, waiting_area, machine, green_materials)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 10000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)

        # 7. 处理任务：二并机处理黄桶
        for machine in self.machines:
            if machine.type == "drawing2" and machine.status == "idle":
                # 找出对应的二并待装区和完成区
                machine_idx = machine.id - 15
                waiting_area = self.areas[31 + machine_idx * 2]
                finished_area = self.areas[32 + machine_idx * 2]

                # 二并处理需要6个黄桶，完成区需要有空间容纳1个黄桶
                if len(waiting_area.materials) >= 6 and not finished_area.is_full():
                    yellow_materials = waiting_area.get_material_by_type("yellow", 6)
                    if len(yellow_materials) == 6:
                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 11, waiting_area, machine, yellow_materials)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 10000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)

        # 8. 处理任务：粗纱机处理黄桶
        for machine in self.machines:
            if machine.type == "roving" and machine.status == "idle":
                # 找出对应的粗纱待装区和完成区
                machine_idx = machine.id - 20
                waiting_area = self.areas[41 + machine_idx * 2]
                finished_area = self.areas[42 + machine_idx * 2]

                # 粗纱处理需要6个黄桶，完成区需要有空间容纳1个红桶
                if len(waiting_area.materials) >= 6 and not finished_area.is_full():
                    yellow_materials = waiting_area.get_material_by_type("yellow", 6)
                    if len(yellow_materials) == 6:
                        task_id = len(self.tasks)
                        task = Task(task_id, "process", 11, waiting_area, machine, yellow_materials)
                        task.creation_time = self.current_time
                        task.timeout = self.current_time + 10000
                        self.tasks.append(task)
                        self.stats['task_stats']['created'] += 1
                        self.stats['task_stats']['by_category'][task.category] += 1
                        task.update_priority(self.current_time)
                        heapq.heappush(self.task_queue, task)

        # 9. 从机器运输处理完成的材料到完成区
        for machine in self.machines:
            if machine.status == "waiting_for_pickup" and machine.materials:
                # 找出对应的完成区
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

                # 如果完成区有空间，创建从机器到完成区的运输任务
                if finished_area and not finished_area.is_full():
                    task_id = len(self.tasks)
                    # 运输完成品是最高优先级任务
                    task = Task(task_id, "transport", 15, machine, finished_area, machine.materials[:])
                    task.creation_time = self.current_time
                    task.timeout = self.current_time + 5000  # 较短的超时时间
                    self.tasks.append(task)
                    self.stats['task_stats']['created'] += 1
                    self.stats['task_stats']['by_category'][task.category] += 1
                    task.update_priority(self.current_time)
                    heapq.heappush(self.task_queue, task)

        # 10. 处理机器和机器人维护任务
        for machine in self.machines:
            if machine.consecutive_failures > 5 and machine.status != "processing":
                task_id = len(self.tasks)
                task = Task(task_id, "maintenance", 20, None, machine, [])
                task.creation_time = self.current_time
                task.timeout = self.current_time + 2000
                self.tasks.append(task)
                self.stats['task_stats']['created'] += 1
                self.stats['task_stats']['by_category'][task.category] += 1
                task.update_priority(self.current_time)
                heapq.heappush(self.task_queue, task)

        for robot in self.robots:
            if robot.consecutive_failures > 5 and robot.status == "idle":
                task_id = len(self.tasks)
                task = Task(task_id, "maintenance", 20, None, robot, [])
                task.creation_time = self.current_time
                task.timeout = self.current_time + 2000
                self.tasks.append(task)
                self.stats['task_stats']['created'] += 1
                self.stats['task_stats']['by_category'][task.category] += 1
                task.update_priority(self.current_time)
                heapq.heappush(self.task_queue, task)

        # 11. 更新所有存在任务的优先级
        updated_queue = []
        while self.task_queue:
            task = heapq.heappop(self.task_queue)
            # 检查任务是否过期
            if task.is_expired(self.current_time):
                task.status = "failed"
                self.failed_task_ids.add(task.id)
                self.stats['task_stats']['failed'] += 1
                continue

            task.update_priority(self.current_time)
            updated_queue.append(task)

        # 重建任务队列
        self.task_queue = []
        for task in updated_queue:
            heapq.heappush(self.task_queue, task)

        # 记录任务队列长度变化
        new_tasks = len(self.task_queue) - start_count
        if new_tasks > 0:
            logger.debug(f"生成了 {new_tasks} 个新任务，当前任务队列长度: {len(self.task_queue)}")

    def assign_tasks(self):
        """为可用机器人分配任务，采用改进的任务分配策略"""
        # 筛选可用机器人
        available_robots = [robot for robot in self.robots if robot.is_available()]

        if not available_robots or not self.task_queue:
            return

        # 创建临时任务队列副本
        temp_queue = self.task_queue.copy()
        heapq.heapify(temp_queue)

        # 为每个可用机器人分配任务
        task_assignments = []

        for robot in available_robots:
            if not temp_queue:
                break

            # 寻找适合该机器人的任务
            best_task = None
            best_task_score = -float('inf')
            best_task_idx = -1

            # 检查最多10个任务以找到最佳匹配
            candidate_tasks = []
            for _ in range(min(10, len(temp_queue))):
                if temp_queue:
                    candidate_tasks.append(heapq.heappop(temp_queue))

            for idx, task in enumerate(candidate_tasks):
                if task.status != "pending":
                    continue

                # 计算任务与机器人的匹配度
                score = self._calculate_task_robot_match(task, robot)

                if score > best_task_score:
                    best_task_score = score
                    best_task = task
                    best_task_idx = idx

            # 将未选中的候选任务放回队列
            for idx, task in enumerate(candidate_tasks):
                if idx != best_task_idx:
                    heapq.heappush(temp_queue, task)

            # 如果找到合适任务，则分配给机器人
            if best_task and best_task_score > 0:
                best_task.status = "assigned"
                best_task.assigned_robot = robot
                best_task.assignment_time = self.current_time
                best_task.attempt_count += 1
                task_assignments.append((robot, best_task))

        # 处理分配结果
        self.task_queue = []  # 清空原队列

        # 将剩余任务放回队列
        for task in temp_queue:
            heapq.heappush(self.task_queue, task)

        # 开始执行分配的任务
        for robot, task in task_assignments:
            if task.type == "transport":
                self._start_transport_task(robot, task)
            elif task.type == "process":
                self._start_process_task(robot, task)
            elif task.type == "maintenance":
                self._start_maintenance_task(robot, task)

            # 将分配的任务放回队列(状态为assigned)
            heapq.heappush(self.task_queue, task)

    def _calculate_task_robot_match(self, task, robot):
        """计算任务与机器人的匹配度，返回匹配分数"""
        # 基础分数 = 任务动态优先级
        score = task.dynamic_priority

        # 如果机器人已经在运输材料，则不适合接新任务
        if robot.carrying:
            return -100

        # 计算机器人到任务源位置的距离
        source_pos = None
        if task.type == "transport" and task.source:
            if isinstance(task.source, Area):
                source_pos = task.source.get_center()
            elif isinstance(task.source, Machine):
                source_pos = task.source.position

        if source_pos:
            distance = math.sqrt((robot.position[0] - source_pos[0]) ** 2 +
                                 (robot.position[1] - source_pos[1]) ** 2)

            # 距离影响因子：距离越近越好
            distance_factor = 1000 / (distance + 100)  # 防止除零
            score += distance_factor

        # 考虑机器人的区域专属性
        if robot.assigned_region:
            # 判断任务是否与机器人负责区域相关
            task_region = None
            if task.type == "transport":
                if task.source and isinstance(task.source, Area):
                    task_region = task.source.type
                elif task.destination and isinstance(task.destination, Area):
                    task_region = task.destination.type
            elif task.type == "process":
                if task.destination and isinstance(task.destination, Machine):
                    task_region = task.destination.type

            # 如果任务区域与机器人负责区域匹配，增加分数
            if task_region and robot.assigned_region in task_region:
                score += 5

        # 考虑任务-机器人容量匹配
        if task.type == "transport" and len(task.materials) > robot.capacity:
            # 如果单个机器人不能完成整个任务，降低分数
            score -= 10

        # 历史表现因素：机器人的任务成功率
        if robot.completed_tasks > 0:
            success_rate = robot.completed_tasks / (robot.completed_tasks + robot.consecutive_failures)
            score += success_rate * 3

        return score

    def _start_transport_task(self, robot, task):
        """开始执行运输任务"""
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
                logger.debug(
                    f"机器人 {robot.id} 开始执行运输任务 {task.id}，从 {type(task.source).__name__} {task.source.id} 到 {type(task.destination).__name__} {task.destination.id}")
            else:
                task.status = "pending"  # 移动失败，重新放回队列
                robot.current_task = None
        else:
            task.status = "failed"
            self.failed_task_ids.add(task.id)
            self.stats['task_stats']['failed'] += 1
            robot.current_task = None

    def _start_process_task(self, robot, task):
        """开始执行处理任务"""
        source_pos = task.source.get_center()

        started = robot.move_to(source_pos, self.current_time, self.grid_map)
        if started:
            task.status = "in_progress"
            task.start_time = self.current_time
            robot.current_task = task
            logger.debug(
                f"机器人 {robot.id} 开始执行处理任务 {task.id}，从 {task.source.type} {task.source.id} 到 {task.destination.type} {task.destination.id}")
        else:
            task.status = "pending"  # 移动失败，重新放回队列
            robot.current_task = None

    def _start_maintenance_task(self, robot, task):
        """开始执行维护任务"""
        if isinstance(task.destination, Machine):
            task.destination.require_maintenance()
            task.status = "completed"
            task.completion_time = self.current_time
            self.completed_task_ids.add(task.id)
            self.stats['task_stats']['completed'] += 1
            logger.info(f"机器 {task.destination.id} ({task.destination.type}) 开始维护")
        elif isinstance(task.destination, Robot):
            task.destination.require_maintenance()
            task.status = "completed"
            task.completion_time = self.current_time
            self.completed_task_ids.add(task.id)
            self.stats['task_stats']['completed'] += 1
            logger.info(f"机器人 {task.destination.id} 开始维护")
        else:
            task.status = "failed"
            self.failed_task_ids.add(task.id)
            self.stats['task_stats']['failed'] += 1

    def process_robot_actions(self):
        """处理所有机器人的当前行动"""
        for robot in self.robots:
            if robot.status == "moving" and robot.current_task:
                # 更新机器人位置
                reached_destination = robot.update_position(self.current_time)

                if reached_destination:
                    task = robot.current_task

                    if task.type == "transport":
                        if not robot.carrying:
                            # 到达源位置，拾取材料
                            all_picked = True
                            for material in task.materials:
                                # 检查材料是否仍在源位置
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
                                # 有材料没找到，任务失败
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1
                                robot.current_task = None
                                robot.consecutive_failures += 1
                                continue

                            # 前往目的地
                            dest_pos = None
                            if isinstance(task.destination, Area):
                                dest_pos = task.destination.get_center()
                            elif isinstance(task.destination, Machine):
                                dest_pos = task.destination.position

                            if dest_pos:
                                robot.move_to(dest_pos, self.current_time, self.grid_map)
                            else:
                                # 目的地无效，任务失败
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1
                                robot.current_task = None
                                robot.consecutive_failures += 1
                        else:
                            # 到达目的地，放下材料
                            if isinstance(task.destination, Area):
                                success = robot.drop_material(task.destination, self.current_time)
                                if success:
                                    # 材料成功放置，完成任务
                                    task.status = "completed"
                                    task.completion_time = self.current_time
                                    self.completed_task_ids.add(task.id)
                                    self.stats['task_stats']['completed'] += 1
                                    robot.current_task = None
                                    robot.consecutive_failures = 0

                                    # 处理特殊情况：从机器到完成区的材料变化
                                    if isinstance(task.source, Machine):
                                        machine = task.source

                                        # 重置机器状态
                                        machine.status = "idle"
                                        machine.materials = []

                                        # 材料转换
                                        area = task.destination
                                        material_type = None

                                        if machine.type == "carding":
                                            # 空桶 -> 绿桶
                                            material_type = "green"
                                            self.stats['green_buckets_produced'] += len(robot.carrying)
                                        elif machine.type == "drawing1":
                                            # 6绿桶 -> 1黄桶
                                            material_type = "yellow"
                                            self.stats['yellow_buckets_produced'] += 1
                                        elif machine.type == "drawing2":
                                            # 6黄桶 -> 1黄桶 (保持黄色)
                                            material_type = "yellow"
                                            self.stats['yellow_buckets_produced'] += 1
                                        elif machine.type == "roving":
                                            # 6黄桶 -> 1红桶
                                            material_type = "red"
                                            self.stats['red_buckets_produced'] += 1

                                        # 清空区域当前所有材料(刚放下的)
                                        area.materials = []

                                        # 创建新材料
                                        if material_type == "green":
                                            # 每个空桶转为一个绿桶
                                            for i in range(len(task.materials)):
                                                material_id = len(self.materials)
                                                new_material = Material(material_id, material_type)
                                                new_material.creation_time = self.current_time
                                                new_material.creation_history = [task.materials[i].id]
                                                self.materials.append(new_material)
                                                area.add_material(new_material)

                                                # 记录领导时间
                                                lead_time = self.current_time - task.materials[i].creation_time
                                                self.stats['lead_times']['green'].append(lead_time)
                                        elif material_type in ["yellow", "red"]:
                                            # 批量转换，多个输入生成一个输出
                                            material_id = len(self.materials)
                                            new_material = Material(material_id, material_type)
                                            new_material.creation_time = self.current_time
                                            new_material.creation_history = [m.id for m in task.materials]
                                            self.materials.append(new_material)
                                            area.add_material(new_material)

                                            # 记录领导时间
                                            if material_type == "red":
                                                lead_time = self.current_time
                                                self.stats['lead_times']['red'].append(lead_time)
                                else:
                                    # 放置失败，重试任务
                                    task.status = "pending"
                                    task.attempt_count += 1
                                    robot.consecutive_failures += 1
                                    if task.attempt_count > 5:
                                        task.status = "failed"
                                        self.failed_task_ids.add(task.id)
                                        self.stats['task_stats']['failed'] += 1
                                    heapq.heappush(self.task_queue, task)
                                    robot.current_task = None
                            elif isinstance(task.destination, Machine):
                                # 将材料放入机器
                                machine = task.destination
                                if machine.is_available():
                                    # 将材料添加到机器
                                    for material in robot.carrying:
                                        machine.materials.append(material)
                                    robot.carrying = []

                                    # 启动机器处理
                                    success = machine.start_processing(self.current_time)
                                    if success:
                                        task.status = "completed"
                                        task.completion_time = self.current_time
                                        self.completed_task_ids.add(task.id)
                                        self.stats['task_stats']['completed'] += 1
                                        robot.current_task = None
                                        robot.consecutive_failures = 0
                                    else:
                                        task.status = "failed"
                                        self.failed_task_ids.add(task.id)
                                        self.stats['task_stats']['failed'] += 1
                                        robot.current_task = None
                                        robot.consecutive_failures += 1
                                        machine.consecutive_failures += 1
                                else:
                                    # 机器不可用，任务失败
                                    task.status = "failed"
                                    self.failed_task_ids.add(task.id)
                                    self.stats['task_stats']['failed'] += 1
                                    robot.current_task = None
                                    robot.consecutive_failures += 1

                    elif task.type == "process":
                        if not robot.carrying:
                            # 到达待处理材料区域，拾取材料
                            materials_to_pick = []
                            for material in task.materials:
                                if material in task.source.materials:
                                    materials_to_pick.append(material)
                                    task.source.remove_material(material)

                            # 拾取材料
                            all_picked = True
                            for material in materials_to_pick:
                                if not robot.pick_material(material, self.current_time):
                                    all_picked = False
                                    break

                            if not all_picked or not robot.carrying:
                                # 拾取失败，将材料放回区域
                                for material in robot.carrying:
                                    task.source.add_material(material)
                                robot.carrying = []

                                # 任务失败
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1
                                robot.current_task = None
                                robot.consecutive_failures += 1
                                continue

                            # 前往机器
                            machine_pos = task.destination.position
                            robot.move_to(machine_pos, self.current_time, self.grid_map)
                        else:
                            # 到达机器，将材料放入机器
                            machine = task.destination

                            if machine.is_available():
                                # 将材料添加到机器
                                for material in robot.carrying:
                                    machine.materials.append(material)
                                robot.carrying = []

                                # 启动机器处理
                                success = machine.start_processing(self.current_time)
                                if success:
                                    task.status = "completed"
                                    task.completion_time = self.current_time
                                    self.completed_task_ids.add(task.id)
                                    self.stats['task_stats']['completed'] += 1
                                    robot.current_task = None
                                    robot.consecutive_failures = 0
                                else:
                                    task.status = "failed"
                                    self.failed_task_ids.add(task.id)
                                    self.stats['task_stats']['failed'] += 1
                                    robot.current_task = None
                                    robot.consecutive_failures += 1
                                    machine.consecutive_failures += 1
                            else:
                                # 机器不可用，任务失败
                                # 将材料放回原区域
                                source_pos = task.source.get_center()
                                robot.move_to(source_pos, self.current_time, self.grid_map)
                                task.status = "pending"
                                task.attempt_count += 1
                                heapq.heappush(self.task_queue, task)
            else:
                # 更新非移动状态的机器人位置和状态
                robot.update_position(self.current_time, 1)

    def update_machines(self):
        """更新所有机器的状态"""
        for machine in self.machines:
            # 检查加工是否完成
            if machine.status == "processing":
                machine.check_process_completion(self.current_time)

            # 更新机器状态统计
            machine.update_machine_status(self.current_time, 1)

    def update_areas(self):
        """更新所有区域的状态"""
        for area in self.areas:
            area.update_materials_status(self.current_time, 1)

    def check_for_deadlocks(self):
        """检查并处理死锁情况"""
        red_count = sum(1 for m in self.materials if m.type == "red")
        active_robots = sum(1 for r in self.robots if r.status != "idle")
        active_machines = sum(1 for m in self.machines if m.status != "idle")

        # 检查是否有进展
        if (red_count > 0 or active_robots > 0 or active_machines > 0 or
                self.current_time - self.last_progress_time < 1000):
            self.last_progress_time = self.current_time
            return

        # 如果长时间无进展，尝试解除死锁
        if self.current_time - self.last_progress_time > self.deadlock_threshold:
            self.deadlock_count += 1
            self.last_progress_time = self.current_time

            logger.warning(f"检测到死锁 #{self.deadlock_count}，时间: {self.current_time}，开始解除...")
            self.stats['deadlocks'].append({
                'time': self.current_time,
                'red_count': red_count,
                'task_queue_length': len(self.task_queue)
            })

            # 1. 重置所有处理中的机器
            for machine in self.machines:
                if machine.status == "processing":
                    machine.status = "waiting_for_pickup"
                    machine.check_process_completion(self.current_time)
                elif machine.consecutive_failures > 3:
                    machine.require_maintenance(300)

            # 2. 重置所有卡住的机器人
            for robot in self.robots:
                if robot.status == "moving" and robot.current_task:
                    if robot.consecutive_failures > 3:
                        robot.require_maintenance(300)
                    else:
                        robot.status = "idle"
                        task = robot.current_task

                        # 返还所有正在运输的材料
                        if robot.carrying:
                            if task and task.source:
                                # 尝试将材料放回源位置
                                for material in robot.carrying:
                                    if isinstance(task.source, Area):
                                        task.source.add_material(material)
                                    elif isinstance(task.source, Machine):
                                        task.source.materials.append(material)

                            robot.carrying = []

                        # 重置任务
                        if task:
                            task.status = "pending"
                            task.attempt_count += 1
                            if task.attempt_count <= 5:
                                heapq.heappush(self.task_queue, task)
                            else:
                                task.status = "failed"
                                self.failed_task_ids.add(task.id)
                                self.stats['task_stats']['failed'] += 1

                        robot.current_task = None

            # 3. 清理超时任务
            updated_queue = []
            while self.task_queue:
                task = heapq.heappop(self.task_queue)
                if task.creation_time < self.current_time - 5000:
                    task.status = "failed"
                    self.failed_task_ids.add(task.id)
                    self.stats['task_stats']['failed'] += 1
                    continue
                updated_queue.append(task)

            # 重建任务队列
            self.task_queue = []
            for task in updated_queue:
                heapq.heappush(self.task_queue, task)

            logger.info(f"死锁解除完成，剩余任务数: {len(self.task_queue)}")

    def update_statistics(self):
        """更新系统统计信息"""
        # 更新机器人统计
        for robot in self.robots:
            self.stats['robot_distances'][robot.id] = robot.total_distance
            self.stats['robot_tasks'][robot.id] = robot.completed_tasks

        # 更新机器利用率
        for machine in self.machines:
            self.stats['machine_utilization'][machine.id] = machine.total_processing_time / max(1,
                                                                                                self.current_time) * 100

        # 记录材料计数
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

        # 定期生成性能快照
        if self.current_time % 5000 == 0:
            self.stats['performance_snapshots'].append({
                'time': self.current_time,
                'red_count': red_count,
                'robot_utilization': {r.id: (r.busy_time / max(1, self.current_time)) * 100 for r in self.robots},
                'machine_utilization': {m.id: (m.total_processing_time / max(1, self.current_time)) * 100 for m in
                                        self.machines},
                'task_queue_length': len(self.task_queue),
                'completed_tasks': len(self.completed_task_ids),
                'failed_tasks': len(self.failed_task_ids),
                'deadlocks': self.deadlock_count
            })

        # 更新可视化数据
        self.vis_data['time'].append(self.current_time)
        self.vis_data['red_count'].append(red_count)
        self.vis_data['task_queue_length'].append(len(self.task_queue))

        for robot in self.robots:
            self.vis_data['robot_positions'][robot.id].append(robot.position)

        for material in self.materials:
            if material.location:
                self.vis_data['material_positions'][material.id].append(material.location)

    def run(self, max_time=100000, target_red_buckets=15, update_interval=1000):
        """运行工厂模拟"""
        print("启动工厂自动化生产模拟系统...")
        start_time = time.time()

        self.current_time = 0
        self.last_progress_time = 0

        next_update = update_interval

        while self.current_time < max_time:
            # 执行一个时间步
            try:
                # 生成和分配任务
                self.generate_tasks()
                self.assign_tasks()

                # 处理机器人行动
                self.process_robot_actions()

                # 更新机器状态
                self.update_machines()

                # 更新区域状态
                self.update_areas()

                # 检查死锁
                self.check_for_deadlocks()

                # 更新统计信息
                self.update_statistics()

                # 检查是否达到目标
                red_count = sum(1 for m in self.materials if m.type == "red")
                if red_count >= target_red_buckets:
                    print(f"目标已达成！生产了 {red_count} 个红桶，用时 {self.current_time} 秒")
                    break

                # 打印进度更新
                if self.current_time >= next_update:
                    print(f"时间: {self.current_time}秒, 红桶数量: {red_count}/{target_red_buckets}")
                    next_update += update_interval

                # 时间递增
                self.current_time += 1

            except Exception as e:
                logger.error(f"模拟运行出错: {str(e)}", exc_info=True)
                break

        # 最终状态更新
        red_count = sum(1 for m in self.materials if m.type == "red")
        print(f"时间: {self.current_time}秒, 红桶数量: {red_count}/{target_red_buckets}")

        # 生成最终报告
        self.generate_report(time.time() - start_time)

        # 可视化结果
        try:
            self.visualize_results()
        except Exception as e:
            print(f"可视化更新错误: {str(e)}")

        return self.stats

    def generate_report(self, computation_time):
        """生成模拟报告"""
        # 计算统计数据
        red_count = sum(1 for m in self.materials if m.type == "red")
        empty_count = sum(1 for m in self.materials if m.type == "empty")
        green_count = sum(1 for m in self.materials if m.type == "green")
        yellow_count = sum(1 for m in self.materials if m.type == "yellow")

        # 理论vs实际时间
        theoretical_time = 0
        if red_count > 0:
            # 生产一个红桶的理论时间:
            # 1空桶 -> 1绿桶: 4225.94s
            # 6绿桶 -> 1黄桶: 2594.75s
            # 6黄桶 -> 1红桶: 2594.75s
            theoretical_time = 4225.94 + 2594.75 + 2594.75
            theoretical_efficiency = (theoretical_time / (self.current_time / red_count)) * 100
        else:
            theoretical_efficiency = 0.0

        # 找出瓶颈
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

        # 计算领导时间
        lead_times = self.stats['lead_times']
        avg_lead_times = {}
        for color, times in lead_times.items():
            if times:
                avg_lead_times[color] = sum(times) / len(times)
            else:
                avg_lead_times[color] = "未生产"

        # 打印报告
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

        print("\n任务统计:")
        print(f"  创建任务数: {self.stats['task_stats']['created']}")
        print(f"  完成任务数: {self.stats['task_stats']['completed']}")
        print(f"  失败任务数: {self.stats['task_stats']['failed']}")
        print(
            f"  任务成功率: {self.stats['task_stats']['completed'] / max(1, self.stats['task_stats']['created']) * 100:.2f}%")

        # 保存输出到目录
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"output_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)
        print(f"所有图表和数据已保存到目录: {output_dir}")

        # 最终总结
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
        """可视化模拟结果"""
        # 创建输出目录
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"output_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # 1. 绘制红桶生产情况
        plt.figure(figsize=(10, 6))
        plt.plot(self.vis_data['time'][::100], self.vis_data['red_count'][::100], 'r-', linewidth=2)
        plt.xlabel('时间 (秒)')
        plt.ylabel('红桶数量')
        plt.title('红桶生产情况')
        plt.grid(True)
        plt.savefig(f"{output_dir}/red_bucket_production.png")

        # 2. 绘制材料数量变化
        times = [state['time'] for state in self.stats['material_states'][::100]]
        empty_counts = [state['empty'] for state in self.stats['material_states'][::100]]
        green_counts = [state['green'] for state in self.stats['material_states'][::100]]
        yellow_counts = [state['yellow'] for state in self.stats['material_states'][::100]]
        red_counts = [state['red'] for state in self.stats['material_states'][::100]]

        plt.figure(figsize=(12, 6))
        plt.plot(times, empty_counts, 'k-', label='空桶', linewidth=2)
        plt.plot(times, green_counts, 'g-', label='绿桶', linewidth=2)
        plt.plot(times, yellow_counts, 'y-', label='黄桶', linewidth=2)
        plt.plot(times, red_counts, 'r-', label='红桶', linewidth=2)
        plt.xlabel('时间 (秒)')
        plt.ylabel('数量')
        plt.title('物料数量变化')
        plt.legend()
        plt.grid(True)
        plt.savefig(f"{output_dir}/material_counts.png")

        # 3. 绘制机器人性能对比
        robot_ids = list(self.stats['robot_tasks'].keys())
        tasks_completed = [self.stats['robot_tasks'][r_id] for r_id in robot_ids]
        distances_moved = [self.stats['robot_distances'][r_id] for r_id in robot_ids]

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

        colors = plt.cm.tab10(np.linspace(0, 1, len(robot_ids)))

        ax1.bar(robot_ids, tasks_completed, color=colors)
        ax1.set_xlabel('机器人ID')
        ax1.set_ylabel('完成任务数')
        ax1.set_title('机器人完成任务数')
        ax1.grid(True, axis='y')

        ax2.bar(robot_ids, distances_moved, color=colors)
        ax2.set_xlabel('机器人ID')
        ax2.set_ylabel('移动距离')
        ax2.set_title('机器人移动距离')
        ax2.grid(True, axis='y')

        plt.tight_layout()
        plt.savefig(f"{output_dir}/robot_performance.png")

        # 4. 绘制机器利用率
        machine_types = ['carding', 'drawing1', 'drawing2', 'roving']
        machine_type_names = {'carding': '梳棉机', 'drawing1': '一并机', 'drawing2': '二并机', 'roving': '粗纱机'}

        utilization_data = []

        for machine in self.machines:
            machine_type = machine.type
            utilization = machine.total_processing_time / max(1, self.current_time) * 100
            utilization_data.append((machine.id, machine_type, utilization))

        # 按机器类型分组
        utilization_by_type = {}
        for _, machine_type, util in utilization_data:
            if machine_type not in utilization_by_type:
                utilization_by_type[machine_type] = []
            utilization_by_type[machine_type].append(util)

        # 计算各类型机器的平均利用率
        avg_utilization = [sum(utilization_by_type.get(t, [0])) / len(utilization_by_type.get(t, [1])) for t in
                           machine_types]

        plt.figure(figsize=(10, 6))
        bars = plt.bar(range(len(machine_types)), avg_utilization,
                       color=['skyblue', 'lightgreen', 'lightyellow', 'salmon'])

        # 添加利用率数值标签
        for bar, util in zip(bars, avg_utilization):
            plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5, f"{util:.2f}%",
                     ha='center', va='bottom')

        plt.xticks(range(len(machine_types)), [machine_type_names[t] for t in machine_types])
        plt.xlabel('机器类型')
        plt.ylabel('平均利用率 (%)')
        plt.title('各类型机器平均利用率')
        plt.grid(True, axis='y')
        plt.savefig(f"{output_dir}/machine_utilization.png")

        # 5. 绘制任务队列长度变化
        plt.figure(figsize=(10, 6))
        plt.plot(self.vis_data['time'][::100], self.vis_data['task_queue_length'][::100], 'b-', linewidth=2)
        plt.xlabel('时间 (秒)')
        plt.ylabel('任务队列长度')
        plt.title('任务队列长度变化')
        plt.grid(True)
        plt.savefig(f"{output_dir}/task_queue_length.png")

        # 6. 创建工厂布局可视化
        plt.figure(figsize=(16, 16))

        # 绘制区域
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

            if 'waiting' in area.type:
                alpha = 0.3
            elif 'finished' in area.type:
                alpha = 0.7
            else:
                alpha = 0.5

            rect = patches.Rectangle((x, y), width, height, linewidth=1,
                                     edgecolor='black', facecolor=color, alpha=alpha)
            plt.gca().add_patch(rect)
            plt.text(x + width / 2, y + height / 2, area.type,
                     horizontalalignment='center', verticalalignment='center')

        # 绘制机器
        for machine in self.machines:
            x, y = machine.position
            if machine.type == 'carding':
                color = 'blue'
                marker = 'o'
            elif machine.type == 'drawing1':
                color = 'green'
                marker = 's'
            elif machine.type == 'drawing2':
                color = 'orange'
                marker = 'd'
            elif machine.type == 'roving':
                color = 'red'
                marker = '^'

            plt.scatter(x, y, c=color, s=150, edgecolor='black', zorder=2, marker=marker)
            plt.text(x, y - 20, f"{machine.type} {machine.id}",
                     horizontalalignment='center', verticalalignment='center', fontsize=8)

        # 绘制机器人最终位置
        for robot in self.robots:
            x, y = robot.position
            plt.scatter(x, y, c='black', marker='s', s=80, zorder=3)
            plt.text(x, y + 20, f"R{robot.id}", color='black',
                     horizontalalignment='center', verticalalignment='center', fontsize=10)

        # 绘制一部分材料位置
        materials_to_show = []
        for material in self.materials:
            if material.location and material.type in ['red', 'yellow']:
                materials_to_show.append(material)

        for material in materials_to_show:
            x, y = material.location
            if material.type == 'red':
                color = 'red'
            elif material.type == 'yellow':
                color = 'yellow'
            elif material.type == 'green':
                color = 'green'
            else:
                color = 'gray'

            plt.scatter(x, y, c=color, s=30, edgecolor='black', zorder=1, alpha=0.7)

        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.title('工厂布局及当前状态')
        plt.savefig(f"{output_dir}/factory_layout.png")

        # 7. 绘制死锁情况分析
        if self.stats['deadlocks']:
            deadlock_times = [d['time'] for d in self.stats['deadlocks']]
            deadlock_task_counts = [d['task_queue_length'] for d in self.stats['deadlocks']]

            plt.figure(figsize=(10, 6))
            plt.scatter(deadlock_times, deadlock_task_counts, c='red', s=80, marker='x')
            plt.xlabel('时间 (秒)')
            plt.ylabel('死锁时任务队列长度')
            plt.title('死锁发生时间点分析')
            plt.grid(True)
            plt.savefig(f"{output_dir}/deadlock_analysis.png")

        # 8. 绘制性能快照变化趋势
        if self.stats['performance_snapshots']:
            snapshot_times = [s['time'] for s in self.stats['performance_snapshots']]
            red_counts = [s['red_count'] for s in self.stats['performance_snapshots']]
            completed_tasks = [s['completed_tasks'] for s in self.stats['performance_snapshots']]

            plt.figure(figsize=(12, 8))
            plt.subplot(2, 1, 1)
            plt.plot(snapshot_times, red_counts, 'r-', marker='o')
            plt.xlabel('时间 (秒)')
            plt.ylabel('红桶数量')
            plt.title('生产进度快照')
            plt.grid(True)

            plt.subplot(2, 1, 2)
            plt.plot(snapshot_times, completed_tasks, 'b-', marker='s')
            plt.xlabel('时间 (秒)')
            plt.ylabel('完成任务数')
            plt.title('任务完成情况快照')
            plt.grid(True)

            plt.tight_layout()
            plt.savefig(f"{output_dir}/performance_snapshots.png")

        # 保存统计数据到CSV
        with open(f"{output_dir}/simulation_stats.txt", 'w') as f:
            f.write("===== 工厂模拟统计报告 =====\n")
            f.write(f"总模拟时间: {self.current_time} 秒\n")
            f.write(f"红桶生产数量: {sum(1 for m in self.materials if m.type == 'red')}\n")
            f.write(f"死锁检测次数: {self.deadlock_count}\n")

            f.write("\n物料数量统计:\n")
            f.write(f"  空桶: {sum(1 for m in self.materials if m.type == 'empty')}\n")
            f.write(f"  绿桶: {sum(1 for m in self.materials if m.type == 'green')}\n")
            f.write(f"  黄桶: {sum(1 for m in self.materials if m.type == 'yellow')}\n")
            f.write(f"  红桶: {sum(1 for m in self.materials if m.type == 'red')}\n")

            f.write("\n机器人统计:\n")
            for robot in self.robots:
                f.write(f"  机器人 {robot.id}:\n")
                f.write(f"    完成任务数: {robot.completed_tasks}\n")
                f.write(f"    移动距离: {robot.total_distance:.2f}\n")
                f.write(f"    利用率: {(robot.busy_time / max(1, self.current_time)) * 100:.2f}%\n")

            f.write("\n机器利用率:\n")
            for machine in self.machines:
                utilization = machine.total_processing_time / max(1, self.current_time) * 100
                f.write(f"  {machine.type} {machine.id}: {utilization:.2f}%\n")

            f.write("\n任务统计:\n")
            f.write(f"  创建任务数: {self.stats['task_stats']['created']}\n")
            f.write(f"  完成任务数: {self.stats['task_stats']['completed']}\n")
            f.write(f"  失败任务数: {self.stats['task_stats']['failed']}\n")
            success_rate = self.stats['task_stats']['completed'] / max(1, self.stats['task_stats']['created']) * 100
            f.write(f"  任务成功率: {success_rate:.2f}%\n")

            f.write("\n任务类型统计:\n")
            for category, count in self.stats['task_stats']['by_category'].items():
                f.write(f"  {category}: {count}\n")


def main():
    """主函数，创建并运行模拟"""
    # 创建并运行模拟
    simulation = FactorySimulation()
    simulation.run(max_time=100000, target_red_buckets=15, update_interval=1000)


if __name__ == "__main__":
    main()