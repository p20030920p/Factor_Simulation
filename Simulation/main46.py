# 导入必要的库
from multiprocessing import Array, Value
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass, field
import numpy as np
import threading
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor, wait
import time
import random
import json
import matplotlib.pyplot as plt
from collections import defaultdict
from scipy.optimize import linear_sum_assignment


# 1. 系统配置类
class SimConfig:
    """系统配置类"""
    # 地图相关配置
    MAP_SIZE = 1000  # 地图大小 (1000x1000)
    GRID_SIZE = 10  # 网格大小，用于路径规划

    # 机器人相关配置
    NUM_ROBOTS = 10  # 机器人总数
    ROBOT_SIZE = 10  # 机器人尺寸
    ROBOT_SPEED = 1  # 机器人移动速度(格/时间单位)
    ROBOT_TURNING_RADIUS = 2  # 转弯半径

    # 任务相关配置
    MAX_TASKS = 1000  # 最大任务数
    INITIAL_TASKS = 45  # 初始任务数
    TASK_TYPES = 3  # 任务类型数量
    TASK_BUFFER_SIZE = 100  # 任务缓冲区大小

    # 时间相关配置
    TIME_UNIT = 1  # 基本时间单位(ms)
    MAX_SIMULATION_TIME = 360000  # 最大模拟时间
    UPDATE_INTERVAL = 5  # 状态更新间隔

    # 性能相关配置
    METRICS_BUFFER_SIZE = 1000  # 性能指标缓冲区大小
    REPORT_INTERVAL = 100  # 报告生成间隔

    # 机器设备位置配置
    MACHINE_POSITIONS = {
        'combing': 150,  # 梳棉机x坐标
        'drawing1': 350,  # 一并条x坐标
        'drawing2': 550,  # 二并条x坐标
        'roving': 750  # 粗纱机x坐标
    }

    # 机器数量配置
    MACHINE_COUNTS = {
        'combing': 10,  # 梳棉机数量
        'drawing1': 5,  # 一并条数量
        'drawing2': 5,  # 二并条数量
        'roving': 5  # 粗纱机数量
    }

    # 区域类型定义
    AREA_TYPES = {
        'empty': 0,
        'boundary': 1,
        'material_warehouse': 2,
        'product_warehouse': 3,
        'charging': 4,
        'combing': 5,
        'drawing1': 6,
        'drawing2': 7,
        'roving': 8,
        'robot': 9,
        'combing_pickup': 15,
        'drawing1_pickup': 16,
        'drawing2_pickup': 17,
        'roving_pickup': 18
    }

    # 算法相关配置
    PRIORITY_WEIGHTS = {
        'waiting_time': 0.4,  # 等待时间权重
        'urgency': 0.3,  # 紧急程度权重
        'load_balance': 0.2,  # 负载均衡权重
        'chain': 0.1  # 任务链权重
    }

    # 优化相关配置
    OPTIMIZATION = {
        'replan_threshold': 3,  # 重规划阈值
        'load_threshold': 0.8,  # 负载阈值
        'chain_threshold': 0.7  # 任务链优化阈值
    }


# 2. 基础数据结构
@dataclass
class Position:
    """位置类"""
    x: int
    y: int

    def __iter__(self):
        yield self.x
        yield self.y

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Position(self.x - other.x, self.y - other.y)

    def manhattan_distance(self, other) -> int:
        """计算曼哈顿距离"""
        return abs(self.x - other.x) + abs(self.y - other.y)


@dataclass
class TaskParams:
    """任务参数类"""
    unload_time: float = 0.0
    full_rate: float = 0.0
    empty_rate: float = 0.0
    load_time: float = 0.0
    priority_factor: float = 1.0


@dataclass
class TaskStatus:
    """任务状态类"""
    created_time: int
    assigned_time: int = 0
    start_time: int = 0
    completion_time: int = 0
    current_state: str = 'created'
    assigned_robot: int = 0
    attempts: int = 0


class AtomicCounter:
    """原子计数器类"""

    def __init__(self, initial_value: int = 0):
        self._value = Value('i', initial_value)

    def increment(self) -> int:
        with self._value.get_lock():
            self._value.value += 1
            return self._value.value

    def decrement(self) -> int:
        with self._value.get_lock():
            self._value.value -= 1
            return self._value.value

    @property
    def value(self) -> int:
        return self._value.value


class FactoryLayout:
    """工厂布局类"""

    def __init__(self):
        self.map_matrix = np.zeros((SimConfig.MAP_SIZE, SimConfig.MAP_SIZE))

    def create_layout(self):
        """创建工厂布局"""
        self._set_boundaries()
        self._set_machine_areas()
        self._set_storage_areas()
        self._set_charging_areas()

    def _set_boundaries(self):
        """设置边界"""
        self.map_matrix[0, :] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[-1, :] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[:, 0] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[:, -1] = SimConfig.AREA_TYPES['boundary']

    def _set_machine_areas(self):
        """设置设备区域"""
        for machine_type, x_pos in SimConfig.MACHINE_POSITIONS.items():
            area_type = SimConfig.AREA_TYPES[machine_type]
            count = SimConfig.MACHINE_COUNTS[machine_type]
            spacing = SimConfig.MAP_SIZE // (count + 1)

            for i in range(count):
                y_pos = (i + 1) * spacing
                self._set_machine_area(x_pos, y_pos, area_type)

    def _set_machine_area(self, x: int, y: int, area_type: int):
        """设置单个设备区域"""
        size = 20
        x_start = max(0, x - size // 2)
        x_end = min(SimConfig.MAP_SIZE, x + size // 2)
        y_start = max(0, y - size // 2)
        y_end = min(SimConfig.MAP_SIZE, y + size // 2)

        self.map_matrix[y_start:y_end, x_start:x_end] = area_type

    def _set_storage_areas(self):
        """设置仓储区域"""
        self.map_matrix[100:300, 50:100] = SimConfig.AREA_TYPES['material_warehouse']
        self.map_matrix[100:300, SimConfig.MAP_SIZE - 100:SimConfig.MAP_SIZE - 50] = SimConfig.AREA_TYPES[
            'product_warehouse']

    def _set_charging_areas(self):
        """设置充电区域"""
        self.map_matrix[SimConfig.MAP_SIZE - 150:SimConfig.MAP_SIZE - 100, 50:100] = SimConfig.AREA_TYPES['charging']

    def get_map_matrix(self) -> np.ndarray:
        """获取地图矩阵"""
        return self.map_matrix.copy()


class LockFreeTask:
    """无锁任务类"""

    def __init__(self, task_id: int, task_type: int,
                 start: Position, end: Position, params: TaskParams):
        self.id = task_id
        self.type = task_type
        self.start = start
        self.end = end
        self.params = params
        self._status = Value('i', 0)  # 0:pending, 1:assigned, 2:executing, 3:completed
        self._priority = Value('d', 0.0)
        self._assigned_to = Value('i', 0)
        self._timestamps = Array('d', [0.0] * 4)  # [created, assigned, started, completed]
        self.waiting_time = 0.0
        self.execution_time = 0.0

    def try_assign(self, robot_id: int, current_time: float) -> bool:
        """尝试分配任务给机器人"""
        with self._status.get_lock():
            if self._status.value == 0:  # pending
                self._status.value = 1
                self._assigned_to.value = robot_id
                self._timestamps[1] = current_time
                return True
        return False

    def update_priority(self, new_priority: float) -> bool:
        """更新任务优先级"""
        with self._priority.get_lock():
            self._priority.value = new_priority
            return True

    @property
    def priority(self) -> float:
        """获取当前优先级"""
        return self._priority.value

    @property
    def status(self) -> int:
        """获取当前状态"""
        return self._status.value


class LockFreeTaskQueue:
    """无锁任务队列"""

    def __init__(self, capacity: int = SimConfig.TASK_BUFFER_SIZE):
        self._capacity = capacity
        self._buffer = Array('i', capacity)
        self._head = Value('i', 0)
        self._tail = Value('i', 0)
        self._size = Value('i', 0)

    def try_enqueue(self, task_id: int) -> bool:
        """尝试将任务加入队列"""
        with self._size.get_lock():
            if self._size.value >= self._capacity:
                return False

            tail = self._tail.value
            self._buffer[tail % self._capacity] = task_id
            self._tail.value = (tail + 1) % self._capacity
            self._size.value += 1
            return True

    def try_dequeue(self) -> Optional[int]:
        """尝试从队列中取出任务"""
        with self._size.get_lock():
            if self._size.value == 0:
                return None

            head = self._head.value
            task_id = self._buffer[head % self._capacity]
            self._head.value = (head + 1) % self._capacity
            self._size.value -= 1
            return task_id

    @property
    def size(self) -> int:
        """获取当前队列大小"""
        return self._size.value

    @property
    def is_empty(self) -> bool:
        """检查队列是否为空"""
        return self.size == 0

    @property
    def is_full(self) -> bool:
        """检查队列是否已满"""
        return self.size >= self._capacity


class TaskManager:
    """任务管理器"""

    def __init__(self):
        self._tasks = {}
        self._pending_queue = LockFreeTaskQueue()
        self._processing_queue = LockFreeTaskQueue()
        self._completed_queue = LockFreeTaskQueue()
        self._task_counter = AtomicCounter()
        self._task_chains = {}
        self._total_plans = AtomicCounter()
        self._successful_plans = AtomicCounter()

    def create_task(self, task_type: int, start: Position, end: Position,
                    params: TaskParams) -> Optional[int]:
        """创建新任务"""
        task_id = self._task_counter.increment()
        task = LockFreeTask(task_id, task_type, start, end, params)

        # 存储任务
        self._tasks[task_id] = task

        # 记录创建时间
        task._timestamps[0] = time.time()

        # 加入待处理队列
        if not self._pending_queue.try_enqueue(task_id):
            self._task_counter.decrement()
            return None

        return task_id

    def get_task(self, task_id: int) -> Optional[LockFreeTask]:
        """获取指定ID的任务"""
        return self._tasks.get(task_id)

    def get_next_task(self) -> Optional[LockFreeTask]:
        """获取下一个待处理任务"""
        task_id = self._pending_queue.try_dequeue()
        if task_id is None:
            return None
        return self._tasks.get(task_id)

    def get_completed_tasks(self) -> List[LockFreeTask]:
        """获取已完成任务列表"""
        completed = []
        task_id = self._completed_queue.try_dequeue()
        while task_id is not None:
            task = self._tasks.get(task_id)
            if task:
                completed.append(task)
            task_id = self._completed_queue.try_dequeue()
        return completed

    def get_total_tasks(self) -> int:
        """获取总任务数"""
        return self._task_counter.value

    def get_pending_tasks(self) -> List[LockFreeTask]:
        """获取待处理任务列表"""
        pending = []
        for task in self._tasks.values():
            if task.status == 0:  # pending
                pending.append(task)
        return pending

    def get_total_plans(self) -> int:
        """获取总规划次数"""
        return self._total_plans.value

    def get_successful_plans(self) -> int:
        """获取成功规划次数"""
        return self._successful_plans.value

    def complete_task(self, task_id: int) -> bool:
        """完成任务"""
        task = self._tasks.get(task_id)
        if task is None:
            return False

        with task._status.get_lock():
            if task._status.value == 2:  # executing
                task._status.value = 3  # completed
                task._timestamps[3] = time.time()
                task.execution_time = task._timestamps[3] - task._timestamps[2]
                task.waiting_time = task._timestamps[1] - task._timestamps[0]
                return self._completed_queue.try_enqueue(task_id)
        return False

    def return_task_to_pending(self, task_id: int) -> bool:
        """将任务返回到待处理队列"""
        task = self._tasks.get(task_id)
        if task and task.status != 0:
            with task._status.get_lock():
                task._status.value = 0
                task._assigned_to.value = 0
                return self._pending_queue.try_enqueue(task_id)
        return False

    def update_priorities(self, current_time: float):
        """更新所有待处理任务的优先级"""
        for task in self._tasks.values():
            if task.status == 0:  # pending
                priority = self._calculate_priority(task, current_time)
                task.update_priority(priority)

    def _calculate_priority(self, task: LockFreeTask, current_time: float) -> float:
        """计算任务优先级"""
        # 基础优先级因子
        base_priority = task.params.priority_factor

        # 等待时间因子
        waiting_time = current_time - task._timestamps[0]
        wait_factor = min(1.0, waiting_time / 100.0)

        # 任务类型因子
        type_factor = 1.0
        if task.type == 2:  # 一并->二并
            type_factor = 1.1
        elif task.type == 3:  # 二并->粗纱
            type_factor = 1.2

        # 任务链因子
        chain_factor = 1.2 if task.id in self._task_chains else 1.0

        # 计算最终优先级
        priority = (base_priority *
                    SimConfig.PRIORITY_WEIGHTS['waiting_time'] * wait_factor *
                    type_factor * chain_factor)

        return round(priority, 3)


class RobotState:
    """机器人状态枚举"""
    IDLE = 0
    MOVING = 1
    WORKING = 2
    CHARGING = 3
    BLOCKED = 4
    ERROR = 5
    RETURNING = 6
    WAITING = 7


@dataclass
class RobotStats:
    """机器人统计信息"""

    def __init__(self):
        self.total_distance = Value('d', 0.0)
        self.completed_tasks = Value('i', 0)
        self.total_working_time = Value('d', 0.0)
        self.total_idle_time = Value('d', 0.0)
        self.collision_count = Value('i', 0)
        self.error_count = Value('i', 0)
        self.battery_cycles = Value('i', 0)

    def update_distance(self, distance: float):
        with self.total_distance.get_lock():
            self.total_distance.value += distance

    def increment_completed_tasks(self):
        with self.completed_tasks.get_lock():
            self.completed_tasks.value += 1
class LockFreeRobot:
    """无锁机器人类"""
    def __init__(self, robot_id: int, initial_position: Position):
        self.id = robot_id
        self._position = Array('i', [initial_position.x, initial_position.y])
        self._state = Value('i', RobotState.IDLE)
        self._current_task = Value('i', 0)
        self._battery = Value('d', 100.0)
        self._path_index = Value('i', 0)
        self._last_update_time = Value('d', 0.0)

        # 路径存储
        self._path_buffer_size = 1000
        self._path = Array('i', [0] * (self._path_buffer_size * 2))
        self._path_length = Value('i', 0)

        # 统计信息
        self.stats = RobotStats()

        # 目标位置
        self._target = Array('i', [0, 0])

        # 错误处理
        self._error_code = Value('i', 0)
        self._retry_count = Value('i', 0)

    def get_position(self) -> Position:
        """获取当前位置"""
        return Position(self._position[0], self._position[1])

    def try_update_position(self, new_pos: Position) -> bool:
        """尝试更新位置"""
        with self._position.get_lock():
            old_pos = Position(self._position[0], self._position[1])
            self._position[0] = new_pos.x
            self._position[1] = new_pos.y

            # 更新移动距离
            distance = old_pos.manhattan_distance(new_pos)
            self.stats.update_distance(distance)
            return True

    def try_update_state(self, new_state: int, current_time: float) -> bool:
        """尝试更新状态"""
        with self._state.get_lock():
            old_state = self._state.value
            if self._is_valid_state_transition(old_state, new_state):
                self._state.value = new_state
                self._update_timing_stats(old_state, new_state, current_time)
                return True
        return False

    def _is_valid_state_transition(self, old_state: int, new_state: int) -> bool:
        """检查状态转换是否有效"""
        valid_transitions = {
            RobotState.IDLE: [RobotState.MOVING, RobotState.CHARGING],
            RobotState.MOVING: [RobotState.WORKING, RobotState.BLOCKED, RobotState.IDLE],
            RobotState.WORKING: [RobotState.MOVING, RobotState.IDLE, RobotState.ERROR],
            RobotState.BLOCKED: [RobotState.MOVING, RobotState.IDLE],
            RobotState.CHARGING: [RobotState.IDLE],
            RobotState.ERROR: [RobotState.IDLE],
            RobotState.RETURNING: [RobotState.CHARGING, RobotState.MOVING],
            RobotState.WAITING: [RobotState.MOVING, RobotState.IDLE]
        }
        return new_state in valid_transitions.get(old_state, [])

    def _update_timing_stats(self, old_state: int, new_state: int, current_time: float):
        """更新时间统计"""
        elapsed = current_time - self._last_update_time.value

        if old_state == RobotState.IDLE:
            with self.stats.total_idle_time.get_lock():
                self.stats.total_idle_time.value += elapsed
        elif old_state == RobotState.WORKING:
            with self.stats.total_working_time.get_lock():
                self.stats.total_working_time.value += elapsed

        self._last_update_time.value = current_time

    def set_path(self, path: List[Position]) -> bool:
        """设置规划路径"""
        if len(path) * 2 > self._path_buffer_size:
            return False

        with self._path_length.get_lock():
            # 清空当前路径
            self._path_index.value = 0
            self._path_length.value = len(path)

            # 写入新路径
            for i, pos in enumerate(path):
                self._path[i * 2] = pos.x
                self._path[i * 2 + 1] = pos.y
            return True

    def get_next_path_position(self) -> Optional[Position]:
        """获取下一个路径点"""
        with self._path_index.get_lock():
            if self._path_index.value >= self._path_length.value:
                return None

            idx = self._path_index.value * 2
            pos = Position(self._path[idx], self._path[idx + 1])
            self._path_index.value += 1
            return pos

    def update_battery(self, consumption: float) -> float:
        """更新电池电量"""
        with self._battery.get_lock():
            self._battery.value = max(0.0, min(100.0, self._battery.value - consumption))
            return self._battery.value

    def needs_charging(self) -> bool:
        """检查是否需要充电"""
        return self._battery.value < 20.0

    def is_idle(self) -> bool:
        """检查是否空闲"""
        return self._state.value == RobotState.IDLE

    def has_error(self) -> bool:
        """检查是否有错误"""
        return self._error_code.value != 0

    def clear_error(self):
        """清除错误状态"""
        self._error_code.value = 0
        self._retry_count.value = 0

    def reset(self):
        """重置机器人状态"""
        with self._state.get_lock():
            self._state.value = RobotState.IDLE
            self._current_task.value = 0
            self._path_index.value = 0
            self._path_length.value = 0
            self.clear_error()

    def complete_current_task(self):
        """完成当前任务"""
        self._current_task.value = 0
        self.stats.increment_completed_tasks()
        self.try_update_state(RobotState.IDLE, time.time())

@dataclass(order=True)
class PlanningNode:
    """A* 规划节点"""
    priority: float
    position: Position = field(compare=False)
    g_cost: float = field(compare=False)
    h_cost: float = field(compare=False)
    parent: Optional['PlanningNode'] = field(compare=False, default=None)
    timestamp: float = field(compare=False, default_factory=time.time)

    def f_cost(self) -> float:
        """获取总代价"""
        return self.g_cost + self.h_cost

class CollisionChecker:
    """碰撞检测器"""
    def __init__(self):
        self._occupancy_grid = Array('i', (SimConfig.MAP_SIZE * SimConfig.MAP_SIZE))
        self._dynamic_obstacles = Array('i', (SimConfig.MAP_SIZE * SimConfig.MAP_SIZE))
        self._robot_positions = {}
        self._lock = threading.Lock()

        self.ROBOT_RADIUS = SimConfig.ROBOT_SIZE // 2
        self.SAFETY_MARGIN = 2

    def update_static_map(self, map_matrix: np.ndarray):
        """更新静态地图"""
        with self._lock:
            for y in range(SimConfig.MAP_SIZE):
                for x in range(SimConfig.MAP_SIZE):
                    self._occupancy_grid[y * SimConfig.MAP_SIZE + x] = map_matrix[y, x]

    def update_robot_position(self, robot_id: int, position: Position):
        """更新机器人位置"""
        with self._lock:
            old_pos = self._robot_positions.get(robot_id)
            if old_pos:
                self._clear_robot_area(old_pos)
            self._mark_robot_area(position)
            self._robot_positions[robot_id] = position

    def _mark_robot_area(self, position: Position):
        """标记机器人占用区域"""
        radius = self.ROBOT_RADIUS + self.SAFETY_MARGIN
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    x, y = position.x + dx, position.y + dy
                    if 0 <= x < SimConfig.MAP_SIZE and 0 <= y < SimConfig.MAP_SIZE:
                        self._dynamic_obstacles[y * SimConfig.MAP_SIZE + x] = 1

    def _clear_robot_area(self, position: Position):
        """清除机器人占用区域"""
        radius = self.ROBOT_RADIUS + self.SAFETY_MARGIN
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    x, y = position.x + dx, position.y + dy
                    if 0 <= x < SimConfig.MAP_SIZE and 0 <= y < SimConfig.MAP_SIZE:
                        self._dynamic_obstacles[y * SimConfig.MAP_SIZE + x] = 0

    def check_collision(self, position: Position) -> bool:
        """检查指定位置是否有碰撞"""
        if not (0 <= position.x < SimConfig.MAP_SIZE and 0 <= position.y < SimConfig.MAP_SIZE):
            return True

        if self._occupancy_grid[position.y * SimConfig.MAP_SIZE + position.x] == 1:
            return True

        if self._dynamic_obstacles[position.y * SimConfig.MAP_SIZE + position.x] == 1:
            return True

        return False

class PathPlanner:
    """路径规划器"""
    def __init__(self):
        self._collision_checker = None
        self._planning_grid = np.zeros((SimConfig.MAP_SIZE, SimConfig.MAP_SIZE))
        self._path_cache = {}

        self.DIRECTIONS = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, -1), (-1, 1)
        ]

        self.DIRECTION_COSTS = [
            1.0, 1.0, 1.0, 1.0,
            1.414, 1.414, 1.414, 1.414
        ]

    def set_map(self, map_matrix: np.ndarray):
        """设置地图"""
        self._planning_grid = map_matrix.copy()


class PathPlanner(PathPlanner):  # 继续之前的PathPlanner类
    def plan_path(self, start: Position, goal: Position,
                  collision_checker: CollisionChecker) -> Optional[List[Position]]:
        """规划路径"""
        self._collision_checker = collision_checker

        # 检查起点和终点
        if (self._collision_checker.check_collision(start) or
                self._collision_checker.check_collision(goal)):
            return None

        # 检查缓存
        cache_key = (start.x, start.y, goal.x, goal.y)
        if cache_key in self._path_cache:
            cached_path = self._path_cache[cache_key]
            if not self._collision_checker.check_path_collision(cached_path):
                return cached_path

        # A*搜索
        path = self._a_star_search(start, goal)

        # 更新缓存
        if path:
            self._path_cache[cache_key] = path

        return path

    def _a_star_search(self, start: Position, goal: Position) -> Optional[List[Position]]:
        """A*搜索算法"""
        open_set = []
        closed_set = set()

        # 创建起始节点
        start_node = PlanningNode(
            priority=0,
            position=start,
            g_cost=0,
            h_cost=self._heuristic(start, goal)
        )

        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            if current.position.x == goal.x and current.position.y == goal.y:
                return self._reconstruct_path(current)

            closed_set.add((current.position.x, current.position.y))

            for i, (dx, dy) in enumerate(self.DIRECTIONS):
                new_x = current.position.x + dx
                new_y = current.position.y + dy

                if not (0 <= new_x < SimConfig.MAP_SIZE and 0 <= new_y < SimConfig.MAP_SIZE):
                    continue

                if (new_x, new_y) in closed_set:
                    continue

                new_pos = Position(new_x, new_y)

                if self._collision_checker.check_collision(new_pos):
                    continue

                new_g_cost = current.g_cost + self.DIRECTION_COSTS[i]

                new_node = PlanningNode(
                    priority=new_g_cost + self._heuristic(new_pos, goal),
                    position=new_pos,
                    g_cost=new_g_cost,
                    h_cost=self._heuristic(new_pos, goal),
                    parent=current
                )

                # 检查是否已经在开放列表中
                existing_node = None
                for node in open_set:
                    if (node.position.x == new_x and
                            node.position.y == new_y and
                            node.g_cost <= new_g_cost):
                        existing_node = node
                        break

                if existing_node is None:
                    heapq.heappush(open_set, new_node)

        return None

    def _heuristic(self, pos: Position, goal: Position) -> float:
        """启发式函数"""
        dx = pos.x - goal.x
        dy = pos.y - goal.y
        return (dx * dx + dy * dy) ** 0.5

    def _reconstruct_path(self, node: PlanningNode) -> List[Position]:
        """重建路径"""
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return list(reversed(path))

    def smooth_path(self, path: List[Position]) -> List[Position]:
        """平滑路径"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_index = 0

        while current_index < len(path) - 1:
            far_index = len(path) - 1
            while far_index > current_index:
                if self._is_line_free(path[current_index], path[far_index]):
                    smoothed.append(path[far_index])
                    current_index = far_index
                    break
                far_index -= 1

            if far_index == current_index:
                current_index += 1
                if current_index < len(path):
                    smoothed.append(path[current_index])

        return smoothed

    def _is_line_free(self, start: Position, end: Position) -> bool:
        """检查两点之间的直线是否无障碍"""
        dx = end.x - start.x
        dy = end.y - start.y
        steps = max(abs(dx), abs(dy))

        if steps == 0:
            return True

        x_step = dx / steps
        y_step = dy / steps

        x = start.x
        y = start.y

        for _ in range(int(steps)):
            pos = Position(int(x), int(y))
            if self._collision_checker.check_collision(pos):
                return False
            x += x_step
            y += y_step

        return True


class RobotManager:
    """机器人管理器"""

    def __init__(self, num_robots: int):
        self.robots = {}
        self.collision_checker = CollisionChecker()
        self.path_planner = PathPlanner()

        # 初始化机器人
        self._initialize_robots(num_robots)

    def _initialize_robots(self, num_robots: int):
        """初始化指定数量的机器人"""
        for i in range(num_robots):
            initial_pos = self._calculate_initial_position(i)
            robot = LockFreeRobot(i + 1, initial_pos)
            self.robots[i + 1] = robot

    def _calculate_initial_position(self, index: int) -> Position:
        """计算机器人初始位置"""
        row = index // 5
        col = index % 5
        x = SimConfig.MAP_SIZE - 150 + col * 30
        y = SimConfig.MAP_SIZE - 150 + row * 30
        return Position(x, y)

    def get_idle_robots(self) -> List[int]:
        """获取所有空闲机器人ID"""
        return [r_id for r_id, robot in self.robots.items() if robot.is_idle()]

    def assign_task(self, robot_id: int, task: LockFreeTask) -> bool:
        """为机器人分配任务"""
        robot = self.robots.get(robot_id)
        if not robot or not robot.is_idle():
            return False

        path = self.path_planner.plan_path(
            robot.get_position(),
            task.start,
            self.collision_checker
        )

        if not path:
            return False

        if not robot.set_path(path):
            return False

        if not robot.try_update_state(RobotState.MOVING, time.time()):
            return False

        return True

    def update_robot_states(self, current_time: float):
        """更新所有机器人状态"""
        for robot in self.robots.values():
            self._update_single_robot(robot, current_time)

    def _update_single_robot(self, robot: LockFreeRobot, current_time: float):
        """更新单个机器人状态"""
        if robot.has_error():
            self._handle_robot_error(robot)
            return

        state = robot._state.value

        if state == RobotState.MOVING:
            self._handle_moving_robot(robot)
        elif state == RobotState.WORKING:
            self._handle_working_robot(robot, current_time)
        elif state == RobotState.CHARGING:
            self._handle_charging_robot(robot)

        if robot.needs_charging() and state not in [RobotState.CHARGING, RobotState.RETURNING]:
            self._initiate_charging(robot)

    def _handle_moving_robot(self, robot: LockFreeRobot):
        """处理移动中的机器人"""
        next_pos = robot.get_next_path_position()
        if not next_pos:
            return

        if self.collision_checker.check_collision(next_pos):
            robot.try_update_state(RobotState.BLOCKED, time.time())
            return

        robot.try_update_position(next_pos)
        robot.update_battery(0.1)

    def _handle_working_robot(self, robot: LockFreeRobot, current_time: float):
        """处理工作中的机器人"""
        task_id = robot._current_task.value
        if task_id == 0:
            robot.try_update_state(RobotState.IDLE, current_time)
            return

        robot.update_battery(0.2)

    def _handle_charging_robot(self, robot: LockFreeRobot):
        """处理充电中的机器人"""
        with robot._battery.get_lock():
            if robot._battery.value < 100.0:
                robot._battery.value = min(100.0, robot._battery.value + 1.0)
            else:
                robot.try_update_state(RobotState.IDLE, time.time())

    def _handle_robot_error(self, robot: LockFreeRobot):
        """处理机器人错误"""
        with robot._retry_count.get_lock():
            if robot._retry_count.value < 3:
                robot._retry_count.value += 1
                robot.try_update_state(RobotState.IDLE, time.time())
                robot.clear_error()
            else:
                robot.try_update_state(RobotState.ERROR, time.time())

    def _initiate_charging(self, robot: LockFreeRobot):
        """初始化充电过程"""
        charging_station = self._find_nearest_charging_station(robot.get_position())

        path = self.path_planner.plan_path(
            robot.get_position(),
            charging_station,
            self.collision_checker
        )

        if path:
            robot.set_path(path)
            robot.try_update_state(RobotState.RETURNING, time.time())

    def _find_nearest_charging_station(self, position: Position) -> Position:
        """找到最近的充电站"""
        # 简化版本，返回固定位置
        return Position(50, 50)


class PerformanceMetrics:
    """性能指标类"""

    def __init__(self):
        # 任务指标
        self.task_metrics = {
            'completed_tasks': Value('i', 0),
            'average_wait_time': Value('d', 0.0),
            'average_execution_time': Value('d', 0.0),
            'task_success_rate': Value('d', 0.0),
            'type_distribution': Array('i', [0, 0, 0]),  # 三种任务类型的完成数量
        }

        # 机器人指标
        self.robot_metrics = {
            'total_distance': Value('d', 0.0),
            'average_utilization': Value('d', 0.0),
            'collision_count': Value('i', 0),
            'battery_efficiency': Value('d', 0.0),
            'idle_time_percentage': Value('d', 0.0)
        }

        # 系统指标
        self.system_metrics = {
            'system_load': Value('d', 0.0),
            'planning_success_rate': Value('d', 0.0),
            'average_response_time': Value('d', 0.0),
            'resource_utilization': Value('d', 0.0)
        }

        # 历史数据存储
        self.history_size = 1000
        self.metrics_history = {
            'timestamps': Array('d', [0.0] * self.history_size),
            'task_completion_rate': Array('d', [0.0] * self.history_size),
            'system_load': Array('d', [0.0] * self.history_size),
            'robot_utilization': Array('d', [0.0] * self.history_size)
        }
        self.history_index = Value('i', 0)


class PerformanceMonitor:
    """性能监控器"""

    def __init__(self):
        self.metrics = PerformanceMetrics()
        self.visualization = VisualizationManager()
        self._start_time = time.time()
        self._update_interval = 1.0
        self._last_update = self._start_time

    def initialize(self):
        """初始化监控器"""
        self.visualization.initialize()

    def update(self, task_manager, robot_manager, current_time: float):
        """更新性能指标"""
        if current_time - self._last_update < self._update_interval:
            return

        self._update_task_metrics(task_manager)
        self._update_robot_metrics(robot_manager)
        self._update_system_metrics(task_manager, robot_manager)
        self._record_history(current_time)
        self._last_update = current_time

    def _update_task_metrics(self, task_manager):
        """更新任务相关指标"""
        completed_tasks = task_manager.get_completed_tasks()
        total_tasks = task_manager.get_total_tasks()

        with self.metrics.task_metrics['completed_tasks'].get_lock():
            self.metrics.task_metrics['completed_tasks'].value = len(completed_tasks)

        if completed_tasks:
            wait_times = [task.waiting_time for task in completed_tasks]
            exec_times = [task.execution_time for task in completed_tasks]

            avg_wait = sum(wait_times) / len(wait_times)
            avg_exec = sum(exec_times) / len(exec_times)
            success_rate = len(completed_tasks) / total_tasks

            self.metrics.task_metrics['average_wait_time'].value = avg_wait
            self.metrics.task_metrics['average_execution_time'].value = avg_exec
            self.metrics.task_metrics['task_success_rate'].value = success_rate

            type_counts = [0, 0, 0]
            for task in completed_tasks:
                type_counts[task.type - 1] += 1
            for i in range(3):
                self.metrics.task_metrics['type_distribution'][i] = type_counts[i]

    def _update_robot_metrics(self, robot_manager):
        """更新机器人相关指标"""
        total_distance = 0
        total_utilization = 0
        total_collisions = 0
        total_battery = 0
        total_idle_time = 0
        robot_count = len(robot_manager.robots)

        for robot in robot_manager.robots.values():
            total_distance += robot.stats.total_distance.value
            working_time = robot.stats.total_working_time.value
            total_time = time.time() - self._start_time
            utilization = working_time / total_time if total_time > 0 else 0
            total_utilization += utilization
            total_collisions += robot.stats.collision_count.value
            total_battery += robot._battery.value
            idle_time = robot.stats.total_idle_time.value
            total_idle_time += idle_time / total_time if total_time > 0 else 0

        with self.metrics.robot_metrics['total_distance'].get_lock():
            self.metrics.robot_metrics['total_distance'].value = total_distance
            self.metrics.robot_metrics['average_utilization'].value = total_utilization / robot_count
            self.metrics.robot_metrics['collision_count'].value = total_collisions
            self.metrics.robot_metrics['battery_efficiency'].value = total_battery / robot_count
            self.metrics.robot_metrics['idle_time_percentage'].value = total_idle_time / robot_count

    def _update_system_metrics(self, task_manager, robot_manager):
        """更新系统整体指标"""
        pending_tasks = len(task_manager.get_pending_tasks())
        total_capacity = len(robot_manager.robots) * 2
        system_load = min(1.0, pending_tasks / total_capacity)

        total_plans = task_manager.get_total_plans()
        successful_plans = task_manager.get_successful_plans()
        planning_success_rate = successful_plans / total_plans if total_plans > 0 else 1.0

        self.metrics.system_metrics['system_load'].value = system_load
        self.metrics.system_metrics['planning_success_rate'].value = planning_success_rate

    def _record_history(self, current_time: float):
        """记录历史数据"""
        idx = self.metrics.history_index.value % self.metrics.history_size

        self.metrics.metrics_history['timestamps'][idx] = current_time
        self.metrics.metrics_history['task_completion_rate'][idx] = (
            self.metrics.task_metrics['task_success_rate'].value
        )
        self.metrics.metrics_history['system_load'][idx] = (
            self.metrics.system_metrics['system_load'].value
        )
        self.metrics.metrics_history['robot_utilization'][idx] = (
            self.metrics.robot_metrics['average_utilization'].value
        )

        self.metrics.history_index.value += 1

    def record_task_completion(self, task: LockFreeTask):
        """记录任务完成"""
        with self.metrics.task_metrics['completed_tasks'].get_lock():
            self.metrics.task_metrics['completed_tasks'].value += 1

    def generate_report(self) -> dict:
        """生成性能报告"""
        return {
            'task_metrics': {
                'completed_tasks': self.metrics.task_metrics['completed_tasks'].value,
                'average_wait_time': self.metrics.task_metrics['average_wait_time'].value,
                'average_execution_time': self.metrics.task_metrics['average_execution_time'].value,
                'task_success_rate': self.metrics.task_metrics['task_success_rate'].value,
                'type_distribution': list(self.metrics.task_metrics['type_distribution'])
            },
            'robot_metrics': {
                'total_distance': self.metrics.robot_metrics['total_distance'].value,
                'average_utilization': self.metrics.robot_metrics['average_utilization'].value,
                'collision_count': self.metrics.robot_metrics['collision_count'].value,
                'battery_efficiency': self.metrics.robot_metrics['battery_efficiency'].value,
                'idle_time_percentage': self.metrics.robot_metrics['idle_time_percentage'].value
            },
            'system_metrics': {
                'system_load': self.metrics.system_metrics['system_load'].value,
                'planning_success_rate': self.metrics.system_metrics['planning_success_rate'].value
            }
        }


class VisualizationManager:
    """可视化管理器"""

    def __init__(self):
        self._fig = None
        self._axes = {}
        self._animation = None
        self._last_update = 0
        self._update_interval = 100  # 毫秒

    def initialize(self):
        """初始化可视化界面"""
        plt.ion()
        self._fig = plt.figure(figsize=(15, 10))

        self._axes['map'] = self._fig.add_subplot(221)
        self._axes['metrics'] = self._fig.add_subplot(222)
        self._axes['robot_status'] = self._fig.add_subplot(223)
        self._axes['task_progress'] = self._fig.add_subplot(224)

        plt.tight_layout()

    def update(self, system_state, performance_metrics):
        """更新可视化显示"""
        current_time = time.time() * 1000
        if current_time - self._last_update < self._update_interval:
            return

        self._update_map(system_state)
        self._update_metrics(performance_metrics)
        self._update_robot_status(system_state)
        self._update_task_progress(system_state)

        plt.draw()
        plt.pause(0.001)
        self._last_update = current_time

    def _update_map(self, system_state):
        """更新地图显示"""
        ax = self._axes['map']
        ax.clear()

        ax.imshow(system_state.map_matrix, cmap='tab20')

        robot_positions = [(r.position.x, r.position.y) for r in system_state.robots]
        x, y = zip(*robot_positions)
        ax.scatter(x, y, c='red', marker='s', label='Robots')

        for task in system_state.active_tasks:
            ax.plot([task.start.x, task.end.x],
                    [task.start.y, task.end.y],
                    'g--', alpha=0.5)

        ax.set_title('Factory Layout')
        ax.grid(True)

    def _update_metrics(self, metrics):
        """更新性能指标显示"""
        ax = self._axes['metrics']
        ax.clear()

        times = metrics.metrics_history['timestamps'][:metrics.history_index.value]
        values = metrics.metrics_history['system_load'][:metrics.history_index.value]

        ax.plot(times, values, label='System Load')
        ax.set_title('Performance Metrics')
        ax.set_ylim(0, 1)
        ax.grid(True)
        ax.legend()

    def _update_robot_status(self, system_state):
        """更新机器人状态显示"""
        ax = self._axes['robot_status']
        ax.clear()

        status_counts = defaultdict(int)
        for robot in system_state.robots:
            status_counts[robot.state] += 1

        labels = list(status_counts.keys())
        sizes = list(status_counts.values())
        ax.pie(sizes, labels=labels, autopct='%1.1f%%')
        ax.set_title('Robot Status Distribution')

    def _update_task_progress(self, system_state):
        """更新任务进度显示"""
        ax = self._axes['task_progress']
        ax.clear()

        task_counts = {
            'Pending': len(system_state.task_queues.pending),
            'Assigned': len(system_state.task_queues.assigned),
            'Completed': len(system_state.task_queues.completed)
        }

        x = range(len(task_counts))
        ax.bar(x, task_counts.values())
        ax.set_xticks(x)
        ax.set_xticklabels(task_counts.keys())
        ax.set_title('Task Progress')
        ax.grid(True)

    def save_visualization(self, filename: str):
        """保存可视化结果"""
        if self._fig:
            self._fig.savefig(filename)

    def close(self):
        """关闭可视化窗口"""
        if self._fig:
            plt.close(self._fig)


class SystemManager:
    """系统管理器"""

    def __init__(self):
        # 初始化各个子系统
        self.task_manager = TaskManager()
        self.robot_manager = RobotManager(SimConfig.NUM_ROBOTS)
        self.performance_monitor = PerformanceMonitor()
        self.collision_checker = CollisionChecker()
        self.path_planner = PathPlanner()

        # 系统状态
        self.global_time = Value('d', 0.0)
        self.is_running = Value('b', False)
        self.status = Value('i', 0)  # 0: 停止, 1: 运行, 2: 暂停, 3: 错误

        # 事件队列
        self.event_queue = Queue()

        # 线程池
        self.executor = ThreadPoolExecutor(max_workers=4)

    def initialize(self):
        """初始化系统"""
        try:
            print("Initializing system...")

            # 初始化地图
            self._initialize_map()

            # 初始化机器人
            self._initialize_robots()

            # 生成初始任务
            self._generate_initial_tasks()

            # 初始化监控和可视化
            self.performance_monitor.initialize()

            print("System initialized successfully")
            return True
        except Exception as e:
            print(f"Initialization failed: {str(e)}")
            return False

    def _initialize_map(self):
        """初始化地图"""
        factory_layout = FactoryLayout()
        factory_layout.create_layout()

        self.collision_checker.update_static_map(factory_layout.get_map_matrix())
        self.path_planner.set_map(factory_layout.get_map_matrix())

    def _initialize_robots(self):
        """初始化机器人"""
        robot_positions = self._calculate_robot_positions()
        for pos in robot_positions:
            self.robot_manager.create_robot(pos)

    def _calculate_robot_positions(self) -> List[Position]:
        """计算机器人初始位置"""
        positions = []
        start_x = SimConfig.MAP_SIZE - 150
        start_y = SimConfig.MAP_SIZE - 150

        for i in range(SimConfig.NUM_ROBOTS):
            row = i // 5
            col = i % 5
            x = start_x + col * 30
            y = start_y + row * 30
            positions.append(Position(x, y))

        return positions

    def _generate_initial_tasks(self):
        """生成初始任务集"""
        for _ in range(SimConfig.INITIAL_TASKS):
            task_type = random.randint(1, 3)
            self._create_task(task_type)

    def _create_task(self, task_type: int):
        """创建新任务"""
        start_pos, end_pos = self._get_task_positions(task_type)
        params = self._generate_task_params(task_type)
        self.task_manager.create_task(task_type, start_pos, end_pos, params)

    def _get_task_positions(self, task_type: int) -> Tuple[Position, Position]:
        """获取任务的起点和终点位置"""
        if task_type == 1:  # 梳棉->一并
            start = self._get_random_position_near_machine('combing')
            end = self._get_random_position_near_machine('drawing1')
        elif task_type == 2:  # 一并->二并
            start = self._get_random_position_near_machine('drawing1')
            end = self._get_random_position_near_machine('drawing2')
        else:  # 二并->粗纱
            start = self._get_random_position_near_machine('drawing2')
            end = self._get_random_position_near_machine('roving')

        return start, end

    def _get_random_position_near_machine(self, machine_type: str) -> Position:
        """获取机器附近的随机位置"""
        x = SimConfig.MACHINE_POSITIONS[machine_type]
        y = random.randint(100, SimConfig.MAP_SIZE - 100)
        return Position(x, y)

    def _generate_task_params(self, task_type: int) -> TaskParams:
        """生成任务参数"""
        return TaskParams(
            unload_time=random.uniform(1.0, 3.0),
            full_rate=random.uniform(0.8, 1.0),
            empty_rate=random.uniform(0.1, 0.3),
            load_time=random.uniform(1.0, 3.0),
            priority_factor=random.uniform(0.8, 1.2)
        )

    def run(self):
        """运行系统"""
        try:
            self.is_running.value = True
            print("Starting system...")

            # 启动子线程
            self._start_background_threads()

            # 主循环
            while self.is_running.value:
                self._main_loop()

            print("System stopped normally")
        except Exception as e:
            print(f"System error: {str(e)}")
            self.status.value = 3
        finally:
            self._cleanup()

    def _start_background_threads(self):
        """启动后台线程"""
        self.executor.submit(self._monitor_thread)
        self.executor.submit(self._task_generation_thread)
        self.executor.submit(self._event_handling_thread)

    def _main_loop(self):
        """主循环"""
        current_time = time.time()
        self.global_time.value = current_time

        self.task_manager.update_priorities(current_time)
        self._assign_tasks()
        self._update_robots()
        self._check_completion()

        time.sleep(0.01)

    def _assign_tasks(self):
        """分配任务"""
        idle_robots = self.robot_manager.get_idle_robots()
        if not idle_robots:
            return

        pending_tasks = self.task_manager.get_pending_tasks()
        if not pending_tasks:
            return

        assignments = self._hungarian_assignment(idle_robots, pending_tasks)

        for robot_id, task_id in assignments:
            self._assign_single_task(robot_id, task_id)

    def _hungarian_assignment(self, robots: List[int], tasks: List[LockFreeTask]) -> List[Tuple[int, int]]:
        """使用匈牙利算法进行任务分配"""
        n = len(robots)
        m = len(tasks)
        size = max(n, m)
        cost_matrix = np.full((size, size), float('inf'))

        for i, robot_id in enumerate(robots):
            robot = self.robot_manager.robots[robot_id]
            robot_pos = robot.get_position()
            for j, task in enumerate(tasks):
                distance = robot_pos.manhattan_distance(task.start)
                cost = distance / task.priority
                cost_matrix[i][j] = cost

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        assignments = []
        for i, j in zip(row_ind, col_ind):
            if i < n and j < m and cost_matrix[i][j] != float('inf'):
                assignments.append((robots[i], tasks[j].id))

        return assignments

    def _assign_single_task(self, robot_id: int, task_id: int) -> bool:
        """分配单个任务"""
        task = self.task_manager.get_task(task_id)
        if not task:
            return False

        if task.try_assign(robot_id, time.time()):
            return self.robot_manager.assign_task(robot_id, task)

        return False

    def _update_robots(self):
        """更新机器人状态"""
        self.robot_manager.update_robot_states(time.time())

    def _check_completion(self):
        """检查任务完成情况"""
        for robot in self.robot_manager.robots.values():
            if robot._state.value == RobotState.WORKING:
                task_id = robot._current_task.value
                if self._check_task_completion(robot, task_id):
                    self._complete_task(robot, task_id)

    def _check_task_completion(self, robot: LockFreeRobot, task_id: int) -> bool:
        """检查任务是否完成"""
        if task_id == 0:
            return False

        task = self.task_manager.get_task(task_id)
        if not task:
            return False

        current_pos = robot.get_position()
        target_pos = task.end

        return current_pos.manhattan_distance(target_pos) <= 1

    def _complete_task(self, robot: LockFreeRobot, task_id: int):
        """完成任务"""
        if self.task_manager.complete_task(task_id):
            robot.complete_current_task()

    def _monitor_thread(self):
        """性能监控线程"""
        while self.is_running.value:
            try:
                current_time = time.time()
                self.performance_monitor.update(
                    self.task_manager,
                    self.robot_manager,
                    current_time
                )
                time.sleep(1)
            except Exception as e:
                print(f"Monitor thread error: {str(e)}")

    def _task_generation_thread(self):
        """任务生成线程"""
        while self.is_running.value:
            try:
                if self._should_generate_new_task():
                    task_type = random.randint(1, 3)
                    self._create_task(task_type)
                time.sleep(0.1)
            except Exception as e:
                print(f"Task generation thread error: {str(e)}")

    def _should_generate_new_task(self) -> bool:
        """检查是否应该生成新任务"""
        pending_count = len(self.task_manager.get_pending_tasks())
        return pending_count < SimConfig.NUM_ROBOTS * 2

    def _event_handling_thread(self):
        """事件处理线程"""
        while self.is_running.value:
            try:
                event = self.event_queue.get(timeout=1)
                self._handle_event(event)
            except Empty:
                continue
            except Exception as e:
                print(f"Event handling thread error: {str(e)}")

    def _handle_event(self, event: dict):
        """处理系统事件"""
        event_type = event.get('type')
        if event_type == 'robot_error':
            self._handle_robot_error(event)
        elif event_type == 'task_timeout':
            self._handle_task_timeout(event)
        elif event_type == 'system_warning':
            self._handle_system_warning(event)

    def _handle_robot_error(self, event: dict):
        """处理机器人错误事件"""
        robot_id = event.get('robot_id')
        error_code = event.get('error_code')
        robot = self.robot_manager.robots.get(robot_id)
        if robot:
            robot._error_code.value = error_code
            print(f"Robot {robot_id} error: {error_code}")

    def _handle_task_timeout(self, event: dict):
        """处理任务超时事件"""
        task_id = event.get('task_id')
        task = self.task_manager.get_task(task_id)
        if task:
            self.task_manager.return_task_to_pending(task_id)
            print(f"Task {task_id} timeout, returning to pending queue")

    def _handle_system_warning(self, event: dict):
        """处理系统警告事件"""
        warning_type = event.get('warning_type')
        warning_message = event.get('message')
        print(f"System warning: {warning_type} - {warning_message}")

    def stop(self):
        """停止系统"""
        self.is_running.value = False
        self._cleanup()

    def _cleanup(self):
        """清理资源"""
        self.executor.shutdown(wait=True)
        self._save_final_report()
        self.performance_monitor.visualization.close()

    def _save_final_report(self):
        """保存最终报告"""
        report = self.performance_monitor.generate_report()
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"system_report_{timestamp}.json"

        with open(filename, 'w') as f:
            json.dump(report, f, indent=4)

        print(f"Final report saved to {filename}")


class SystemTest:
    """系统测试框架"""

    def __init__(self):
        self.system = None
        self.test_results = []

    def run_all_tests(self):
        """运行所有测试"""
        print("Starting system tests...")

        self._test_initialization()
        self._test_task_management()
        self._test_robot_control()
        self._test_path_planning()
        self._test_performance()

        self._print_test_results()

    def _test_initialization(self):
        """测试系统初始化"""
        try:
            self.system = SystemManager()
            success = self.system.initialize()

            self._record_test_result(
                "System Initialization",
                success,
                "System initialized successfully" if success else "Initialization failed"
            )
        except Exception as e:
            self._record_test_result(
                "System Initialization",
                False,
                f"Error during initialization: {str(e)}"
            )

    def _test_task_management(self):
        """测试任务管理"""
        try:
            # 测试任务创建
            task_id = self.system.task_manager.create_task(
                1,
                Position(100, 100),
                Position(200, 200),
                TaskParams()
            )

            self._record_test_result(
                "Task Creation",
                task_id is not None,
                f"Task created with ID: {task_id}"
            )

            # 测试任务分配
            success = self.system._assign_single_task(1, task_id)

            self._record_test_result(
                "Task Assignment",
                success,
                "Task assigned successfully"
            )

        except Exception as e:
            self._record_test_result(
                "Task Management",
                False,
                f"Error in task management: {str(e)}"
            )

    def _test_robot_control(self):
        """测试机器人控制"""
        try:
            robot = self.system.robot_manager.robots[1]

            # 测试位置更新
            success = robot.try_update_position(Position(150, 150))

            self._record_test_result(
                "Robot Position Update",
                success,
                "Position updated successfully"
            )

            # 测试状态转换
            success = robot.try_update_state(RobotState.MOVING, time.time())

            self._record_test_result(
                "Robot State Transition",
                success,
                "State updated successfully"
            )

        except Exception as e:
            self._record_test_result(
                "Robot Control",
                False,
                f"Error in robot control: {str(e)}"
            )

    def _test_path_planning(self):
        """测试路径规划"""
        try:
            start = Position(100, 100)
            goal = Position(200, 200)

            path = self.system.path_planner.plan_path(
                start,
                goal,
                self.system.collision_checker
            )

            self._record_test_result(
                "Path Planning",
                path is not None,
                f"Path planned with {len(path) if path else 0} points"
            )

        except Exception as e:
            self._record_test_result(
                "Path Planning",
                False,
                f"Error in path planning: {str(e)}"
            )

    def _test_performance(self):
        """测试性能监控"""
        try:
            # 运行系统一段时间
            self.system.is_running.value = True
            time.sleep(5)

            # 获取性能报告
            report = self.system.performance_monitor.generate_report()

            self._record_test_result(
                "Performance Monitoring",
                bool(report),
                "Performance metrics collected successfully"
            )

        except Exception as e:
            self._record_test_result(
                "Performance Monitoring",
                False,
                f"Error in performance monitoring: {str(e)}"
            )
        finally:
            self.system.stop()

    def _record_test_result(self, test_name: str, success: bool, message: str):
        """记录测试结果"""
        self.test_results.append({
            'test_name': test_name,
            'success': success,
            'message': message,
            'timestamp': time.time()
        })

    def _print_test_results(self):
        """打印测试结果"""
        print("\nTest Results:")
        print("=" * 50)

        success_count = 0
        total_tests = len(self.test_results)

        for result in self.test_results:
            status = "PASS" if result['success'] else "FAIL"
            print(f"{result['test_name']}: {status}")
            print(f"Message: {result['message']}")
            print("-" * 50)

            if result['success']:
                success_count += 1

        print(f"\nSummary: {success_count}/{total_tests} tests passed")


def run_system_simulation():
    """运行系统仿真"""
    print("Starting system simulation...")

    # 创建并初始化系统
    system = SystemManager()
    if not system.initialize():
        print("System initialization failed")
        return False

    try:
        # 运行系统
        system.run()
        return True
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    except Exception as e:
        print(f"Simulation error: {str(e)}")
    finally:
        system.stop()

    return False


def main():
    """主函数"""
    # 运行测试
    test = SystemTest()
    test.run_all_tests()

    # 如果测试通过，运行实际系统
    if all(result['success'] for result in test.test_results):
        print("\nAll tests passed. Starting system simulation...")
        success = run_system_simulation()
        if success:
            print("Simulation completed successfully")
        else:
            print("Simulation failed")
    else:
        print("\nSome tests failed. Please fix the issues before running the system")


if __name__ == "__main__":
    main()