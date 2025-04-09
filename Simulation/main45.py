"""
Multi-Robot Task Assignment and Path Planning System
多机器人任务分配与路径规划系统

Features/功能:
- CBS (Conflict-Based Search) for multi-robot path planning
  基于CBS的多机器人路径规划
- Dynamic task assignment and priority management
  动态任务分配和优先级管理
- Deadlock detection and resolution
  死锁检测与解决
- Performance monitoring and optimization
  性能监控与优化
- Cost map based conflict avoidance
  基于代价地图的冲突避免

Version: 1.0
Date: 2024-01
"""

# ===== 1. Required Imports/所需导入 =====
import numpy as np          # 用于数值计算和数组操作
from dataclasses import dataclass, field  # 用于创建数据类
from typing import List, Dict, Tuple, Set, Optional, Any  # 类型提示
from collections import defaultdict  # 用于创建默认字典
import heapq               # 用于优先队列操作
import random             # 用于随机数生成
import time               # 用于时间相关操作
import matplotlib.pyplot as plt  # 用于数据可视化
from scipy.optimize import linear_sum_assignment  # 用于匈牙利算法的任务分配
import traceback  # 添加这个导入

# ===== 2. System Configuration/系统配置 =====
class SimConfig:
    """
    System Configuration Class
    系统配置类

    包含所有系统运行所需的静态配置参数：
    - 地图相关配置
    - 机器人相关配置
    - 任务相关配置
    - 时间相关配置
    - 算法相关配置
    """

    # ==== Map Configuration/地图配置 ====
    MAP_SIZE = 1000  # Size of the simulation map/仿真地图大小(1000x1000)

    # ==== Robot Configuration/机器人配置 ====
    NUM_ROBOTS = 10        # Number of robots/机器人数量
    ROBOT_RADIUS = 2       # Robot radius/机器人半径
    ROBOT_MAX_VELOCITY = 1  # Maximum robot velocity/机器人最大速度
    ROBOT_MIN_SEPARATION = 5  # Minimum separation between robots/机器人最小间距

    # ==== Task Configuration/任务配置 ====
    NUM_INITIAL_TASKS = 45  # Number of initial tasks/初始任务数量
    TASK_GENERATION_RATE = 0.1  # Rate of new task generation/新任务生成率
    MAX_TASK_PRIORITY = 10  # Maximum task priority/任务最高优先级
    MIN_TASK_PRIORITY = 1   # Minimum task priority/任务最低优先级

    # ==== Time Configuration/时间配置 ====
    MAX_SIMULATION_TIME = 360000  # Maximum simulation time/最大仿真时间
    TIME_UNIT = 1          # Time unit for simulation/仿真时间单位
    TIME_STEP = 0.1        # Time step for updates/更新时间步长

    # ==== Machine Positions/机器位置配置 ====
    MACHINE_POSITIONS = {
        'combing': 150,    # Combing machine x-coordinate/梳棉机x坐标
        'drawing1': 350,   # First drawing machine x-coordinate/一并条x坐标
        'drawing2': 550,   # Second drawing machine x-coordinate/二并条x坐标
        'roving': 750      # Roving machine x-coordinate/粗纱机x坐标
    }

    # ==== Machine Counts/机器数量配置 ====
    MACHINE_COUNTS = {
        'combing': 10,     # Number of combing machines/梳棉机数量
        'drawing1': 5,     # Number of first drawing machines/一并条数量
        'drawing2': 5,     # Number of second drawing machines/二并条数量
        'roving': 5        # Number of roving machines/粗纱机数量
    }

    # ==== Area Types/区域类型定义 ====
    AREA_TYPES = {
        'empty': 0,               # Empty area/空闲区域
        'boundary': 1,            # Boundary area/边界区域
        'material_warehouse': 2,  # Material warehouse/原料仓库
        'product_warehouse': 3,   # Product warehouse/成品仓库
        'charging': 4,            # Charging station/充电站
        'combing': 5,            # Combing area/梳棉区域
        'drawing1': 6,           # First drawing area/一并条区域
        'drawing2': 7,           # Second drawing area/二并条区域
        'roving': 8,             # Roving area/粗纱区域
        'robot': 9,              # Robot occupied area/机器人占用区域
        'combing_pickup': 15,    # Combing pickup area/梳棉取料区
        'drawing1_pickup': 16,   # First drawing pickup area/一并条取料区
        'drawing2_pickup': 17,   # Second drawing pickup area/二并条取料区
        'roving_pickup': 18      # Roving pickup area/粗纱取料区
    }

    # CBS配置（添加这些配置）
    MAX_CBS_ITERATIONS = 100    # CBS最大迭代次数
    CBS_TIMEOUT = 1000         # CBS超时时间（毫秒）
    CBS_MAX_DEPTH = 50         # CBS最大搜索深度
    CBS_NODE_LIMIT = 1000      # CBS节点数限制

    # ==== Planning Configuration/规划配置 ====
    PLANNING_HORIZON = 100  # Planning horizon/规划时域
    MAX_REPLAN_ATTEMPTS = 3  # Maximum replanning attempts/最大重规划次数
    A_STAR_WEIGHT = 1.5    # A* algorithm weight/A*算法权重
    PATH_SAFETY_MARGIN = 2  # Safety margin for path planning/路径安全边界

    # ==== Priority Configuration/优先级配置 ====
    PRIORITY_UPDATE_INTERVAL = 5  # Priority update interval/优先级更新间隔
    CHAIN_OPTIMIZATION_THRESHOLD = 15  # Task chain optimization threshold/任务链优化阈值
    PRIORITY_WAITING_WEIGHT = 0.4  # Weight for waiting time/等待时间权重
    PRIORITY_URGENCY_WEIGHT = 0.3  # Weight for urgency/紧急程度权重
    PRIORITY_LOAD_WEIGHT = 0.3    # Weight for system load/负载权重

    # ==== Deadlock Configuration/死锁配置 ====
    DEADLOCK_DETECTION_INTERVAL = 10  # Deadlock detection interval/死锁检测间隔
    DEADLOCK_TIMEOUT = 20  # Deadlock timeout threshold/死锁超时阈值
    COST_ADJUST_FACTOR = 0.5  # Cost adjustment factor/代价调整因子
    MAX_DEADLOCK_RESOLVE_ATTEMPTS = 3  # Maximum deadlock resolution attempts/最大死锁解决尝试次数

    # ==== Performance Configuration/性能配置 ====
    METRICS_UPDATE_INTERVAL = 10  # Metrics update interval/指标更新间隔
    PERFORMANCE_LOG_INTERVAL = 100  # Performance logging interval/性能日志记录间隔
    METRICS_WINDOW_SIZE = 1000  # Performance metrics window size/性能指标窗口大小

    @classmethod
    def validate_config(cls) -> bool:
        """
        Validate configuration parameters
        验证配置参数的有效性

        Returns/返回:
            bool: Whether the configuration is valid/配置是否有效
        """
        try:
            # 验证地图尺寸
            assert cls.MAP_SIZE > 0, "地图尺寸必须大于0"

            # 验证机器人参数
            assert cls.NUM_ROBOTS > 0, "机器人数量必须大于0"
            assert cls.ROBOT_RADIUS > 0, "机器人半径必须大于0"

            # 验证任务参数
            assert cls.NUM_INITIAL_TASKS > 0, "初始任务数量必须大于0"

            # 验证时间参数
            assert cls.MAX_SIMULATION_TIME > 0, "最大模拟时间必须大于0"
            assert cls.TIME_UNIT > 0, "时间单位必须大于0"

            # 验证优先级权重
            total_weight = (cls.PRIORITY_WAITING_WEIGHT +
                          cls.PRIORITY_URGENCY_WEIGHT +
                          cls.PRIORITY_LOAD_WEIGHT)
            assert abs(total_weight - 1.0) < 1e-6, "优先级权重总和必须为1"

            return True

        except AssertionError as e:
            print(f"配置验证失败: {str(e)}")
            return False


# ===== 3. Basic Data Structures/基础数据结构 =====

@dataclass
class Position:
    """
    Position class for 2D coordinates
    二维坐标位置类

    Attributes/属性:
        x: X coordinate/X坐标
        y: Y coordinate/Y坐标
    """
    x: int
    y: int

    def __iter__(self):
        """
        Make Position object iterable
        使Position对象可迭代

        Yields/生成:
            x and y coordinates/x和y坐标
        """
        yield self.x
        yield self.y

    def __add__(self, other):
        """
        Addition operator for positions
        位置加法运算符

        Args/参数:
            other: Another Position object/另一个Position对象

        Returns/返回:
            Position: New position after addition/相加后的新位置
        """
        return Position(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        """
        Subtraction operator for positions
        位置减法运算符

        Args/参数:
            other: Another Position object/另一个Position对象

        Returns/返回:
            Position: New position after subtraction/相减后的新位置
        """
        return Position(self.x - other.x, self.y - other.y)


@dataclass
class TaskSpecificParams:
    """
    Specific parameters for different task types
    不同任务类型的特定参数

    Attributes/属性:
        unload_time: Time required for unloading/卸载所需时间
        full_rate: Full load rate/满载率
        empty_rate: Empty load rate/空载率
        load_time: Time required for loading/装载所需时间
    """
    unload_time: float = 0.0  # Time for unloading/卸载时间
    full_rate: float = 0.0  # Full load rate/满载率
    empty_rate: float = 0.0  # Empty load rate/空载率
    load_time: float = 0.0  # Time for loading/装载时间


@dataclass
class Task:
    """
    Task class representing a task in the system
    系统中的任务类

    Attributes/属性:
        id: Task unique identifier/任务唯一标识符
        type: Task type (1:combing->drawing1, 2:drawing1->drawing2, 3:drawing2->roving)
              任务类型(1:梳棉->一并, 2:一并->二并, 3:二并->粗纱)
        state: Task state (new/open/assigned/completed)/任务状态
        start: Start position/起始位置
        end: End position/目标位置
        open_time: Time when task becomes open/任务开始时间
        deadline: Task deadline/任务截止时间
        priority: Task priority/任务优先级
        assigned_to: ID of assigned robot/分配的机器人ID
        assign_time: Time of assignment/分配时间
        start_time: Time when execution started/开始执行时间
        complete_time: Time of completion/完成时间
        waiting_time: Time spent waiting/等待时间
        execution_time: Time spent executing/执行时间
        total_time: Total time from open to completion/总时间
        specific: Task specific parameters/任务特定参数
        next_task_id: ID of next related task/下一个关联任务的ID
    """
    id: int
    type: int
    state: str
    start: Position
    end: Position
    open_time: int
    deadline: int
    priority: float = 0.0
    assigned_to: int = 0
    assign_time: int = 0
    start_time: int = 0
    complete_time: int = 0
    waiting_time: float = 0.0
    execution_time: float = 0.0
    total_time: float = 0.0
    specific: TaskSpecificParams = field(default_factory=TaskSpecificParams)
    next_task_id: int = 0

    def update_waiting_time(self, current_time: int) -> None:
        """
        Update task waiting time
        更新任务等待时间

        Args/参数:
            current_time: Current system time/当前系统时间
        """
        if self.state == 'open':
            self.waiting_time = current_time - self.open_time

    def is_expired(self, current_time: int) -> bool:
        """
        Check if task has expired
        检查任务是否过期

        Args/参数:
            current_time: Current system time/当前系统时间

        Returns/返回:
            bool: Whether the task has expired/任务是否过期
        """
        return current_time > self.deadline


@dataclass
class Robot:
    """
    Robot class representing a robot in the system
    系统中的机器人类

    Attributes/属性:
        id: Robot unique identifier/机器人唯一标识符
        position: Current position/当前位置
        target: Target position/目标位置
        path: Planned path/规划路径
        state: Robot state (idle/moving/working)/机器人状态
        current_task_id: Current task ID/当前任务ID
        path_index: Current position in path/当前路径索引
        last_update_time: Last update time/上次更新时间
        completed_tasks: List of completed task IDs/已完成任务ID列表
        start_position: Initial position/初始位置
    """
    id: int
    position: Position
    target: Position
    path: List[Position]
    state: str
    current_task_id: int
    path_index: int
    last_update_time: int
    completed_tasks: List[int]
    start_position: Position

    def get_task_statistics(self, tasks: List[Task]) -> Tuple[int, int, int, int, List[int]]:
        """
        Get task completion statistics for the robot
        获取机器人的任务完成统计

        Args/参数:
            tasks: List of all tasks/所有任务列表

        Returns/返回:
            Tuple containing/包含以下内容的元组:
            - Total completed tasks/完成总数
            - Type 1 tasks completed/类型1任务数
            - Type 2 tasks completed/类型2任务数
            - Type 3 tasks completed/类型3任务数
            - List of completed task IDs/已完成任务ID列表
        """
        completed_ids = self.completed_tasks
        type1_count = sum(1 for task_id in completed_ids if tasks[task_id - 1].type == 1)
        type2_count = sum(1 for task_id in completed_ids if tasks[task_id - 1].type == 2)
        type3_count = sum(1 for task_id in completed_ids if tasks[task_id - 1].type == 3)
        return (len(completed_ids), type1_count, type2_count, type3_count,
                sorted(completed_ids))

    def is_stuck(self, current_time: int, threshold: int = 30) -> bool:
        """
        Check if robot is stuck
        检查机器人是否卡住

        Args/参数:
            current_time: Current system time/当前系统时间
            threshold: Time threshold for stuck detection/卡住判定阈值

        Returns/返回:
            bool: Whether the robot is stuck/机器人是否卡住
        """
        return (self.state != 'idle' and
                current_time - self.last_update_time > threshold)


@dataclass
class TaskRecord:
    """
    Task execution record class
    任务执行记录类

    用于记录和统计任务执行的详细信息

    Attributes/属性:
        task_id: Task ID/任务ID
        robot_id: Robot ID that executed the task/执行任务的机器人ID
        start_time: Execution start time/开始执行时间
        end_time: Execution end time/结束执行时间
        waiting_time: Time spent waiting/等待时间
        path_length: Length of path taken/路径长度
        priority: Task priority when executed/执行时的优先级
        conflicts: Number of conflicts encountered/遇到的冲突数
        replans: Number of replannings needed/需要重规划的次数
    """
    task_id: int
    robot_id: int
    start_time: int
    end_time: int = 0
    waiting_time: int = 0
    path_length: int = 0
    priority: float = 0.0
    conflicts: int = 0
    replans: int = 0

    def calculate_metrics(self) -> Dict[str, float]:
        """
        Calculate performance metrics for this task execution
        计算此任务执行的性能指标

        Returns/返回:
            Dict: Dictionary containing performance metrics/包含性能指标的字典
            {
                'execution_time': Total execution time/总执行时间,
                'efficiency': Task execution efficiency/任务执行效率,
                'quality': Task execution quality/任务执行质量
            }
        """
        execution_time = self.end_time - self.start_time

        # 计算效率（考虑等待时间和路径长度）
        efficiency = 1.0
        if execution_time > 0:
            efficiency = self.path_length / execution_time

        # 计算质量（考虑冲突和重规划次数）
        quality = 1.0 - (0.1 * self.conflicts + 0.2 * self.replans)
        quality = max(0.0, quality)  # 确保质量不为负

        return {
            'execution_time': execution_time,
            'efficiency': efficiency,
            'quality': quality
        }


class TaskQueues:
    """
    Task queues management class
    任务队列管理类

    管理系统中不同状态的任务队列

    Attributes/属性:
        all: All tasks/所有任务
        open: Open (unassigned) tasks/开放（未分配）任务
        assigned: Assigned tasks/已分配任务
        executing: Tasks being executed/执行中任务
        completed: Completed tasks/已完成任务
        failed: Failed tasks/失败任务
    """

    def __init__(self):
        """Initialize task queues/初始化任务队列"""
        self.all: List[int] = []
        self.open: List[int] = []
        self.assigned: List[int] = []
        self.executing: List[int] = []
        self.completed: List[int] = []
        self.failed: List[int] = []

    def add_task(self, task_id: int) -> None:
        """
        Add a new task to queues
        添加新任务到队列

        Args/参数:
            task_id: Task ID to add/要添加的任务ID
        """
        self.all.append(task_id)
        self.open.append(task_id)

    def update_task_state(self, task_id: int, old_state: str, new_state: str) -> None:
        """
        Update task state in queues
        更新任务在队列中的状态

        Args/参数:
            task_id: Task ID/任务ID
            old_state: Previous task state/任务原状态
            new_state: New task state/任务新状态
        """
        # 从原状态队列中移除
        if old_state == 'open' and task_id in self.open:
            self.open.remove(task_id)
        elif old_state == 'assigned' and task_id in self.assigned:
            self.assigned.remove(task_id)
        elif old_state == 'executing' and task_id in self.executing:
            self.executing.remove(task_id)

        # 添加到新状态队列
        if new_state == 'assigned':
            self.assigned.append(task_id)
        elif new_state == 'executing':
            self.executing.append(task_id)
        elif new_state == 'completed':
            self.completed.append(task_id)
        elif new_state == 'failed':
            self.failed.append(task_id)


class TaskSequenceTracker:
    """
    Task sequence tracking class
    任务序列跟踪类

    跟踪任务之间的依赖关系和执行顺序

    Attributes/属性:
        task_dependencies: Task dependency graph/任务依赖图
        completion_order: Task completion order/任务完成顺序
        current_chains: Active task chains/当前活动的任务链
    """

    def __init__(self):
        """Initialize sequence tracker/初始化序列跟踪器"""
        self.task_dependencies: Dict[int, Set[int]] = defaultdict(set)
        self.completion_order: List[int] = []
        self.current_chains: Dict[int, List[int]] = {}

    def add_dependency(self, task_id: int, dependent_task_id: int) -> None:
        """
        Add a dependency between tasks
        添加任务间的依赖关系

        Args/参数:
            task_id: Task ID/任务ID
            dependent_task_id: ID of task that depends on task_id
                             依赖于task_id的任务ID
        """
        self.task_dependencies[task_id].add(dependent_task_id)

    def record_completion(self, task_id: int) -> None:
        """
        Record task completion
        记录任务完成

        Args/参数:
            task_id: Completed task ID/完成的任务ID
        """
        self.completion_order.append(task_id)

    def get_next_tasks(self, task_id: int) -> Set[int]:
        """
        Get tasks that depend on the given task
        获取依赖于给定任务的任务

        Args/参数:
            task_id: Task ID/任务ID

        Returns/返回:
            Set[int]: Set of dependent task IDs/依赖任务ID集合
        """
        return self.task_dependencies[task_id]

    def create_task_chain(self, start_task_id: int) -> List[int]:
        """
        Create a chain of related tasks
        创建相关任务链

        Args/参数:
            start_task_id: Starting task ID/起始任务ID

        Returns/返回:
            List[int]: Ordered list of task IDs in chain/任务链中的有序任务ID列表
        """
        chain = [start_task_id]
        current_task = start_task_id

        while self.task_dependencies[current_task]:
            next_tasks = self.task_dependencies[current_task]
            if len(next_tasks) > 0:
                next_task = min(next_tasks)  # 选择ID最小的任务
                chain.append(next_task)
                current_task = next_task
            else:
                break

        return chain

    def get_chain_progress(self, chain_id: int) -> Tuple[int, int]:
        """
        Get progress of a task chain
        获取任务链的进度

        Args/参数:
            chain_id: Chain ID/任务链ID

        Returns/返回:
            Tuple[int, int]: (Completed tasks, Total tasks)/（已完成任务数，总任务数）
        """
        if chain_id not in self.current_chains:
            return (0, 0)

        chain = self.current_chains[chain_id]
        completed = sum(1 for task_id in chain if task_id in self.completion_order)
        return (completed, len(chain))


@dataclass
class Constraint:
    """
    Constraint class for CBS algorithm
    CBS算法的约束类

    用于表示时空约束，防止机器人之间的碰撞

    Attributes/属性:
        robot_id: Robot affected by constraint/受约束影响的机器人ID
        position: Constrained position/约束位置
        time_step: Time step when constraint is active/约束生效的时间步
        type: Constraint type (vertex/edge)/约束类型（顶点/边）
        prev_position: Previous position (for edge constraints)/前一位置（用于边约束）
    """
    robot_id: int
    position: Position
    time_step: int
    type: str = 'vertex'  # 'vertex' or 'edge'
    prev_position: Optional[Position] = None

    def conflicts_with(self, robot_id: int, pos: Position, time: int) -> bool:
        """
        Check if this constraint conflicts with given parameters
        检查此约束是否与给定参数冲突

        Args/参数:
            robot_id: Robot ID to check/要检查的机器人ID
            pos: Position to check/要检查的位置
            time: Time to check/要检查的时间

        Returns/返回:
            bool: Whether there is a conflict/是否存在冲突
        """
        if self.robot_id != robot_id:
            return False

        if self.time_step != time:
            return False

        if self.type == 'vertex':
            return self.position.x == pos.x and self.position.y == pos.y
        else:  # edge constraint
            if not self.prev_position:
                return False
            return (self.position.x == pos.x and self.position.y == pos.y and
                    self.prev_position == pos)


@dataclass
class Conflict:
    """
    Conflict class for CBS algorithm
    CBS算法的冲突类

    表示机器人之间的路径冲突

    Attributes/属性:
        robot1_id: First robot ID/第一个机器人ID
        robot2_id: Second robot ID/第二个机器人ID
        position: Conflict position/冲突位置
        time_step: Time step of conflict/冲突时间步
        type: Conflict type (vertex/edge)/冲突类型（顶点/边）
        robot1_prev_pos: Robot 1's previous position/机器人1的前一位置
        robot2_prev_pos: Robot 2's previous position/机器人2的前一位置
    """
    robot1_id: int
    robot2_id: int
    position: Position
    time_step: int
    type: str = 'vertex'
    robot1_prev_pos: Optional[Position] = None
    robot2_prev_pos: Optional[Position] = None

    def get_constraints(self) -> Tuple[Constraint, Constraint]:
        """
        Generate constraints from this conflict
        从此冲突生成约束

        Returns/返回:
            Tuple[Constraint, Constraint]: Two constraints resolving this conflict
                                         解决此冲突的两个约束
        """
        if self.type == 'vertex':
            return (
                Constraint(self.robot1_id, self.position, self.time_step, 'vertex'),
                Constraint(self.robot2_id, self.position, self.time_step, 'vertex')
            )
        else:
            return (
                Constraint(self.robot1_id, self.position, self.time_step, 'edge',
                           self.robot1_prev_pos),
                Constraint(self.robot2_id, self.position, self.time_step, 'edge',
                           self.robot2_prev_pos)
            )


class CBSNode:
    """
    Node class for CBS search tree
    CBS搜索树的节点类

    表示CBS搜索过程中的一个状态节点

    Attributes/属性:
        constraints: Set of constraints/约束集合
        solution: Current solution paths/当前解决方案路径
        cost: Total cost of solution/解决方案总代价
        parent: Parent node/父节点
        conflicts: List of conflicts in solution/解决方案中的冲突列表
    """

    def __init__(self, constraints: Set[Constraint] = None,
                 solution: Dict[int, List[Position]] = None,
                 cost: float = 0, parent: Optional['CBSNode'] = None):
        """
        Initialize CBS node
        初始化CBS节点

        Args/参数:
            constraints: Initial constraints/初始约束
            solution: Initial solution/初始解决方案
            cost: Initial cost/初始代价
            parent: Parent node/父节点
        """
        self.constraints = constraints or set()
        self.solution = solution or {}
        self.cost = cost
        self.parent = parent
        self.conflicts: List[Conflict] = []

    def get_first_conflict(self) -> Optional[Conflict]:
        """
        Get first conflict in current solution
        获取当前解决方案中的第一个冲突

        Returns/返回:
            Optional[Conflict]: First conflict or None if no conflicts
                              第一个冲突，如果没有冲突则返回None
        """
        for t in range(max(len(path) for path in self.solution.values())):
            # 检查顶点冲突
            robots_at_t = {}
            for robot_id, path in self.solution.items():
                if t < len(path):
                    pos = path[t]
                    if pos in robots_at_t:
                        return Conflict(
                            robot_id, robots_at_t[pos],
                            pos, t, 'vertex'
                        )
                    robots_at_t[pos] = robot_id

            # 检查边冲突
            if t > 0:
                for robot1_id, path1 in self.solution.items():
                    if t >= len(path1):
                        continue
                    for robot2_id, path2 in self.solution.items():
                        if robot1_id >= robot2_id or t >= len(path2):
                            continue
                        if (path1[t] == path2[t - 1] and
                                path1[t - 1] == path2[t]):
                            return Conflict(
                                robot1_id, robot2_id,
                                path1[t], t, 'edge',
                                path1[t - 1], path2[t - 1]
                            )
        return None

    def get_solution_cost(self) -> float:
        """
        Calculate total cost of current solution
        计算当前解决方案的总代价

        Returns/返回:
            float: Total cost/总代价
        """
        return sum(len(path) for path in self.solution.values())


class CostMap:
    """
    Cost map class for path planning
    路径规划的代价地图类

    维护地图中每个位置的移动代价

    Attributes/属性:
        width: Map width/地图宽度
        height: Map height/地图高度
        base_costs: Base movement costs/基础移动代价
        dynamic_costs: Dynamic cost adjustments/动态代价调整
    """

    def __init__(self, width: int, height: int):
        """
        Initialize cost map
        初始化代价地图

        Args/参数:
            width: Map width/地图宽度
            height: Map height/地图高度
        """
        self.width = width
        self.height = height
        self.base_costs = np.ones((height, width), dtype=float)
        self.dynamic_costs = np.zeros((height, width), dtype=float)

    def update_base_cost(self, pos: Position, cost: float) -> None:
        """
        Update base cost for a position
        更新位置的基础代价

        Args/参数:
            pos: Position to update/要更新的位置
            cost: New base cost/新的基础代价
        """
        if 0 <= pos.x < self.width and 0 <= pos.y < self.height:
            self.base_costs[pos.y, pos.x] = cost

    def add_dynamic_cost(self, pos: Position, cost: float) -> None:
        """
        Add dynamic cost to a position
        添加位置的动态代价

        Args/参数:
            pos: Position to update/要更新的位置
            cost: Additional dynamic cost/额外的动态代价
        """
        if 0 <= pos.x < self.width and 0 <= pos.y < self.height:
            self.dynamic_costs[pos.y, pos.x] += cost

    def get_cost(self, pos: Position) -> float:
        """
        Get total cost for a position
        获取位置的总代价

        Args/参数:
            pos: Position to check/要检查的位置

        Returns/返回:
            float: Total cost/总代价
        """
        if 0 <= pos.x < self.width and 0 <= pos.y < self.height:
            return self.base_costs[pos.y, pos.x] + self.dynamic_costs[pos.y, pos.x]
        return float('inf')


class CBSPlanner:
    """
    Conflict-Based Search (CBS) path planner
    基于冲突的搜索路径规划器

    实现多机器人的协同路径规划算法

    Attributes/属性:
        map_matrix: Environment map matrix/环境地图矩阵
        cost_map: Cost map/代价地图
        robot_radius: Robot radius/机器人半径
        max_iterations: Maximum CBS iterations/CBS最大迭代次数
        timeout: Planning timeout/规划超时时间
    """

    def __init__(self, map_matrix: np.ndarray, robot_radius: int,
                 max_iterations: int = 100, timeout: int = 1000):
        """
        Initialize CBS planner
        初始化CBS规划器

        Args/参数:
            map_matrix: Environment map/环境地图
            robot_radius: Robot radius/机器人半径
            max_iterations: Maximum iterations/最大迭代次数
            timeout: Timeout in milliseconds/超时时间（毫秒）
        """
        self.map_matrix = map_matrix
        self.height, self.width = map_matrix.shape
        self.cost_map = CostMap(self.width, self.height)
        self.robot_radius = robot_radius
        self.max_iterations = max_iterations
        self.timeout = timeout

        # 初始化代价地图
        self._initialize_cost_map()

    def _initialize_cost_map(self) -> None:
        """
        Initialize cost map based on environment
        基于环境初始化代价地图
        """
        # 设置障碍物的基础代价为无穷大
        for y in range(self.height):
            for x in range(self.width):
                if self.map_matrix[y, x] == 1:  # 障碍物
                    self.cost_map.update_base_cost(Position(x, y), float('inf'))

        # 在障碍物周围添加安全距离代价
        for y in range(self.height):
            for x in range(self.width):
                if self.map_matrix[y, x] == 1:
                    self._add_safety_costs(Position(x, y))

    def _add_safety_costs(self, obstacle_pos: Position) -> None:
        """
        Add safety distance costs around obstacle
        在障碍物周围添加安全距离代价

        Args/参数:
            obstacle_pos: Obstacle position/障碍物位置
        """
        safety_radius = self.robot_radius + 1
        for dy in range(-safety_radius, safety_radius + 1):
            for dx in range(-safety_radius, safety_radius + 1):
                if dy == 0 and dx == 0:
                    continue

                dist = (dx ** 2 + dy ** 2) ** 0.5
                if dist > safety_radius:
                    continue

                pos = Position(obstacle_pos.x + dx, obstacle_pos.y + dy)
                safety_cost = 1.0 / (dist + 0.1)  # 避免除以零
                self.cost_map.add_dynamic_cost(pos, safety_cost)

    def _get_neighbors(self, pos: Position) -> List[Position]:
        """
        Get valid neighboring positions
        获取有效的相邻位置

        Args/参数:
            pos: Current position/当前位置

        Returns/返回:
            List[Position]: List of valid neighbors/有效相邻位置列表
        """
        neighbors = []
        # 八个方向：上、下、左、右、左上、右上、左下、右下
        directions = [(0, 1), (0, -1), (-1, 0), (1, 0),
                      (-1, 1), (1, 1), (-1, -1), (1, -1)]

        for dx, dy in directions:
            new_pos = Position(pos.x + dx, pos.y + dy)
            if (0 <= new_pos.x < self.width and
                    0 <= new_pos.y < self.height and
                    self.map_matrix[new_pos.y, new_pos.x] == 0):
                neighbors.append(new_pos)

        return neighbors

    def _a_star_search(self, start: Position, goal: Position,
                       constraints: Set[Constraint], robot_id: int) -> List[Position]:
        """
        A* search algorithm with constraints
        带约束的A*搜索算法

        Args/参数:
            start: Start position/起始位置
            goal: Goal position/目标位置
            constraints: Set of constraints/约束集合
            robot_id: Robot ID/机器人ID

        Returns/返回:
            List[Position]: Planned path or empty list if no path found
                          规划的路径，如果未找到路径则返回空列表
        """

        def heuristic(pos: Position) -> float:
            """Manhattan distance heuristic/曼哈顿距离启发式"""
            return abs(pos.x - goal.x) + abs(pos.y - goal.y)

        # 优先队列存储：(f-value, time_step, position)
        open_set = [(heuristic(start), 0, start)]
        came_from = {}
        g_score = defaultdict(lambda: float('inf'))
        g_score[(start, 0)] = 0

        while open_set:
            _, time_step, current = heapq.heappop(open_set)

            if current.x == goal.x and current.y == goal.y:
                # 重建路径
                path = []
                while time_step >= 0:
                    path.append(current)
                    if time_step == 0:
                        break
                    current = came_from[(current, time_step)]
                    time_step -= 1
                return list(reversed(path))

            # 检查邻居
            for next_pos in self._get_neighbors(current):
                # 检查约束
                constrained = False
                for constraint in constraints:
                    if (constraint.robot_id == robot_id and
                            constraint.time_step == time_step + 1):
                        if (constraint.type == 'vertex' and
                                constraint.position == next_pos):
                            constrained = True
                            break
                        elif (constraint.type == 'edge' and
                              constraint.position == next_pos and
                              constraint.prev_position == current):
                            constrained = True
                            break

                if constrained:
                    continue

                # 计算新的g值
                tentative_g = g_score[(current, time_step)] + self.cost_map.get_cost(next_pos)

                if tentative_g < g_score[(next_pos, time_step + 1)]:
                    came_from[(next_pos, time_step + 1)] = current
                    g_score[(next_pos, time_step + 1)] = tentative_g
                    f_score = tentative_g + heuristic(next_pos)
                    heapq.heappush(open_set, (f_score, time_step + 1, next_pos))

        return []  # 没找到路径

    def plan_paths(self, robots: Dict[int, Position],
                   goals: Dict[int, Position]) -> Dict[int, List[Position]]:
        """
        Plan paths for multiple robots using CBS
        使用CBS为多个机器人规划路径

        Args/参数:
            robots: Dictionary of robot positions/机器人位置字典
            goals: Dictionary of goal positions/目标位置字典

        Returns/返回:
            Dict[int, List[Position]]: Dictionary of planned paths/规划路径字典
        """
        # 创建根节点
        root = CBSNode()

        # 为每个机器人计算初始路径
        for robot_id, start_pos in robots.items():
            path = self._a_star_search(start_pos, goals[robot_id], set(), robot_id)
            if not path:
                print(f"Warning: No initial path found for robot {robot_id}")
                return {}
            root.solution[robot_id] = path

        # 更新根节点的代价
        root.cost = root.get_solution_cost()

        # 使用优先队列进行CBS搜索
        queue = [root]
        iterations = 0
        start_time = time.time()

        while queue and iterations < self.max_iterations:
            if time.time() - start_time > self.timeout / 1000:
                print("CBS planning timeout")
                return {}

            # 获取代价最小的节点
            current = heapq.heappop(queue)

            # 检查冲突
            conflict = current.get_first_conflict()
            if not conflict:
                return current.solution

            # 为每个冲突创建新的约束和节点
            constraints = conflict.get_constraints()
            for constraint in constraints:
                # 创建新节点
                new_node = CBSNode(
                    constraints=current.constraints | {constraint},
                    parent=current
                )

                # 复制当前解决方案
                new_node.solution = current.solution.copy()

                # 重新规划受影响机器人的路径
                robot_id = constraint.robot_id
                new_path = self._a_star_search(
                    robots[robot_id],
                    goals[robot_id],
                    new_node.constraints,
                    robot_id
                )

                if new_path:
                    new_node.solution[robot_id] = new_path
                    new_node.cost = new_node.get_solution_cost()
                    heapq.heappush(queue, new_node)

            iterations += 1

        print(f"CBS reached maximum iterations ({self.max_iterations})")
        return {}

    def update_dynamic_costs(self, robot_positions: Dict[int, Position]) -> None:
        """
        Update dynamic costs based on current robot positions
        基于当前机器人位置更新动态代价

        Args/参数:
            robot_positions: Current robot positions/当前机器人位置
        """
        # 重置动态代价
        self.cost_map.dynamic_costs.fill(0)

        # 为每个机器人添加动态代价
        for robot_id, pos in robot_positions.items():
            self._add_safety_costs(pos)


class TaskPriorityManager:
    """
    Task priority management system
    任务优先级管理系统

    负责动态计算和更新任务优先级

    Attributes/属性:
        tasks: Task dictionary/任务字典
        priority_weights: Priority calculation weights/优先级计算权重
        update_interval: Priority update interval/优先级更新间隔
        last_update_time: Last update timestamp/上次更新时间戳
    """

    def __init__(self, priority_weights: Optional[Dict[str, float]] = None,
                 update_interval: int = SimConfig.PRIORITY_UPDATE_INTERVAL):
        """
        Initialize task priority manager
        初始化任务优先级管理器

        Args/参数:
            priority_weights: Custom priority weights/自定义优先级权重
            update_interval: Update interval/更新间隔
        """
        self.tasks: Dict[int, Task] = {}
        self.priority_weights = priority_weights or {
            'deadline': 0.4,  # 截止时间权重
            'waiting': 0.3,  # 等待时间权重
            'chain': 0.2,  # 任务链权重
            'type': 0.1  # 任务类型权重
        }
        self.update_interval = update_interval
        self.last_update_time = 0
        self.chain_tracker = TaskSequenceTracker()

    def add_task(self, task: Task) -> None:
        """
        Add new task to priority management
        添加新任务到优先级管理

        Args/参数:
            task: Task to add/要添加的任务
        """
        self.tasks[task.id] = task
        self._calculate_single_task_priority(task)

    def remove_task(self, task_id: int) -> None:
        """
        Remove task from priority management
        从优先级管理中移除任务

        Args/参数:
            task_id: Task ID to remove/要移除的任务ID
        """
        if task_id in self.tasks:
            del self.tasks[task_id]

    def update_priorities(self, current_time: int) -> None:
        """
        Update priorities for all tasks
        更新所有任务的优先级

        Args/参数:
            current_time: Current system time/当前系统时间
        """
        if current_time - self.last_update_time < self.update_interval:
            return

        for task in self.tasks.values():
            self._calculate_single_task_priority(task, current_time)

        self.last_update_time = current_time

    def _calculate_single_task_priority(self, task: Task,
                                        current_time: Optional[int] = None) -> None:
        """
        Calculate priority for a single task
        计算单个任务的优先级

        Args/参数:
            task: Task to calculate priority for/要计算优先级的任务
            current_time: Current system time/当前系统时间
        """
        if current_time is None:
            current_time = self.last_update_time

        # 计算截止时间分数
        deadline_score = self._calculate_deadline_score(task, current_time)

        # 计算等待时间分数
        waiting_score = self._calculate_waiting_score(task, current_time)

        # 计算任务链分数
        chain_score = self._calculate_chain_score(task)

        # 计算任务类型分数
        type_score = self._calculate_type_score(task)

        # 计算加权总分
        total_score = (
                self.priority_weights['deadline'] * deadline_score +
                self.priority_weights['waiting'] * waiting_score +
                self.priority_weights['chain'] * chain_score +
                self.priority_weights['type'] * type_score
        )

        # 更新任务优先级
        task.priority = total_score

    def _calculate_deadline_score(self, task: Task, current_time: int) -> float:
        """
        Calculate deadline-based priority score
        计算基于截止时间的优先级分数

        Args/参数:
            task: Task to calculate for/要计算的任务
            current_time: Current system time/当前系统时间

        Returns/返回:
            float: Deadline score/截止时间分数
        """
        if task.deadline <= current_time:
            return 1.0

        time_remaining = task.deadline - current_time
        total_time = task.deadline - task.open_time

        if total_time <= 0:
            return 1.0

        return 1.0 - (time_remaining / total_time)

    def _calculate_waiting_score(self, task: Task, current_time: int) -> float:
        """
        Calculate waiting time-based priority score
        计算基于等待时间的优先级分数

        Args/参数:
            task: Task to calculate for/要计算的任务
            current_time: Current system time/当前系统时间

        Returns/返回:
            float: Waiting score/等待时间分数
        """
        waiting_time = current_time - task.open_time
        max_expected_wait = task.deadline - task.open_time

        if max_expected_wait <= 0:
            return 1.0

        return min(1.0, waiting_time / max_expected_wait)

    def _calculate_chain_score(self, task: Task) -> float:
        """
        Calculate task chain-based priority score
        计算基于任务链的优先级分数

        Args/参数:
            task: Task to calculate for/要计算的任务

        Returns/返回:
            float: Chain score/任务链分数
        """
        if not task.next_task_id:
            return 0.0

        chain = self.chain_tracker.create_task_chain(task.id)
        if not chain:
            return 0.0

        # 任务链中的位置越靠前，优先级越高
        position = chain.index(task.id)
        return 1.0 - (position / len(chain))

    def _calculate_type_score(self, task: Task) -> float:
        """
        Calculate task type-based priority score
        计算基于任务类型的优先级分数

        Args/参数:
            task: Task to calculate for/要计算的任务

        Returns/返回:
            float: Type score/任务类型分数
        """
        # 基于任务类型的固定优先级
        type_priorities = {
            1: 0.8,  # 梳棉->一并 (高优先级)
            2: 0.6,  # 一并->二并
            3: 0.4  # 二并->粗纱
        }
        return type_priorities.get(task.type, 0.0)

    def get_high_priority_tasks(self, threshold: float = 0.7) -> List[Task]:
        """
        Get list of high priority tasks
        获取高优先级任务列表

        Args/参数:
            threshold: Priority threshold/优先级阈值

        Returns/返回:
            List[Task]: List of high priority tasks/高优先级任务列表
        """
        return [task for task in self.tasks.values()
                if task.priority >= threshold]

    def get_task_priority_groups(self) -> Dict[str, List[Task]]:
        """
        Group tasks by priority levels
        按优先级级别对任务分组

        Returns/返回:
            Dict[str, List[Task]]: Tasks grouped by priority level
                                  按优先级级别分组的任务
        """
        groups = {
            'high': [],  # 优先级 >= 0.7
            'medium': [],  # 0.3 <= 优先级 < 0.7
            'low': []  # 优先级 < 0.3
        }

        for task in self.tasks.values():
            if task.priority >= 0.7:
                groups['high'].append(task)
            elif task.priority >= 0.3:
                groups['medium'].append(task)
            else:
                groups['low'].append(task)

        return groups

    def analyze_priority_distribution(self) -> Dict[str, float]:
        """
        Analyze priority distribution statistics
        分析优先级分布统计

        Returns/返回:
            Dict[str, float]: Priority distribution statistics
                             优先级分布统计
        """
        if not self.tasks:
            return {}

        priorities = [task.priority for task in self.tasks.values()]
        return {
            'mean': np.mean(priorities),
            'median': np.median(priorities),
            'std': np.std(priorities),
            'min': min(priorities),
            'max': max(priorities)
        }


class TaskAssignmentManager:
    """
    Task assignment management system
    任务分配管理系统

    负责任务分配策略和执行

    Attributes/属性:
        robots: Robot dictionary/机器人字典
        tasks: Task dictionary/任务字典
        priority_manager: Task priority manager/任务优先级管理器
        assignments: Current task assignments/当前任务分配
        cost_matrix: Task assignment cost matrix/任务分配代价矩阵
        planner: Path planner/路径规划器
    """

    def __init__(self, priority_manager: TaskPriorityManager,
                 planner: CBSPlanner):
        """
        Initialize task assignment manager
        初始化任务分配管理器

        Args/参数:
            priority_manager: Task priority manager/任务优先级管理器
            planner: Path planner/路径规划器
        """
        self.robots: Dict[int, Robot] = {}
        self.tasks: Dict[int, Task] = {}
        self.priority_manager = priority_manager
        self.planner = planner
        self.assignments: Dict[int, int] = {}  # robot_id -> task_id
        self.cost_matrix: Optional[np.ndarray] = None

    def add_robot(self, robot: Robot) -> None:
        """
        Add robot to assignment system
        添加机器人到分配系统

        Args/参数:
            robot: Robot to add/要添加的机器人
        """
        self.robots[robot.id] = robot

    def add_task(self, task: Task) -> None:
        """
        Add task to assignment system
        添加任务到分配系统

        Args/参数:
            task: Task to add/要添加的任务
        """
        self.tasks[task.id] = task
        self.priority_manager.add_task(task)

    def remove_task(self, task_id: int) -> None:
        """
        Remove task from assignment system
        从分配系统移除任务

        Args/参数:
            task_id: Task ID to remove/要移除的任务ID
        """
        if task_id in self.tasks:
            del self.tasks[task_id]
            self.priority_manager.remove_task(task_id)
            # 清除相关分配
            for robot_id, assigned_task_id in list(self.assignments.items()):
                if assigned_task_id == task_id:
                    del self.assignments[robot_id]

    def update_assignments(self, current_time: int) -> Dict[int, int]:
        """
        Update task assignments
        更新任务分配

        Args/参数:
            current_time: Current system time/当前系统时间

        Returns/返回:
            Dict[int, int]: New task assignments (robot_id -> task_id)
                           新的任务分配（机器人ID -> 任务ID）
        """
        # 更新任务优先级
        self.priority_manager.update_priorities(current_time)

        # 获取可用机器人和待分配任务
        available_robots = self._get_available_robots()
        pending_tasks = self._get_pending_tasks()

        if not available_robots or not pending_tasks:
            return self.assignments

        # 构建代价矩阵
        self._build_cost_matrix(available_robots, pending_tasks, current_time)

        # 使用匈牙利算法进行任务分配
        robot_indices, task_indices = linear_sum_assignment(self.cost_matrix)

        # 更新分配结果
        for robot_idx, task_idx in zip(robot_indices, task_indices):
            robot_id = list(available_robots.keys())[robot_idx]
            task_id = list(pending_tasks.keys())[task_idx]

            # 检查分配代价是否合理
            if self.cost_matrix[robot_idx, task_idx] < float('inf'):
                self.assignments[robot_id] = task_id
                self.tasks[task_id].state = 'assigned'
                self.tasks[task_id].assigned_to = robot_id
                self.tasks[task_id].assign_time = current_time

        return self.assignments

    def _get_available_robots(self) -> Dict[int, Robot]:
        """
        Get available robots for task assignment
        获取可用于任务分配的机器人

        Returns/返回:
            Dict[int, Robot]: Available robots/可用机器人
        """
        return {
            robot.id: robot for robot in self.robots.values()
            if robot.state == 'idle' and robot.id not in self.assignments
        }

    def _get_pending_tasks(self) -> Dict[int, Task]:
        """
        Get pending tasks for assignment
        获取待分配的任务

        Returns/返回:
            Dict[int, Task]: Pending tasks/待分配任务
        """
        return {
            task.id: task for task in self.tasks.values()
            if task.state == 'open'
        }

    def _build_cost_matrix(self, available_robots: Dict[int, Robot],
                           pending_tasks: Dict[int, Task],
                           current_time: int) -> None:
        """
        Build cost matrix for task assignment
        构建任务分配的代价矩阵

        Args/参数:
            available_robots: Available robots/可用机器人
            pending_tasks: Pending tasks/待分配任务
            current_time: Current system time/当前系统时间
        """
        num_robots = len(available_robots)
        num_tasks = len(pending_tasks)
        self.cost_matrix = np.full((num_robots, num_tasks), float('inf'))

        for i, (robot_id, robot) in enumerate(available_robots.items()):
            for j, (task_id, task) in enumerate(pending_tasks.items()):
                cost = self._calculate_assignment_cost(robot, task, current_time)
                self.cost_matrix[i, j] = cost

    def _calculate_assignment_cost(self, robot: Robot, task: Task,
                                   current_time: int) -> float:
        """
        Calculate cost for assigning task to robot
        计算将任务分配给机器人的代价

        Args/参数:
            robot: Robot to assign to/要分配的机器人
            task: Task to be assigned/要分配的任务
            current_time: Current system time/当前系统时间

        Returns/返回:
            float: Assignment cost/分配代价
        """
        # 如果任务已过期，返回无穷大代价
        if task.deadline <= current_time:
            return float('inf')

        # 计算路径长度代价
        path = self.planner._a_star_search(
            robot.position,
            task.start,
            set(),
            robot.id
        )
        if not path:
            return float('inf')

        path_cost = len(path)

        # 计算任务执行时间代价
        execution_cost = self._estimate_execution_time(task)

        # 计算优先级影响
        priority_factor = 1.0 - task.priority  # 优先级越高，代价越低

        # 计算时间窗口影响
        time_window = task.deadline - current_time
        time_factor = max(0.1, min(1.0, path_cost / time_window))

        # 综合代价计算
        total_cost = (
                0.4 * path_cost +
                0.3 * execution_cost +
                0.2 * priority_factor * max(path_cost, execution_cost) +
                0.1 * time_factor * max(path_cost, execution_cost)
        )

        return total_cost

    def _estimate_execution_time(self, task: Task) -> float:
        """
        Estimate task execution time
        估计任务执行时间

        Args/参数:
            task: Task to estimate/要估计的任务

        Returns/返回:
            float: Estimated execution time/估计执行时间
        """
        # 基于任务类型和特定参数估计执行时间
        base_time = {
            1: 100,  # 梳棉->一并基础时间
            2: 120,  # 一并->二并基础时间
            3: 150  # 二并->粗纱基础时间
        }.get(task.type, 100)

        # 考虑任务特定参数的影响
        if task.specific:
            if task.type == 1:
                base_time += task.specific.unload_time
                base_time *= (1 + task.specific.full_rate)
            elif task.type == 2:
                base_time += task.specific.load_time
                base_time *= (1 + task.specific.empty_rate)
            elif task.type == 3:
                base_time += task.specific.unload_time
                base_time *= (1 + task.specific.full_rate)

        return base_time

    def get_robot_assignments(self) -> Dict[int, List[int]]:
        """
        Get historical task assignments for each robot
        获取每个机器人的历史任务分配

        Returns/返回:
            Dict[int, List[int]]: Robot task assignment history
                                 机器人任务分配历史
        """
        robot_assignments = defaultdict(list)
        for robot in self.robots.values():
            robot_assignments[robot.id].extend(robot.completed_tasks)
            if robot.id in self.assignments:
                robot_assignments[robot.id].append(self.assignments[robot.id])
        return dict(robot_assignments)

    def get_assignment_statistics(self) -> Dict[str, Any]:
        """
        Get task assignment statistics
        获取任务分配统计信息

        Returns/返回:
            Dict[str, Any]: Assignment statistics/分配统计信息
        """
        stats = {
            'total_assignments': len(self.assignments),
            'robots_utilized': len(set(self.assignments.keys())),
            'tasks_assigned': len(set(self.assignments.values())),
            'assignment_distribution': defaultdict(int)
        }

        for robot_id in self.assignments:
            stats['assignment_distribution'][robot_id] += 1

        return stats


class DeadlockManager:
    """
    Deadlock detection and resolution system
    死锁检测和解决系统

    负责检测和解决系统中的死锁情况

    Attributes/属性:
        robots: Robot dictionary/机器人字典
        tasks: Task dictionary/任务字典
        planner: Path planner/路径规划器
        detection_interval: Deadlock detection interval/死锁检测间隔
        timeout_threshold: Deadlock timeout threshold/死锁超时阈值
        last_check_time: Last detection check time/上次检测时间
        deadlock_history: History of detected deadlocks/死锁检测历史
        resolution_stats: Statistics of deadlock resolutions/死锁解决统计
    """

    def __init__(self, planner: CBSPlanner,
                 detection_interval: int = SimConfig.DEADLOCK_DETECTION_INTERVAL,
                 timeout_threshold: int = SimConfig.DEADLOCK_TIMEOUT):
        """
        Initialize deadlock manager
        初始化死锁管理器

        Args/参数:
            planner: Path planner/路径规划器
            detection_interval: Detection interval/检测间隔
            timeout_threshold: Timeout threshold/超时阈值
        """
        self.robots: Dict[int, Robot] = {}
        self.tasks: Dict[int, Task] = {}
        self.planner = planner
        self.detection_interval = detection_interval
        self.timeout_threshold = timeout_threshold
        self.last_check_time = 0

        # 死锁历史记录
        self.deadlock_history: List[Dict[str, Any]] = []

        # 解决方案统计
        self.resolution_stats = {
            'total_deadlocks': 0,
            'successful_resolutions': 0,
            'failed_resolutions': 0,
            'resolution_methods': defaultdict(int)
        }

    def add_robot(self, robot: Robot) -> None:
        """
        Add robot to deadlock management
        添加机器人到死锁管理

        Args/参数:
            robot: Robot to add/要添加的机器人
        """
        self.robots[robot.id] = robot

    def add_task(self, task: Task) -> None:
        """
        Add task to deadlock management
        添加任务到死锁管理

        Args/参数:
            task: Task to add/要添加的任务
        """
        self.tasks[task.id] = task

    def check_deadlocks(self, current_time: int) -> List[Dict[str, Any]]:
        """
        Check for deadlocks in the system
        检查系统中的死锁

        Args/参数:
            current_time: Current system time/当前系统时间

        Returns/返回:
            List[Dict[str, Any]]: Detected deadlocks/检测到的死锁
        """
        if current_time - self.last_check_time < self.detection_interval:
            return []

        self.last_check_time = current_time
        deadlocks = []

        # 检查机器人状态
        stuck_robots = self._find_stuck_robots(current_time)
        if not stuck_robots:
            return []

        # 构建等待图
        wait_graph = self._build_wait_graph(stuck_robots)

        # 检测循环等待
        cycles = self._detect_cycles(wait_graph)

        for cycle in cycles:
            deadlock = {
                'robots': cycle,
                'time': current_time,
                'type': self._classify_deadlock(cycle, wait_graph),
                'severity': self._calculate_deadlock_severity(cycle, current_time)
            }
            deadlocks.append(deadlock)
            self.deadlock_history.append(deadlock)
            self.resolution_stats['total_deadlocks'] += 1

        return deadlocks

    def _find_stuck_robots(self, current_time: int) -> List[int]:
        """
        Find stuck robots in the system
        查找系统中卡住的机器人

        Args/参数:
            current_time: Current system time/当前系统时间

        Returns/返回:
            List[int]: List of stuck robot IDs/卡住的机器人ID列表
        """
        stuck_robots = []
        for robot in self.robots.values():
            if robot.is_stuck(current_time, self.timeout_threshold):
                stuck_robots.append(robot.id)
        return stuck_robots

    def _build_wait_graph(self, stuck_robots: List[int]) -> Dict[int, Set[int]]:
        """
        Build wait graph for deadlock detection
        构建用于死锁检测的等待图

        Args/参数:
            stuck_robots: List of stuck robot IDs/卡住的机器人ID列表

        Returns/返回:
            Dict[int, Set[int]]: Wait graph/等待图
        """
        wait_graph = defaultdict(set)

        for robot_id in stuck_robots:
            robot = self.robots[robot_id]
            if not robot.path or robot.path_index >= len(robot.path):
                continue

            # 获取机器人当前目标位置
            target_pos = robot.path[robot.path_index]

            # 检查是否被其他机器人阻塞
            for other_id, other_robot in self.robots.items():
                if other_id != robot_id and other_robot.position == target_pos:
                    wait_graph[robot_id].add(other_id)

        return wait_graph

    def _detect_cycles(self, wait_graph: Dict[int, Set[int]]) -> List[List[int]]:
        """
        Detect cycles in wait graph
        检测等待图中的循环

        Args/参数:
            wait_graph: Wait graph/等待图

        Returns/返回:
            List[List[int]]: List of detected cycles/检测到的循环列表
        """

        def find_cycle(node: int, visited: Set[int],
                       path: Set[int], cycle: List[int]) -> bool:
            """DFS helper to find cycles/用DFS查找循环"""
            visited.add(node)
            path.add(node)
            cycle.append(node)

            for neighbor in wait_graph[node]:
                if neighbor not in visited:
                    if find_cycle(neighbor, visited, path, cycle):
                        return True
                elif neighbor in path:
                    # 找到循环
                    while cycle[0] != neighbor:
                        cycle.pop(0)
                    return True

            path.remove(node)
            cycle.pop()
            return False

        cycles = []
        visited = set()

        for node in wait_graph:
            if node not in visited:
                cycle = []
                if find_cycle(node, visited, set(), cycle):
                    cycles.append(cycle[:])

        return cycles

    def _classify_deadlock(self, cycle: List[int],
                           wait_graph: Dict[int, Set[int]]) -> str:
        """
        Classify type of deadlock
        分类死锁类型

        Args/参数:
            cycle: Deadlock cycle/死锁循环
            wait_graph: Wait graph/等待图

        Returns/返回:
            str: Deadlock type/死锁类型
        """
        if len(cycle) == 2:
            return 'direct_conflict'
        elif len(cycle) == 3:
            return 'triangular_deadlock'
        elif all(len(wait_graph[robot_id]) == 1 for robot_id in cycle):
            return 'circular_deadlock'
        else:
            return 'complex_deadlock'

    def _calculate_deadlock_severity(self, cycle: List[int],
                                     current_time: int) -> float:
        """
        Calculate severity of deadlock
        计算死锁严重程度

        Args/参数:
            cycle: Deadlock cycle/死锁循环
            current_time: Current system time/当前系统时间

        Returns/返回:
            float: Deadlock severity score/死锁严重程度分数
        """
        severity = 0.0

        for robot_id in cycle:
            robot = self.robots[robot_id]

            # 考虑任务紧急程度
            if robot.current_task_id:
                task = self.tasks.get(robot.current_task_id)
                if task:
                    time_to_deadline = task.deadline - current_time
                    severity += max(0, 1 - time_to_deadline / 1000)

            # 考虑等待时间
            stuck_time = current_time - robot.last_update_time
            severity += min(1, stuck_time / self.timeout_threshold)

        return severity / len(cycle)

    def resolve_deadlock(self, deadlock: Dict[str, Any]) -> bool:
        """
        Resolve detected deadlock
        解决检测到的死锁

        Args/参数:
            deadlock: Deadlock information/死锁信息

        Returns/返回:
            bool: Whether resolution was successful/是否成功解决
        """
        resolution_methods = {
            'direct_conflict': self._resolve_direct_conflict,
            'triangular_deadlock': self._resolve_triangular_deadlock,
            'circular_deadlock': self._resolve_circular_deadlock,
            'complex_deadlock': self._resolve_complex_deadlock
        }

        method = resolution_methods.get(deadlock['type'],
                                        self._resolve_complex_deadlock)
        success = method(deadlock)

        # 更新统计信息
        if success:
            self.resolution_stats['successful_resolutions'] += 1
        else:
            self.resolution_stats['failed_resolutions'] += 1
        self.resolution_stats['resolution_methods'][deadlock['type']] += 1

        return success

    def _resolve_direct_conflict(self, deadlock: Dict[str, Any]) -> bool:
        """
        Resolve direct conflict between two robots
        解决两个机器人之间的直接冲突

        Args/参数:
            deadlock: Deadlock information/死锁信息

        Returns/返回:
            bool: Resolution success/解决成功与否
        """
        robot1_id, robot2_id = deadlock['robots']
        robot1 = self.robots[robot1_id]
        robot2 = self.robots[robot2_id]

        # 选择一个机器人后退
        if robot1.priority > robot2.priority:
            return self._make_robot_backoff(robot2)
        else:
            return self._make_robot_backoff(robot1)

    def _resolve_triangular_deadlock(self, deadlock: Dict[str, Any]) -> bool:
        """
        Resolve triangular deadlock
        解决三角死锁

        Args/参数:
            deadlock: Deadlock information/死锁信息

        Returns/返回:
            bool: Resolution success/解决成功与否
        """
        # 找出优先级最低的机器人
        robots = [self.robots[rid] for rid in deadlock['robots']]
        lowest_priority_robot = min(robots, key=lambda r: r.priority)
        return self._make_robot_backoff(lowest_priority_robot)

    def _resolve_circular_deadlock(self, deadlock: Dict[str, Any]) -> bool:
        """
        Resolve circular deadlock
        解决环形死锁

        Args/参数:
            deadlock: Deadlock information/死锁信息

        Returns/返回:
            bool: Resolution success/解决成功与否
        """
        # 选择多个机器人同时移动
        success = True
        for robot_id in deadlock['robots'][::2]:  # 选择间隔的机器人
            robot = self.robots[robot_id]
            if not self._make_robot_backoff(robot):
                success = False
        return success

    def _resolve_complex_deadlock(self, deadlock: Dict[str, Any]) -> bool:
        """
        Resolve complex deadlock
        解决复杂死锁

        Args/参数:
            deadlock: Deadlock information/死锁信息

        Returns/返回:
            bool: Resolution success/解决成功与否
        """
        # 逐个尝试让机器人后退，直到解决死锁
        for robot_id in sorted(deadlock['robots'],
                               key=lambda rid: self.robots[rid].priority):
            if self._make_robot_backoff(self.robots[robot_id]):
                return True
        return False

    def _make_robot_backoff(self, robot: Robot) -> bool:
        """
        Make robot back off from current position
        使机器人从当前位置后退

        Args/参数:
            robot: Robot to back off/要后退的机器人

        Returns/返回:
            bool: Whether back-off was successful/是否成功后退
        """
        # 找到安全的后退位置
        backup_positions = self._find_backup_positions(robot)

        for pos in backup_positions:
            # 尝试规划到后退位置的路径
            new_path = self.planner._a_star_search(
                robot.position,
                pos,
                set(),
                robot.id
            )

            if new_path:
                # 更新机器人路径
                robot.path = new_path + robot.path[robot.path_index:]
                robot.path_index = 0
                return True

        return False

    def _find_backup_positions(self, robot: Robot) -> List[Position]:
        """
        Find safe positions for robot to back off to
        查找机器人可以安全后退到的位置

        Args/参数:
            robot: Robot to find backup positions for/要查找后退位置的机器人

        Returns/返回:
            List[Position]: List of safe backup positions/安全后退位置列表
        """
        positions = []
        current_pos = robot.position

        # 检查周围8个方向
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        for dx, dy in directions:
            pos = Position(current_pos.x + dx, current_pos.y + dy)

            # 检查位置是否安全
            if self._is_position_safe(pos):
                positions.append(pos)

        return positions

    def _is_position_safe(self, pos: Position) -> bool:
        """
        Check if position is safe for robot
        检查位置是否对机器人安全

        Args/参数:
            pos: Position to check/要检查的位置

        Returns/返回:
            bool: Whether position is safe/位置是否安全
        """
        # 检查是否在地图范围内
        if not (0 <= pos.x < self.planner.width and 0 <= pos.y < self.planner.height):
            return False

        # 检查是否是障碍物
        if self.planner.map_matrix[pos.y, pos.x] == 1:
            return False

        # 检查是否被其他机器人占用
        for robot in self.robots.values():
            if robot.position == pos:
                return False

        return True

    def get_deadlock_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about deadlock detection and resolution
        获取死锁检测和解决的统计信息

        Returns/返回:
            Dict[str, Any]: Deadlock statistics/死锁统计信息
        """
        return {
            'total_deadlocks': self.resolution_stats['total_deadlocks'],
            'successful_resolutions': self.resolution_stats['successful_resolutions'],
            'failed_resolutions': self.resolution_stats['failed_resolutions'],
            'resolution_success_rate': (
                    self.resolution_stats['successful_resolutions'] /
                    max(1, self.resolution_stats['total_deadlocks'])
            ),
            'deadlock_types': dict(self.resolution_stats['resolution_methods'])
        }


class PerformanceMonitor:
    """
    System performance monitoring and analysis
    系统性能监控和分析

    监控系统运行状态和性能指标

    Attributes/属性:
        metrics_window: Size of metrics window/指标窗口大小
        update_interval: Metrics update interval/指标更新间隔
        last_update_time: Last metrics update time/上次指标更新时间
        metrics_history: Historical metrics data/历史指标数据
        current_metrics: Current metrics values/当前指标值
        alerts: Performance alerts/性能警报
    """

    def __init__(self, metrics_window: int = SimConfig.METRICS_WINDOW_SIZE,
                 update_interval: int = SimConfig.METRICS_UPDATE_INTERVAL):
        """
        Initialize performance monitor
        初始化性能监控器

        Args/参数:
            metrics_window: Metrics window size/指标窗口大小
            update_interval: Update interval/更新间隔
        """
        self.metrics_window = metrics_window
        self.update_interval = update_interval
        self.last_update_time = 0

        # 历史指标数据
        self.metrics_history = {
            'robot_utilization': [],  # 机器人利用率
            'task_completion_rate': [],  # 任务完成率
            'average_wait_time': [],  # 平均等待时间
            'average_execution_time': [],  # 平均执行时间
            'deadlock_frequency': [],  # 死锁频率
            'path_efficiency': [],  # 路径效率
            'system_throughput': [],  # 系统吞吐量
            'resource_conflicts': [],  # 资源冲突
            'task_distribution': [],  # 任务分布
            'energy_efficiency': []  # 能源效率
        }

        # 当前指标值
        self.current_metrics = {key: 0.0 for key in self.metrics_history.keys()}

        # 性能警报
        self.alerts = []

        # 性能阈值配置
        self.thresholds = {
            'robot_utilization': 0.7,  # 最低机器人利用率
            'task_completion_rate': 0.8,  # 最低任务完成率
            'average_wait_time': 200,  # 最大平均等待时间
            'deadlock_frequency': 0.1,  # 最大死锁频率
            'path_efficiency': 0.6,  # 最低路径效率
            'resource_conflicts': 0.2  # 最大资源冲突率
        }

    def update_metrics(self, robots: Dict[int, Robot], tasks: Dict[int, Task],
                       deadlock_manager: DeadlockManager, current_time: int) -> None:
        """
        Update performance metrics
        更新性能指标

        Args/参数:
            robots: Robot dictionary/机器人字典
            tasks: Task dictionary/任务字典
            deadlock_manager: Deadlock manager/死锁管理器
            current_time: Current system time/当前系统时间
        """
        if current_time - self.last_update_time < self.update_interval:
            return

        # 更新各项指标
        self.current_metrics['robot_utilization'] = self._calculate_robot_utilization(robots)
        self.current_metrics['task_completion_rate'] = self._calculate_task_completion_rate(tasks)
        self.current_metrics['average_wait_time'] = self._calculate_average_wait_time(tasks)
        self.current_metrics['average_execution_time'] = self._calculate_average_execution_time(tasks)
        self.current_metrics['deadlock_frequency'] = self._calculate_deadlock_frequency(deadlock_manager)
        self.current_metrics['path_efficiency'] = self._calculate_path_efficiency(robots)
        self.current_metrics['system_throughput'] = self._calculate_system_throughput(tasks, current_time)
        self.current_metrics['resource_conflicts'] = self._calculate_resource_conflicts(robots)
        self.current_metrics['task_distribution'] = self._calculate_task_distribution(tasks)
        self.current_metrics['energy_efficiency'] = self._calculate_energy_efficiency(robots)

        # 更新历史数据
        for metric, value in self.current_metrics.items():
            self.metrics_history[metric].append(value)
            if len(self.metrics_history[metric]) > self.metrics_window:
                self.metrics_history[metric].pop(0)

        # 检查性能警报
        self._check_alerts()

        self.last_update_time = current_time

    def _calculate_robot_utilization(self, robots: Dict[int, Robot]) -> float:
        """
        Calculate robot utilization rate
        计算机器人利用率

        Args/参数:
            robots: Robot dictionary/机器人字典

        Returns/返回:
            float: Robot utilization rate/机器人利用率
        """
        if not robots:
            return 0.0

        active_robots = sum(1 for robot in robots.values()
                            if robot.state != 'idle')
        return active_robots / len(robots)

    def _calculate_task_completion_rate(self, tasks: Dict[int, Task]) -> float:
        """
        Calculate task completion rate
        计算任务完成率

        Args/参数:
            tasks: Task dictionary/任务字典

        Returns/返回:
            float: Task completion rate/任务完成率
        """
        if not tasks:
            return 0.0

        completed_tasks = sum(1 for task in tasks.values()
                              if task.state == 'completed')
        return completed_tasks / len(tasks)

    def _calculate_average_wait_time(self, tasks: Dict[int, Task]) -> float:
        """
        Calculate average task waiting time
        计算平均任务等待时间

        Args/参数:
            tasks: Task dictionary/任务字典

        Returns/返回:
            float: Average wait time/平均等待时间
        """
        wait_times = [task.waiting_time for task in tasks.values()
                      if task.waiting_time > 0]
        return np.mean(wait_times) if wait_times else 0.0

    def _calculate_average_execution_time(self, tasks: Dict[int, Task]) -> float:
        """
        Calculate average task execution time
        计算平均任务执行时间

        Args/参数:
            tasks: Task dictionary/任务字典

        Returns/返回:
            float: Average execution time/平均执行时间
        """
        execution_times = [task.execution_time for task in tasks.values()
                           if task.execution_time > 0]
        return np.mean(execution_times) if execution_times else 0.0

    def _calculate_deadlock_frequency(self,
                                      deadlock_manager: DeadlockManager) -> float:
        """
        Calculate deadlock frequency
        计算死锁频率

        Args/参数:
            deadlock_manager: Deadlock manager/死锁管理器

        Returns/返回:
            float: Deadlock frequency/死锁频率
        """
        stats = deadlock_manager.get_deadlock_statistics()
        total_checks = max(1, stats['total_deadlocks'] +
                           stats['successful_resolutions'])
        return stats['total_deadlocks'] / total_checks

    def _calculate_path_efficiency(self, robots: Dict[int, Robot]) -> float:
        """
        Calculate path planning efficiency
        计算路径规划效率

        Args/参数:
            robots: Robot dictionary/机器人字典

        Returns/返回:
            float: Path efficiency/路径效率
        """
        efficiencies = []
        for robot in robots.values():
            if robot.path and len(robot.path) > 1:
                # 比较实际路径长度与理想直线距离
                actual_length = len(robot.path)
                start = robot.start_position
                end = robot.target
                ideal_length = ((end.x - start.x) ** 2 +
                                (end.y - start.y) ** 2) ** 0.5
                efficiency = ideal_length / actual_length
                efficiencies.append(efficiency)

        return np.mean(efficiencies) if efficiencies else 1.0

    def _calculate_system_throughput(self, tasks: Dict[int, Task],
                                     current_time: int) -> float:
        """
        Calculate system throughput
        计算系统吞吐量

        Args/参数:
            tasks: Task dictionary/任务字典
            current_time: Current system time/当前系统时间

        Returns/返回:
            float: System throughput/系统吞吐量
        """
        if current_time == 0:
            return 0.0

        completed_tasks = sum(1 for task in tasks.values()
                              if task.state == 'completed')
        return completed_tasks / current_time

    def _calculate_resource_conflicts(self, robots: Dict[int, Robot]) -> float:
        """
        Calculate resource conflict rate
        计算资源冲突率

        Args/参数:
            robots: Robot dictionary/机器人字典

        Returns/返回:
            float: Resource conflict rate/资源冲突率
        """
        total_moves = 0
        conflict_moves = 0

        for robot in robots.values():
            if not robot.path:
                continue

            total_moves += len(robot.path)
            # 计算与其他机器人的路径交叉
            for other_robot in robots.values():
                if other_robot.id != robot.id and other_robot.path:
                    conflicts = set(robot.path) & set(other_robot.path)
                    conflict_moves += len(conflicts)

        return conflict_moves / (2 * max(1, total_moves))

    def _calculate_task_distribution(self, tasks: Dict[int, Task]) -> float:
        """
        Calculate task distribution evenness
        计算任务分布均匀性

        Args/参数:
            tasks: Task dictionary/任务字典

        Returns/返回:
            float: Task distribution evenness/任务分布均匀性
        """
        if not tasks:
            return 1.0

        type_counts = defaultdict(int)
        for task in tasks.values():
            type_counts[task.type] += 1

        # 计算分布的标准差
        mean_count = len(tasks) / len(type_counts)
        variance = sum((count - mean_count) ** 2
                       for count in type_counts.values()) / len(type_counts)
        std_dev = variance ** 0.5

        # 转换为0-1分数，越接近1表示分布越均匀
        return 1.0 / (1.0 + std_dev / mean_count)

    def _calculate_energy_efficiency(self, robots: Dict[int, Robot]) -> float:
        """
        Calculate system energy efficiency
        计算系统能源效率

        Args/参数:
            robots: Robot dictionary/机器人字典

        Returns/返回:
            float: Energy efficiency/能源效率
        """
        total_distance = 0
        total_tasks = 0

        for robot in robots.values():
            if robot.path:
                total_distance += len(robot.path)
            total_tasks += len(robot.completed_tasks)

        if total_distance == 0:
            return 1.0

        return total_tasks / total_distance

    def _check_alerts(self) -> None:
        """
        Check and generate performance alerts
        检查并生成性能警报
        """
        self.alerts = []

        for metric, threshold in self.thresholds.items():
            current_value = self.current_metrics[metric]

            if metric in ['average_wait_time', 'deadlock_frequency',
                          'resource_conflicts']:
                if current_value > threshold:
                    self.alerts.append({
                        'type': metric,
                        'severity': 'high' if current_value > threshold * 1.5 else 'medium',
                        'value': current_value,
                        'threshold': threshold
                    })
            else:
                if current_value < threshold:
                    self.alerts.append({
                        'type': metric,
                        'severity': 'high' if current_value < threshold * 0.5 else 'medium',
                        'value': current_value,
                        'threshold': threshold
                    })

    def get_performance_report(self) -> Dict[str, Any]:
        """
        Generate comprehensive performance report
        生成综合性能报告

        Returns/返回:
            Dict[str, Any]: Performance report/性能报告
        """
        return {
            'current_metrics': self.current_metrics.copy(),
            'historical_trends': {
                metric: {
                    'mean': np.mean(values),
                    'std': np.std(values),
                    'min': min(values),
                    'max': max(values)
                } for metric, values in self.metrics_history.items()
                if values
            },
            'alerts': self.alerts.copy(),
            'system_status': self._get_system_status()
        }

    def _get_system_status(self) -> str:
        """
        Determine overall system status
        确定系统整体状态

        Returns/返回:
            str: System status (normal/warning/critical)
                 系统状态（正常/警告/严重）
        """
        if any(alert['severity'] == 'high' for alert in self.alerts):
            return 'critical'
        elif any(alert['severity'] == 'medium' for alert in self.alerts):
            return 'warning'
        return 'normal'

    def get_optimization_suggestions(self) -> List[str]:
        """
        Generate system optimization suggestions
        生成系统优化建议

        Returns/返回:
            List[str]: List of optimization suggestions/优化建议列表
        """
        suggestions = []

        if self.current_metrics['robot_utilization'] < self.thresholds['robot_utilization']:
            suggestions.append("建议优化机器人分配策略以提高利用率")

        if self.current_metrics['task_completion_rate'] < self.thresholds['task_completion_rate']:
            suggestions.append("建议调整任务优先级机制以提高完成率")

        if self.current_metrics['average_wait_time'] > self.thresholds['average_wait_time']:
            suggestions.append("建议优化路径规划以减少等待时间")

        if self.current_metrics['path_efficiency'] < self.thresholds['path_efficiency']:
            suggestions.append("建议改进路径规划算法以提高效率")

        if self.current_metrics['resource_conflicts'] > self.thresholds['resource_conflicts']:
            suggestions.append("建议加强冲突预防机制")

        return suggestions


class System:
    """
    Main system class integrating all components
    整合所有组件的主系统类

    协调各个子系统的运行，管理整体系统状态

    Attributes/属性:
        global_time: Global system time/系统全局时间
        map_matrix: Environment map matrix/环境地图矩阵
        robots: Robot dictionary/机器人字典
        tasks: Task dictionary/任务字典
        task_queues: Task queue manager/任务队列管理器
        priority_manager: Task priority manager/任务优先级管理器
        assignment_manager: Task assignment manager/任务分配管理器
        deadlock_manager: Deadlock manager/死锁管理器
        performance_monitor: Performance monitor/性能监控器
        planner: Path planner/路径规划器
    """

    def __init__(self):
        """Initialize system/初始化系统"""
        self.global_time = 0

        # 初始化地图
        self.map_matrix = None
        self.work_areas = {}
        self.pickup_areas = {}

        # 初始化组件容器
        self.robots = {}
        self.tasks = {}
        self.task_queues = TaskQueues()

        # 初始化系统组件
        self.planner = None
        self.priority_manager = None
        self.assignment_manager = None
        self.deadlock_manager = None
        self.performance_monitor = None

        # 系统状态
        self.is_running = False
        self.pause_flag = False

        # 性能追踪
        self.performance_history = []
        self.system_logs = []

    def initialize(self) -> None:
        """
        Initialize system components
        初始化系统组件
        """
        print("正在初始化系统...")
        try:
            # 创建语义地图
            self.create_semantic_map()

            # 初始化规划器
            self.planner = CBSPlanner(
                self.map_matrix,
                SimConfig.ROBOT_RADIUS,
                SimConfig.MAX_CBS_ITERATIONS,
                SimConfig.CBS_TIMEOUT
            )

            # 初始化管理器
            self.priority_manager = TaskPriorityManager()
            self.assignment_manager = TaskAssignmentManager(
                self.priority_manager,
                self.planner
            )
            self.deadlock_manager = DeadlockManager(self.planner)
            self.performance_monitor = PerformanceMonitor()

            # 初始化机器人
            self.initialize_robots()

            # 生成初始任务
            self.generate_initial_tasks()

            # 注册组件
            self._register_components()

            print("系统初始化完成")

        except Exception as e:
            print(f"系统运行出错: {str(e)}")
            raise

    def create_semantic_map(self) -> None:
        """
        Create semantic map with different areas
        创建包含不同区域的语义地图
        """
        # 创建基础地图矩阵
        height = SimConfig.MAP_SIZE
        width = SimConfig.MAP_SIZE
        self.map_matrix = np.zeros((height, width), dtype=int)

        # 设置边界
        self.map_matrix[0, :] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[-1, :] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[:, 0] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[:, -1] = SimConfig.AREA_TYPES['boundary']

        # 定义工作区域
        self.work_areas = {
            'combing': [],  # 梳棉区域
            'drawing1': [],  # 一并条区域
            'drawing2': [],  # 二并条区域
            'roving': []  # 粗纱区域
        }

        # 定义取料区域
        self.pickup_areas = {
            'combing': [],  # 梳棉取料区
            'drawing1': [],  # 一并条取料区
            'drawing2': [],  # 二并条取料区
            'roving': []  # 粗纱取料区
        }

        # 设置各个区域
        for machine_type, x_pos in SimConfig.MACHINE_POSITIONS.items():
            count = SimConfig.MACHINE_COUNTS[machine_type]
            spacing = height // (count + 1)

            for i in range(count):
                y_pos = (i + 1) * spacing

                # 设置工作区域
                work_area = ((x_pos - 10, y_pos - 10), (x_pos + 10, y_pos + 10))
                self.work_areas[machine_type].append(work_area)
                self.map_matrix[y_pos - 10:y_pos + 10, x_pos - 10:x_pos + 10] = (
                    SimConfig.AREA_TYPES[machine_type]
                )

                # 设置取料区域
                pickup_area = ((x_pos - 15, y_pos - 5), (x_pos - 11, y_pos + 5))
                self.pickup_areas[machine_type].append(pickup_area)
                self.map_matrix[y_pos - 5:y_pos + 5, x_pos - 15:x_pos - 11] = (
                    SimConfig.AREA_TYPES[f'{machine_type}_pickup']
                )

    def initialize_robots(self) -> None:
        """
        Initialize robots
        初始化机器人
        """
        # 在地图底部均匀分布机器人
        spacing = self.map_matrix.shape[1] // (SimConfig.NUM_ROBOTS + 1)
        y_pos = self.map_matrix.shape[0] - 50  # 距离底部50个单位

        for i in range(SimConfig.NUM_ROBOTS):
            x_pos = (i + 1) * spacing
            position = Position(x_pos, y_pos)

            robot = Robot(
                id=i + 1,
                position=position,
                target=position,
                path=[],
                state='idle',
                current_task_id=0,
                path_index=0,
                last_update_time=0,
                completed_tasks=[],
                start_position=position
            )

            self.robots[robot.id] = robot

    def generate_initial_tasks(self) -> None:
        """
        Generate initial tasks
        生成初始任务
        """
        print(f"开始生成 {SimConfig.NUM_INITIAL_TASKS} 个初始任务...")

        # 任务类型到区域的映射
        task_areas = {
            1: {  # 梳棉->一并任务
                'start': self.pickup_areas['combing'],
                'end': self.pickup_areas['drawing1']
            },
            2: {  # 一并->二并任务
                'start': self.pickup_areas['drawing1'],
                'end': self.pickup_areas['drawing2']
            },
            3: {  # 二并->粗纱任务
                'start': self.pickup_areas['drawing2'],
                'end': self.pickup_areas['roving']
            }
        }

        # 平均分配三种类型的任务
        tasks_per_type = SimConfig.NUM_INITIAL_TASKS // 3
        remaining_tasks = SimConfig.NUM_INITIAL_TASKS % 3

        for task_type in [1, 2, 3]:
            num_tasks = tasks_per_type + (1 if task_type <= remaining_tasks else 0)

            for i in range(num_tasks):
                # 随机选择起点和终点区域
                start_area = random.choice(task_areas[task_type]['start'])
                end_area = random.choice(task_areas[task_type]['end'])

                # 在区域内找到具体位置
                start_pos = Position(
                    random.randint(start_area[0][0], start_area[1][0]),
                    random.randint(start_area[0][1], start_area[1][1])
                )
                end_pos = Position(
                    random.randint(end_area[0][0], end_area[1][0]),
                    random.randint(end_area[0][1], end_area[1][1])
                )

                # 创建任务特定参数
                specific_params = self._generate_task_params(task_type)

                # 创建任务
                task = Task(
                    id=len(self.tasks) + 1,
                    type=task_type,
                    state='open',
                    start=start_pos,
                    end=end_pos,
                    open_time=self.global_time,
                    deadline=self.global_time + random.randint(200, 500),
                    specific=specific_params
                )

                self.tasks[task.id] = task
                self.task_queues.add_task(task.id)

        print(f"成功生成 {len(self.tasks)} 个初始任务")

    def _generate_task_params(self, task_type: int) -> TaskSpecificParams:
        """
        Generate task specific parameters
        生成任务特定参数

        Args/参数:
            task_type: Task type/任务类型

        Returns/返回:
            TaskSpecificParams: Generated parameters/生成的参数
        """
        if task_type == 1:
            return TaskSpecificParams(
                unload_time=50 + random.random() * 100,
                full_rate=0.1 + random.random() * 0.4
            )
        elif task_type == 2:
            return TaskSpecificParams(
                empty_rate=0.1 + random.random() * 0.4,
                load_time=50 + random.random() * 100
            )
        else:
            return TaskSpecificParams(
                unload_time=30 + random.random() * 80,
                full_rate=0.2 + random.random() * 0.3
            )

    def _register_components(self) -> None:
        """
        Register components with each other
        组件之间互相注册
        """
        # 注册机器人到各个管理器
        for robot in self.robots.values():
            self.assignment_manager.add_robot(robot)
            self.deadlock_manager.add_robot(robot)

        # 注册任务到各个管理器
        for task in self.tasks.values():
            self.assignment_manager.add_task(task)
            self.deadlock_manager.add_task(task)

    def run(self) -> None:
        """
        Run system simulation
        运行系统仿真
        """
        print("开始系统仿真...")
        self.is_running = True

        try:
            while (self.is_running and
                   self.global_time < SimConfig.MAX_SIMULATION_TIME):
                if self.pause_flag:
                    time.sleep(0.1)
                    continue

                # 更新任务分配
                self.assignment_manager.update_assignments(self.global_time)

                # 更新机器人状态
                self.update_robot_status()

                # 检查死锁
                deadlocks = self.deadlock_manager.check_deadlocks(self.global_time)
                for deadlock in deadlocks:
                    self.deadlock_manager.resolve_deadlock(deadlock)

                # 更新性能指标
                self.performance_monitor.update_metrics(
                    self.robots,
                    self.tasks,
                    self.deadlock_manager,
                    self.global_time
                )

                # 记录性能历史
                if self.global_time % SimConfig.PERFORMANCE_LOG_INTERVAL == 0:
                    self.log_performance()

                self.global_time += SimConfig.TIME_UNIT

            print("系统仿真完成")
            self.generate_final_report()

        except Exception as e:
            print(f"系统运行出错: {str(e)}")
            raise

    def update_robot_status(self) -> None:
        """
        Update status of all robots
        更新所有机器人的状态
        """
        for robot in self.robots.values():
            if robot.state == 'idle':
                continue

            # 更新路径执行
            if robot.path and robot.path_index < len(robot.path):
                next_pos = robot.path[robot.path_index]

                # 检查移动是否安全
                if self._is_move_safe(robot, next_pos):
                    robot.position = next_pos
                    robot.path_index += 1
                    robot.last_update_time = self.global_time

            # 检查任务完成情况
            if (robot.current_task_id and
                    robot.position == self.tasks[robot.current_task_id].end):
                self._complete_task(robot)

    def _is_move_safe(self, robot: Robot, next_pos: Position) -> bool:
        """
        Check if move is safe
        检查移动是否安全

        Args/参数:
            robot: Robot to move/要移动的机器人
            next_pos: Next position/下一个位置

        Returns/返回:
            bool: Whether move is safe/移动是否安全
        """
        # 检查是否在地图范围内
        if not (0 <= next_pos.x < self.map_matrix.shape[1] and
                0 <= next_pos.y < self.map_matrix.shape[0]):
            return False

        # 检查是否是障碍物
        if self.map_matrix[next_pos.y, next_pos.x] == SimConfig.AREA_TYPES['boundary']:
            return False

        # 检查与其他机器人的碰撞
        for other_robot in self.robots.values():
            if other_robot.id != robot.id:
                if other_robot.position == next_pos:
                    return False

        return True

    def _complete_task(self, robot: Robot) -> None:
        """
        Complete current task for robot
        完成机器人当前任务

        Args/参数:
            robot: Robot completing task/完成任务的机器人
        """
        task_id = robot.current_task_id
        if not task_id:
            return

        task = self.tasks[task_id]
        task.state = 'completed'
        task.complete_time = self.global_time
        task.execution_time = task.complete_time - task.start_time
        task.total_time = task.complete_time - task.open_time

        robot.completed_tasks.append(task_id)
        robot.current_task_id = 0
        robot.state = 'idle'
        robot.path = []
        robot.path_index = 0

    def log_performance(self) -> None:
        """
        Log current performance metrics
        记录当前性能指标
        """
        performance_report = self.performance_monitor.get_performance_report()
        self.performance_history.append({
            'time': self.global_time,
            'metrics': performance_report['current_metrics'],
            'status': performance_report['system_status'],
            'alerts': performance_report['alerts']
        })

    def generate_final_report(self) -> Dict[str, Any]:
        """
        Generate final simulation report
        生成最终仿真报告

        Returns/返回:
            Dict[str, Any]: Final report/最终报告
        """
        report = {
            'simulation_time': self.global_time,
            'total_tasks': len(self.tasks),
            'completed_tasks': len([t for t in self.tasks.values()
                                    if t.state == 'completed']),
            'robot_statistics': self._get_robot_statistics(),
            'task_statistics': self._get_task_statistics(),
            'performance_summary': self._get_performance_summary(),
            'system_recommendations': (
                self.performance_monitor.get_optimization_suggestions()
            )
        }

        print("\n=== 仿真报告 ===")
        print(f"总仿真时间: {report['simulation_time']}")
        print(f"总任务数: {report['total_tasks']}")
        print(f"完成任务数: {report['completed_tasks']}")
        print("系统建议:")
        for rec in report['system_recommendations']:
            print(f"- {rec}")

        return report

    def _get_robot_statistics(self) -> Dict[str, Any]:
        """
        Get robot performance statistics
        获取机器人性能统计

        Returns/返回:
            Dict[str, Any]: Robot statistics/机器人统计
        """
        stats = {
            'total_robots': len(self.robots),
            'robot_utilization': {},
            'tasks_per_robot': {},
            'average_execution_time': {}
        }

        for robot in self.robots.values():
            completed_tasks = len(robot.completed_tasks)
            total_time = sum(self.tasks[task_id].execution_time
                             for task_id in robot.completed_tasks)

            stats['robot_utilization'][robot.id] = (
                    total_time / max(1, self.global_time)
            )
            stats['tasks_per_robot'][robot.id] = completed_tasks
            stats['average_execution_time'][robot.id] = (
                    total_time / max(1, completed_tasks)
            )

        return stats

    def _get_task_statistics(self) -> Dict[str, Any]:
        """
        Get task performance statistics
        获取任务性能统计

        Returns/返回:
            Dict[str, Any]: Task statistics/任务统计
        """
        stats = {
            'completion_rate': {},
            'average_wait_time': {},
            'average_execution_time': {}
        }

        for task_type in [1, 2, 3]:
            type_tasks = [t for t in self.tasks.values() if t.type == task_type]
            completed = [t for t in type_tasks if t.state == 'completed']

            if type_tasks:
                stats['completion_rate'][task_type] = len(completed) / len(type_tasks)
                stats['average_wait_time'][task_type] = (
                        sum(t.waiting_time for t in completed) / max(1, len(completed))
                )
                stats['average_execution_time'][task_type] = (
                        sum(t.execution_time for t in completed) / max(1, len(completed))
                )

        return stats

    def _get_performance_summary(self) -> Dict[str, Any]:
        """
        Get overall performance summary
        获取整体性能总结

        Returns/返回:
            Dict[str, Any]: Performance summary/性能总结
        """
        if not self.performance_history:
            return {}

        metrics = ['robot_utilization', 'task_completion_rate',
                   'average_wait_time', 'system_throughput']
        summary = {}

        for metric in metrics:
            values = [ph['metrics'][metric] for ph in self.performance_history]
            summary[metric] = {
                'mean': np.mean(values),
                'std': np.std(values),
                'min': min(values),
                'max': max(values)
            }

        return summary


def visualize_factory_layout(system: System) -> None:
    """
    Visualize factory layout and robot positions
    可视化工厂布局和机器人位置

    Args/参数:
        system: System instance/系统实例
    """
    plt.figure(figsize=(12, 8))
    plt.title('工厂布局与机器人位置')

    # 绘制地图
    plt.imshow(system.map_matrix, cmap='tab20', origin='lower')

    # 添加颜色条和标签
    cbar = plt.colorbar()
    cbar.set_ticks(list(SimConfig.AREA_TYPES.values()))
    cbar.set_ticklabels(list(SimConfig.AREA_TYPES.keys()))

    # 绘制机器人位置
    robot_positions = [(r.position.x, r.position.y) for r in system.robots.values()]
    if robot_positions:
        x, y = zip(*robot_positions)
        plt.scatter(x, y, c='red', marker='o', s=100, label='机器人')

    # 绘制任务起点和终点
    active_tasks = [t for t in system.tasks.values() if t.state in ['open', 'assigned']]
    if active_tasks:
        starts = [(t.start.x, t.start.y) for t in active_tasks]
        ends = [(t.end.x, t.end.y) for t in active_tasks]
        if starts:
            sx, sy = zip(*starts)
            plt.scatter(sx, sy, c='green', marker='^', s=100, label='任务起点')
        if ends:
            ex, ey = zip(*ends)
            plt.scatter(ex, ey, c='blue', marker='s', s=100, label='任务终点')

    plt.legend()
    plt.grid(True)
    plt.show()


def create_performance_visualization(system: System) -> None:
    """
    Create performance visualization plots
    创建性能可视化图表

    Args/参数:
        system: System instance/系统实例
    """
    if not system.performance_history:
        print("没有可用的性能数据进行可视化")
        return

    # 创建2x2的子图
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('系统性能指标')

    # 提取时间序列数据
    times = [ph['time'] for ph in system.performance_history]

    # 1. 机器人利用率
    utilization = [ph['metrics']['robot_utilization'] for ph in system.performance_history]
    ax1.plot(times, utilization, 'b-')
    ax1.set_title('机器人利用率')
    ax1.set_ylim(0, 1)
    ax1.grid(True)

    # 2. 任务完成率
    completion = [ph['metrics']['task_completion_rate'] for ph in system.performance_history]
    ax2.plot(times, completion, 'g-')
    ax2.set_title('任务完成率')
    ax2.set_ylim(0, 1)
    ax2.grid(True)

    # 3. 平均等待时间
    wait_times = [ph['metrics']['average_wait_time'] for ph in system.performance_history]
    ax3.plot(times, wait_times, 'r-')
    ax3.set_title('平均等待时间')
    ax3.grid(True)

    # 4. 系统吞吐量
    throughput = [ph['metrics']['system_throughput'] for ph in system.performance_history]
    ax4.plot(times, throughput, 'y-')
    ax4.set_title('系统吞吐量')
    ax4.grid(True)

    plt.tight_layout()
    plt.show()


def print_system_status(system: System) -> None:
    """
    Print current system status to terminal
    在终端打印当前系统状态

    Args/参数:
        system: System instance/系统实例
    """
    os.system('cls' if os.name == 'nt' else 'clear')  # 清屏

    print("\n=== 系统状态 ===")
    print(f"当前时间: {system.global_time}")

    # 任务统计
    total_tasks = len(system.tasks)
    completed_tasks = len([t for t in system.tasks.values() if t.state == 'completed'])
    open_tasks = len([t for t in system.tasks.values() if t.state == 'open'])
    assigned_tasks = len([t for t in system.tasks.values() if t.state == 'assigned'])

    print("\n--- 任务状态 ---")
    print(f"总任务数: {total_tasks}")
    print(f"已完成: {completed_tasks}")
    print(f"待分配: {open_tasks}")
    print(f"执行中: {assigned_tasks}")

    # 机器人状态
    idle_robots = len([r for r in system.robots.values() if r.state == 'idle'])
    working_robots = len(system.robots) - idle_robots

    print("\n--- 机器人状态 ---")
    print(f"总机器人数: {len(system.robots)}")
    print(f"空闲: {idle_robots}")
    print(f"工作中: {working_robots}")

    # 性能警报
    if system.performance_monitor.alerts:
        print("\n--- 性能警报 ---")
        for alert in system.performance_monitor.alerts:
            severity = "⚠️ " if alert['severity'] == 'medium' else "❌ "
            print(f"{severity}{alert['type']}: {alert['value']:.2f}")

    # 进度条
    if total_tasks > 0:
        progress = completed_tasks / total_tasks
        bar_length = 40
        filled_length = int(bar_length * progress)
        bar = '=' * filled_length + '-' * (bar_length - filled_length)
        print("\n任务完成进度:")
        print(f"[{bar}] {progress * 100:.1f}%")


def main():
    """
    Main function
    主函数
    """
    print("=== 多机器人任务分配与路径规划系统启动 ===\n")

    try:
        # 创建并初始化系统
        system = System()
        system.initialize()

        # 显示初始布局
        visualize_factory_layout(system)

        # 运行系统
        last_viz_time = 0
        viz_interval = 1000  # 可视化更新间隔

        while system.is_running:
            system.run()

            # 定期更新可视化
            if system.global_time - last_viz_time >= viz_interval:
                print_system_status(system)
                last_viz_time = system.global_time

            # 检查是否所有任务都已完成
            all_completed = all(task.state == 'completed'
                                for task in system.tasks.values())
            if all_completed:
                system.is_running = False

        # 显示最终结果
        print("\n=== 仿真完成 ===")
        create_performance_visualization(system)
        visualize_factory_layout(system)

    except KeyboardInterrupt:
        print("\n系统被用户中断")
    except Exception as e:
        print(f"\n发生错误: {str(e)}")
        traceback.print_exc()
    finally:
        print("\n系统关闭")


if __name__ == "__main__":
    main()