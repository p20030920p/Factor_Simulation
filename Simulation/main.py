import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import random
from collections import defaultdict
import heapq
import time
import matplotlib.pyplot as plt


# 1. 基础配置类
class SimConfig:
    MAP_SIZE = 1000  # 新地图尺寸1000x1000
    NUM_ROBOTS = 10
    NUM_INITIAL_TASKS = 45
    MAX_SIMULATION_TIME = 360000  #
    TIME_UNIT = 1
    MAX_REPLAN_ATTEMPTS = 3
    PRIORITY_UPDATE_INTERVAL = 5
    CHAIN_OPTIMIZATION_THRESHOLD = 15

    # 机器区域配置
    MACHINE_POSITIONS = {
        'combing': 150,  # 梳棉机x坐标
        'drawing1': 350,  # 一并条x坐标
        'drawing2': 550,  # 二并条x坐标
        'roving': 750  # 粗纱机x坐标
    }

    # 机器数量配置
    MACHINE_COUNTS = {
        'combing': 10,
        'drawing1': 5,
        'drawing2': 5,
        'roving': 5
    }

    # 区域值定义
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


# 2. 基础数据结构
@dataclass
class Position:
    x: int
    y: int

    def __iter__(self):
        yield self.x
        yield self.y

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Position(self.x - other.x, self.y - other.y)


@dataclass
class TaskSpecificParams:
    unload_time: float = 0.0
    full_rate: float = 0.0
    empty_rate: float = 0.0
    load_time: float = 0.0


@dataclass
class Task:
    id: int
    type: int  # 1: 梳棉->一并, 2: 一并->二并, 3: 二并->粗纱
    state: str  # new, open, assigned, completed
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


@dataclass
class Robot:
    id: int
    position: Position
    target: Position
    path: List[Position]
    state: str  # idle, moving, working
    current_task_id: int
    path_index: int
    last_update_time: int
    completed_tasks: List[int]
    start_position: Position

    def get_task_statistics(self, tasks: List[Task]) -> Tuple[int, int, int, List[int]]:
        completed_ids = self.completed_tasks
        type1_count = sum(1 for task_id in completed_ids if tasks[task_id - 1].type == 1)
        type2_count = sum(1 for task_id in completed_ids if tasks[task_id - 1].type == 2)
        type3_count = sum(1 for task_id in completed_ids if tasks[task_id - 1].type == 3)
        return len(completed_ids), type1_count, type2_count, type3_count, sorted(completed_ids)


@dataclass
class TaskRecord:
    task_id: int
    task_type: int
    start_time: int
    completion_time: int
    waiting_time: float
    execution_time: float
    start_pos: Tuple[int, int]
    end_pos: Tuple[int, int]


class TaskQueues:
    def __init__(self):
        self.all = []
        self.new = []
        self.open = []
        self.assigned = []
        self.completed = []


class TaskSequenceTracker:
    def __init__(self):
        self.sequences: Dict[int, List[TaskRecord]] = {}
        self.completed_tasks: Dict[int, bool] = {}
        self.task_chains: Dict[int, List[int]] = {}  # 存储任务链关系

    def record_task(self, robot_id: int, task_record: TaskRecord) -> None:
        if robot_id not in self.sequences:
            self.sequences[robot_id] = []
        self.sequences[robot_id].append(task_record)
        self.completed_tasks[task_record.task_id] = True

    def get_sequence(self, robot_id: int) -> List[TaskRecord]:
        return self.sequences.get(robot_id, [])

    def check_completion(self, total_tasks: List[int]) -> Tuple[bool, List[int]]:
        uncompleted = [
            task_id for task_id in total_tasks
            if task_id not in self.completed_tasks
        ]
        return len(uncompleted) == 0, uncompleted

    def add_to_chain(self, prev_task_id: int, next_task_id: int):
        if prev_task_id not in self.task_chains:
            self.task_chains[prev_task_id] = []
        self.task_chains[prev_task_id].append(next_task_id)


class System:
    def __init__(self):
        self.task_tracker = TaskSequenceTracker()
        self.map_matrix = np.zeros((SimConfig.MAP_SIZE, SimConfig.MAP_SIZE))
        self.robots: List[Robot] = []
        self.tasks: List[Task] = []
        self.task_queues = TaskQueues()
        self.global_time = 0

        # 区域信息存储
        self.work_areas = {
            'combing': [],
            'drawing1': [],
            'drawing2': [],
            'roving': []
        }
        self.pickup_areas = {
            'combing': [],
            'drawing1': [],
            'drawing2': [],
            'roving': []
        }
        self.charging_areas = []
        self.material_area = None
        self.product_area = None

        # 性能追踪
        self.task_assignments = []
        self.task_completions = []
        self.conflicts = []
        self.system_performance = []

    def initialize(self):
        """系统初始化"""
        self.create_semantic_map()
        self.initialize_robots()
        self.generate_initial_tasks()
        self.calculate_dynamic_priorities()

        print(f"System initialized:")
        print(f"Map size: {SimConfig.MAP_SIZE}x{SimConfig.MAP_SIZE}")
        print(f"Number of robots: {SimConfig.NUM_ROBOTS}")
        print(f"Initial tasks: {SimConfig.NUM_INITIAL_TASKS}")

    def create_semantic_map(self):
        """创建语义地图"""
        # 1. 设置边界
        self.map_matrix[0:10, :] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[-10:, :] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[:, 0:10] = SimConfig.AREA_TYPES['boundary']
        self.map_matrix[:, -10:] = SimConfig.AREA_TYPES['boundary']

        # 2. 设置机器区域
        for machine_type, x_pos in SimConfig.MACHINE_POSITIONS.items():
            num_machines = SimConfig.MACHINE_COUNTS[machine_type]
            interval = 80
            start_y = (SimConfig.MAP_SIZE - (num_machines - 1) * interval) // 2

            for i in range(num_machines):
                y = start_y + i * interval

                # 设置机器位置
                self.map_matrix[y - 20:y + 20, x_pos - 20:x_pos + 20] = SimConfig.AREA_TYPES[machine_type]
                self.work_areas[machine_type].append([(x_pos - 20, y - 20), (x_pos + 20, y + 20)])

                # 设置取放区域
                pickup_x = x_pos - 40
                self.map_matrix[y - 15:y + 15, pickup_x - 15:pickup_x + 15] = SimConfig.AREA_TYPES[
                    f'{machine_type}_pickup']
                self.pickup_areas[machine_type].append([(pickup_x - 15, y - 15), (pickup_x + 15, y + 15)])

        # 3. 设置仓库区域
        # 原料仓
        self.map_matrix[480:520, 30:70] = SimConfig.AREA_TYPES['material_warehouse']
        self.material_area = [(30, 480), (70, 520)]

        # 成品仓
        self.map_matrix[480:520, 930:970] = SimConfig.AREA_TYPES['product_warehouse']
        self.product_area = [(930, 480), (970, 520)]

        # 4. 设置充电站
        charging_positions = [
            (50, 50), (50, 950),
            (950, 50), (950, 950)
        ]

        for x, y in charging_positions:
            self.map_matrix[y - 20:y + 20, x - 20:x + 20] = SimConfig.AREA_TYPES['charging']
            self.charging_areas.append([(x - 20, y - 20), (x + 20, y + 20)])

        # 5. 设置通道
        # 横向通道
        for y in [300, 500, 700]:
            self.map_matrix[y - 10:y + 10, :] = SimConfig.AREA_TYPES['empty']

        # 纵向通道
        for x in [110, 310, 510, 710]:
            self.map_matrix[:, x - 10:x + 10] = SimConfig.AREA_TYPES['empty']

    def initialize_robots(self):
        """初始化机器人位置"""
        start_x = 880  # 右下角起始x坐标
        start_y = 880  # 右下角起始y坐标
        robots_per_row = 5  # 每行放置的机器人数量

        for i in range(SimConfig.NUM_ROBOTS):
            row = i // robots_per_row
            col = i % robots_per_row
            x = start_x + col * (10 + 5)  # 机器人大小10，间隔5
            y = start_y + row * (10 + 5)

            # 创建机器人
            position = Position(x, y)
            robot = Robot(
                id=i + 1,
                position=position,
                target=position,
                path=[],
                state='idle',
                current_task_id=0,
                path_index=1,
                last_update_time=0,
                completed_tasks=[],
                start_position=position
            )

            self.robots.append(robot)
            # 在地图上标记机器人位置
            self.map_matrix[y:y + 10, x:x + 10] = SimConfig.AREA_TYPES['robot']

    def generate_initial_tasks(self):
        """生成初始任务集"""
        print(f"Starting to generate {SimConfig.NUM_INITIAL_TASKS} initial tasks...")

        # 工作区域定义
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
                if task_type == 1:
                    specific_params = TaskSpecificParams(
                        unload_time=50 + random.random() * 100,
                        full_rate=0.1 + random.random() * 0.4
                    )
                elif task_type == 2:
                    specific_params = TaskSpecificParams(
                        empty_rate=0.1 + random.random() * 0.4,
                        load_time=50 + random.random() * 100
                    )
                else:
                    specific_params = TaskSpecificParams(
                        unload_time=30 + random.random() * 80,
                        full_rate=0.2 + random.random() * 0.3
                    )

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

                self.tasks.append(task)
                self.task_queues.all.append(task.id)
                self.task_queues.open.append(task.id)

        print(f"Successfully generated {len(self.tasks)} initial tasks")

    def generate_dynamic_tasks(self, completed_task: Task):
        """根据完成的任务生成新的任务"""
        # 确定下一个任务类型
        next_type = completed_task.type + 1 if completed_task.type < 3 else None

        if next_type is None:
            return None

        task_areas = {
            2: {  # 一并->二并任务
                'start': self.pickup_areas['drawing1'],
                'end': self.pickup_areas['drawing2']
            },
            3: {  # 二并->粗纱任务
                'start': self.pickup_areas['drawing2'],
                'end': self.pickup_areas['roving']
            }
        }

        # 选择起点和终点区域
        start_area = random.choice(task_areas[next_type]['start'])
        end_area = random.choice(task_areas[next_type]['end'])

        # 创建新任务
        new_task = Task(
            id=len(self.tasks) + 1,
            type=next_type,
            state='open',
            start=Position(
                random.randint(start_area[0][0], start_area[1][0]),
                random.randint(start_area[0][1], start_area[1][1])
            ),
            end=Position(
                random.randint(end_area[0][0], end_area[1][0]),
                random.randint(end_area[0][1], end_area[1][1])
            ),
            open_time=self.global_time,
            deadline=self.global_time + random.randint(150, 350),
            specific=TaskSpecificParams(
                unload_time=40 + random.random() * 80,
                full_rate=0.15 + random.random() * 0.35
            )
        )

        self.tasks.append(new_task)
        self.task_queues.all.append(new_task.id)
        self.task_queues.open.append(new_task.id)

        # 记录任务链关系
        self.task_tracker.add_to_chain(completed_task.id, new_task.id)

        print(f"Generated new task {new_task.id} (type {next_type}) "
              f"following completion of task {completed_task.id}")

        return new_task.id

    def calculate_system_load(self) -> float:
        """计算当前系统负载"""
        queue_length = len(self.task_queues.open)
        completed_length = len(self.task_queues.completed)
        queue_load = queue_length / max(30, queue_length + completed_length)

        busy_robots = sum(1 for robot in self.robots if robot.state != 'idle')
        robot_load = busy_robots / SimConfig.NUM_ROBOTS

        return 0.7 * queue_load + 0.3 * robot_load

    def calculate_dynamic_priorities(self):
        """计算任务动态优先级"""
        if not self.task_queues.open:
            return

        system_load = self.calculate_system_load()

        # 按类型分类任务
        type_tasks = defaultdict(list)
        for task_id in self.task_queues.open:
            task = self.tasks[task_id - 1]
            type_tasks[task.type].append(task_id)

        # 处理每种类型的任务
        for task_type, task_ids in type_tasks.items():
            self._process_type_tasks(task_ids, system_load, task_type)

        # 按优先级排序
        self.task_queues.open.sort(
            key=lambda x: self.tasks[x - 1].priority,
            reverse=True
        )

    def _process_type_tasks(self, task_ids: List[int], system_load: float, task_type: int):
        """处理特定类型任务的优先级"""
        for task_id in task_ids:
            task = self.tasks[task_id - 1]

            # 基础等待时间因子
            waiting_factor = min(1.0, (self.global_time - task.open_time) / 100.0)

            # 紧急程度因子
            urgency_factor = max(0, 1.0 - (task.deadline - self.global_time) / 200.0)

            # 任务链因子 (如果是链中的任务给予额外优先级)
            chain_factor = 0.2 if task_id in self.task_tracker.task_chains else 0.0

            # 根据系统负载调整权重
            if system_load > 0.7:
                w1, w2, w3, w4 = 0.3, 0.3, 0.2, 0.2
            elif system_load > 0.4:
                w1, w2, w3, w4 = 0.4, 0.3, 0.2, 0.1
            else:
                w1, w2, w3, w4 = 0.5, 0.2, 0.2, 0.1

            # 计算最终优先级
            priority = (
                    w1 * waiting_factor +
                    w2 * urgency_factor +
                    w3 * (1.0 - system_load) +
                    w4 * chain_factor
            )

            # 根据任务类型调整优先级
            type_multiplier = 1.0
            if task_type == 2:  # 一并->二并任务略微提高优先级
                type_multiplier = 1.1
            elif task_type == 3:  # 二并->粗纱任务最高优先级
                type_multiplier = 1.2

            task.priority = round(priority * type_multiplier, 2)

    def assign_tasks(self):
        """分配任务给空闲机器人"""
        # 找到空闲机器人
        idle_robots = [robot for robot in self.robots if robot.state == 'idle']

        if not idle_robots or not self.task_queues.open:
            return

        # 决定使用哪种分配算法
        use_hungarian = (len(idle_robots) > 1 and
                         len(self.task_queues.open) > 1 and
                         self.global_time % 50 == 0)  # 每50个时间单位使用一次匈牙利算法

        if use_hungarian:
            self._hungarian_assignment(idle_robots)
        else:
            self._greedy_assignment(idle_robots)

    def _hungarian_assignment(self, idle_robots: List[Robot]):
        """使用匈牙利算法进行任务分配"""
        from scipy.optimize import linear_sum_assignment

        num_robots = len(idle_robots)
        task_queue = self.task_queues.open[:]
        num_tasks = min(len(task_queue), num_robots * 3)  # 考虑每个机器人的前3个任务选项

        if num_tasks == 0:
            return

        # 创建成本矩阵
        cost_matrix = np.zeros((num_robots, num_tasks))

        for i, robot in enumerate(idle_robots):
            robot_pos = robot.position
            for j, task_id in enumerate(task_queue[:num_tasks]):
                task = self.tasks[task_id - 1]

                # 计算基础距离成本
                distance_cost = self.manhattan_distance(robot_pos, task.start)

                # 计算等待时间惩罚
                waiting_penalty = -5 * min(1.0, (self.global_time - task.open_time) / 50.0)

                # 计算紧急程度奖励
                urgency_bonus = -5 * max(0, 1.0 - (task.deadline - self.global_time) / 200.0)

                # 计算优先级奖励
                priority_bonus = -10 * task.priority

                # 任务链奖励
                chain_bonus = -15 if task_id in self.task_tracker.task_chains else 0

                # 总成本
                cost_matrix[i, j] = (distance_cost +
                                     waiting_penalty +
                                     urgency_bonus +
                                     priority_bonus +
                                     chain_bonus)

        # 应用匈牙利算法
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # 执行分配
        for i, j in zip(row_ind, col_ind):
            if j < len(task_queue):
                robot_id = idle_robots[i].id
                task_id = task_queue[j]
                self._assign_single_task(robot_id, task_id)

    def _greedy_assignment(self, idle_robots: List[Robot]):
        """使用贪心算法进行任务分配"""
        for robot in idle_robots:
            if not self.task_queues.open:
                break

            best_task_id = None
            best_cost = float('inf')

            for task_id in self.task_queues.open[:10]:  # 只考虑前10个高优先级任务
                task = self.tasks[task_id - 1]

                # 计算成本
                distance_cost = self.manhattan_distance(robot.position, task.start)
                waiting_cost = self.global_time - task.open_time
                priority_bonus = -1000 * task.priority  # 优先级越高，成本越低

                # 计算总成本
                total_cost = distance_cost + waiting_cost + priority_bonus

                # 如果是任务链中的任务，给予额外奖励
                if task_id in self.task_tracker.task_chains:
                    total_cost -= 500

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_task_id = task_id

            if best_task_id is not None:
                self._assign_single_task(robot.id, best_task_id)

    def _assign_single_task(self, robot_id: int, task_id: int) -> bool:
        """分配单个任务给指定机器人"""
        robot = next(r for r in self.robots if r.id == robot_id)
        task = self.tasks[task_id - 1]

        if robot.state != 'idle' or task.state != 'open':
            return False

        # 规划路径
        path, path_length = self.a_star_planner(robot.position, task.start)
        if not path:
            return False

        # 更新机器人状态
        robot.state = 'moving'
        robot.target = task.start
        robot.path = path
        robot.path_index = 0
        robot.current_task_id = task_id
        robot.last_update_time = self.global_time

        # 更新任务状态
        task.state = 'assigned'
        task.assigned_to = robot_id
        task.assign_time = self.global_time
        task.waiting_time = self.global_time - task.open_time

        # 更新任务队列
        self.task_queues.open.remove(task_id)
        self.task_queues.assigned.append(task_id)

        # 记录分配
        self.task_assignments.append({
            'time': self.global_time,
            'task_id': task_id,
            'robot_id': robot_id,
            'priority': task.priority,
            'waiting_time': task.waiting_time,
            'path_length': path_length
        })

        print(f"Time {self.global_time}: Robot {robot_id} assigned to task {task_id} "
              f"[Priority: {task.priority:.2f}, Waiting: {task.waiting_time}]")

        return True

    def a_star_planner(self, start: Position, goal: Position) -> Tuple[List[Position], int]:
        """A*路径规划算法"""
        if start.x == goal.x and start.y == goal.y:
            return [start], 0

        # 检查起点和终点的有效性
        if not self._is_valid_position(start) or not self._is_valid_position(goal):
            return [], 0

        # 初始化数据结构
        closed_set = set()
        open_set = {(start.x, start.y)}
        came_from = {}

        g_score = defaultdict(lambda: float('inf'))
        f_score = defaultdict(lambda: float('inf'))

        g_score[(start.x, start.y)] = 0
        f_score[(start.x, start.y)] = self.manhattan_distance(start, goal)

        # 定义8个方向的移动
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 上右下左
            (1, 1), (1, -1), (-1, -1), (-1, 1)  # 对角线
        ]

        while open_set:
            current = min(open_set, key=lambda pos: f_score[pos])
            current_pos = Position(current[0], current[1])

            if current[0] == goal.x and current[1] == goal.y:
                # 重建路径
                path = []
                while current in came_from:
                    path.append(Position(current[0], current[1]))
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path, len(path)

            open_set.remove(current)
            closed_set.add(current)

            # 检查所有邻居
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                if not (0 <= neighbor[0] < SimConfig.MAP_SIZE and
                        0 <= neighbor[1] < SimConfig.MAP_SIZE):
                    continue

                if neighbor in closed_set:
                    continue

                # 检查是否是障碍物或其他不可通行区域
                if self.map_matrix[neighbor[0], neighbor[1]] in [
                    SimConfig.AREA_TYPES['boundary'],
                    SimConfig.AREA_TYPES['combing'],
                    SimConfig.AREA_TYPES['drawing1'],
                    SimConfig.AREA_TYPES['drawing2'],
                    SimConfig.AREA_TYPES['roving']
                ]:
                    continue

                # 计算移动代价
                if abs(dx) == 1 and abs(dy) == 1:
                    tentative_g_score = g_score[current] + 1.414  # 对角线移动代价
                else:
                    tentative_g_score = g_score[current] + 1.0  # 直线移动代价

                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g_score >= g_score[neighbor]:
                    continue

                # 更新路径
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = (g_score[neighbor] +
                                     self.manhattan_distance(Position(neighbor[0], neighbor[1]), goal))

        return [], 0

    def manhattan_distance(self, pos1: Position, pos2: Position) -> int:
        """计算曼哈顿距离"""
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)

    def _is_valid_position(self, pos: Position) -> bool:
        """检查位置是否有效"""
        if not (0 <= pos.x < SimConfig.MAP_SIZE and 0 <= pos.y < SimConfig.MAP_SIZE):
            return False

        return self.map_matrix[pos.x, pos.y] not in [
            SimConfig.AREA_TYPES['boundary'],
            SimConfig.AREA_TYPES['combing'],
            SimConfig.AREA_TYPES['drawing1'],
            SimConfig.AREA_TYPES['drawing2'],
            SimConfig.AREA_TYPES['roving']
        ]

    def update_robot_status(self):
        """更新所有机器人的状态"""
        for robot in self.robots:
            elapsed_time = self.global_time - robot.last_update_time

            if robot.state in ['moving', 'working']:
                if robot.current_task_id == 0:
                    self._reset_robot(robot)
                    continue

                task = self.tasks[robot.current_task_id - 1]

                # 处理机器人移动
                if robot.path and robot.path_index < len(robot.path):
                    self._handle_robot_movement(robot, elapsed_time)
                else:
                    # 到达目标位置后的处理
                    if robot.state == 'moving':
                        self._start_task_execution(robot)
                    elif robot.state == 'working':
                        self._complete_task(robot)

            robot.last_update_time = self.global_time

    def _reset_robot(self, robot: Robot):
        """重置机器人状态"""
        robot.state = 'idle'
        robot.path = []
        robot.current_task_id = 0
        robot.path_index = 0
        robot.last_update_time = self.global_time

    def _handle_robot_movement(self, robot: Robot, elapsed_time: int):
        """处理机器人移动"""
        steps_to_move = min(elapsed_time, len(robot.path) - robot.path_index)

        if steps_to_move > 0:
            # 更新机器人位置
            old_pos = robot.position
            new_index = min(robot.path_index + steps_to_move, len(robot.path) - 1)
            new_pos = robot.path[new_index]

            # 检查路径是否被阻塞
            if self._check_path_blocked(old_pos, new_pos):
                # 重新规划路径
                if robot.state == 'moving':
                    task = self.tasks[robot.current_task_id - 1]
                    new_path, _ = self.a_star_planner(robot.position, task.start)
                else:
                    task = self.tasks[robot.current_task_id - 1]
                    new_path, _ = self.a_star_planner(robot.position, task.end)

                if new_path:
                    robot.path = new_path
                    robot.path_index = 0
                    return

            # 更新地图
            self.map_matrix[old_pos.x, old_pos.y] = SimConfig.AREA_TYPES['empty']
            self.map_matrix[new_pos.x, new_pos.y] = SimConfig.AREA_TYPES['robot']

            # 更新机器人状态
            robot.position = new_pos
            robot.path_index = new_index + 1

    def _check_path_blocked(self, start: Position, end: Position) -> bool:
        """检查路径是否被阻塞"""
        # 检查直线路径上的点
        dx = end.x - start.x
        dy = end.y - start.y
        steps = max(abs(dx), abs(dy))

        if steps == 0:
            return False

        for i in range(1, steps):
            x = start.x + dx * i // steps
            y = start.y + dy * i // steps

            if self.map_matrix[x, y] not in [
                SimConfig.AREA_TYPES['empty'],
                SimConfig.AREA_TYPES['robot']
            ]:
                return True

        return False

    def _start_task_execution(self, robot: Robot):
        """开始执行任务"""
        task = self.tasks[robot.current_task_id - 1]

        # 更新机器人状态
        robot.state = 'working'
        robot.target = task.end

        # 规划到任务终点的路径
        path, _ = self.a_star_planner(robot.position, task.end)
        robot.path = path
        robot.path_index = 0

        # 记录任务开始时间
        task.start_time = self.global_time

        print(f"Time {self.global_time}: Robot {robot.id} started executing task {task.id}")

    def _complete_task(self, robot: Robot):
        """处理任务完成"""
        task = self.tasks[robot.current_task_id - 1]

        # 记录任务完成情况
        robot.completed_tasks.append(task.id)
        task.state = 'completed'
        task.completion_time = self.global_time
        task.execution_time = self.global_time - task.start_time
        task.total_time = self.global_time - task.open_time

        # 更新任务队列
        if task.id in self.task_queues.assigned:
            self.task_queues.assigned.remove(task.id)
        self.task_queues.completed.append(task.id)

        # 创建任务记录
        task_record = TaskRecord(
            task_id=task.id,
            task_type=task.type,
            start_time=task.start_time,
            completion_time=self.global_time,
            waiting_time=task.waiting_time,
            execution_time=task.execution_time,
            start_pos=(task.start.x, task.start.y),
            end_pos=(task.end.x, task.end.y)
        )

        # 记录任务完成
        self.task_tracker.record_task(robot.id, task_record)
        self.task_completions.append({
            'time': self.global_time,
            'task_id': task.id,
            'robot_id': robot.id,
            'execution_time': task.execution_time,
            'total_time': task.total_time
        })

        # 生成新任务（基于工艺流程）
        new_task_id = self.generate_dynamic_tasks(task)
        if new_task_id:
            print(f"Time {self.global_time}: Generated new task {new_task_id} "
                  f"following completion of task {task.id}")

        # 重置机器人状态
        self._reset_robot(robot)

    def run(self):
        """主要的模拟循环"""
        print("\nStarting simulation...")
        print(f"Initial configuration:")
        print(f"Map size: {SimConfig.MAP_SIZE}x{SimConfig.MAP_SIZE}")
        print(f"Number of robots: {SimConfig.NUM_ROBOTS}")
        print(f"Initial tasks: {SimConfig.NUM_INITIAL_TASKS}")
        print("=================================")

        start_time = time.time()
        while not self._check_termination():
            # 时间步进
            self.global_time += SimConfig.TIME_UNIT

            # 更新任务优先级
            if self.global_time % SimConfig.PRIORITY_UPDATE_INTERVAL == 0:
                self.calculate_dynamic_priorities()

            # 更新机器人状态
            self.update_robot_status()

            # 分配任务
            self.assign_tasks()

            # 定期打印状态
            if self.global_time % 10 == 0:
                self.print_current_state()

            # 检查卡住的机器人
            self._check_stuck_robots()

        # 打印最终总结
        self.print_summary(start_time)

    def _check_termination(self) -> bool:
        """检查是否应该终止模拟"""
        # 检查是否所有任务都完成且所有机器人空闲
        all_tasks_completed = (
                not self.task_queues.open and
                not self.task_queues.assigned
        )

        all_robots_idle = all(robot.state == 'idle' for robot in self.robots)

        return all_tasks_completed and all_robots_idle

    def _check_stuck_robots(self):
        """检查并处理卡住的机器人"""
        for robot in self.robots:
            if (robot.state != 'idle' and
                    self.global_time - robot.last_update_time > 30):
                print(f"Warning: Robot {robot.id} stuck at position {robot.position}")

                # 尝试重新规划路径
                if robot.current_task_id != 0:
                    task = self.tasks[robot.current_task_id - 1]
                    target = task.start if robot.state == 'moving' else task.end
                    new_path, _ = self.a_star_planner(robot.position, target)

                    if new_path:
                        robot.path = new_path
                        robot.path_index = 0
                        robot.last_update_time = self.global_time
                    else:
                        # 如果无法重新规划，重置机器人
                        self._reset_robot(robot)

    def calculate_metrics(self) -> dict:
        """计算当前系统性能指标"""
        metrics = {
            'queue_status': {
                'open': len(self.task_queues.open),
                'assigned': len(self.task_queues.assigned),
                'completed': len(self.task_queues.completed),
                'total': len(self.task_queues.all)
            },
            'robot_status': {
                'idle': 0,
                'moving': 0,
                'working': 0
            },
            'task_statistics': {
                'type1': {'completed': 0, 'avg_time': 0.0},
                'type2': {'completed': 0, 'avg_time': 0.0},
                'type3': {'completed': 0, 'avg_time': 0.0}
            },
            'performance': {
                'avg_waiting_time': 0.0,
                'avg_execution_time': 0.0,
                'throughput': 0.0,
                'robot_utilization': 0.0
            },
            'task_chains': {
                'completed_chains': 0,
                'avg_chain_time': 0.0
            }
        }

        # 统计机器人状态
        for robot in self.robots:
            metrics['robot_status'][robot.state] += 1

        # 统计任务信息
        completed_tasks = [self.tasks[i - 1] for i in self.task_queues.completed]
        if completed_tasks:
            # 按类型统计
            for task in completed_tasks:
                type_key = f'type{task.type}'
                metrics['task_statistics'][type_key]['completed'] += 1
                metrics['task_statistics'][type_key]['avg_time'] += task.total_time

            # 计算平均时间
            for type_stats in metrics['task_statistics'].values():
                if type_stats['completed'] > 0:
                    type_stats['avg_time'] /= type_stats['completed']

            # 计算整体性能指标
            metrics['performance'].update({
                'avg_waiting_time': sum(t.waiting_time for t in completed_tasks) / len(completed_tasks),
                'avg_execution_time': sum(t.execution_time for t in completed_tasks) / len(completed_tasks),
                'throughput': len(completed_tasks) / self.global_time if self.global_time > 0 else 0,
                'robot_utilization': (metrics['robot_status']['moving'] +
                                      metrics['robot_status']['working']) / SimConfig.NUM_ROBOTS
            })

        # 统计任务链信息
        if self.task_tracker.task_chains:
            completed_chains = 0
            chain_times = []

            for start_task_id, chain in self.task_tracker.task_chains.items():
                if all(task_id in self.task_queues.completed for task_id in chain):
                    completed_chains += 1
                    # 计算链完成时间
                    start_time = self.tasks[start_task_id - 1].start_time
                    end_time = max(self.tasks[task_id - 1].completion_time
                                   for task_id in chain)
                    chain_times.append(end_time - start_time)

            if completed_chains > 0:
                metrics['task_chains'].update({
                    'completed_chains': completed_chains,
                    'avg_chain_time': sum(chain_times) / len(chain_times)
                })

        return metrics

    def print_current_state(self):
        """打印当前系统状态"""
        metrics = self.calculate_metrics()

        print(f"\n----- Time: {self.global_time} -----")

        # 打印队列状态
        print("\nQueue Status:")
        print(f"Open tasks: {metrics['queue_status']['open']}")
        print(f"Assigned tasks: {metrics['queue_status']['assigned']}")
        print(f"Completed tasks: {metrics['queue_status']['completed']}")

        # 打印机器人状态
        print("\nRobot Status:")
        print(f"Idle: {metrics['robot_status']['idle']}")
        print(f"Moving: {metrics['robot_status']['moving']}")
        print(f"Working: {metrics['robot_status']['working']}")
        print(f"Utilization: {metrics['performance']['robot_utilization'] * 100:.1f}%")

        # 打印性能指标
        print("\nPerformance Metrics:")
        print(f"Average waiting time: {metrics['performance']['avg_waiting_time']:.1f}")
        print(f"Average execution time: {metrics['performance']['avg_execution_time']:.1f}")
        print(f"Throughput: {metrics['performance']['throughput']:.3f} tasks/time unit")

        # 打印任务链状态
        if metrics['task_chains']['completed_chains'] > 0:
            print("\nTask Chain Status:")
            print(f"Completed chains: {metrics['task_chains']['completed_chains']}")
            print(f"Average chain completion time: {metrics['task_chains']['avg_chain_time']:.1f}")

    def print_summary(self, start_time: float):
        """打印模拟总结"""
        end_time = time.time()
        total_real_time = end_time - start_time
        metrics = self.calculate_metrics()

        print("\n========== Simulation Summary ==========")
        print(f"\nSimulation Time:")
        print(f"Total simulation time: {self.global_time} time units")
        print(f"Real execution time: {total_real_time:.2f} seconds")

        # 任务完成统计
        print("\nTask Completion Statistics:")
        print(f"Total tasks: {metrics['queue_status']['total']}")
        print(f"Completed tasks: {metrics['queue_status']['completed']}")
        print(f"Completion rate: {metrics['queue_status']['completed'] / metrics['queue_status']['total'] * 100:.1f}%")

        # 按任务类型统计
        print("\nTask Type Statistics:")
        for task_type, stats in metrics['task_statistics'].items():
            print(f"\n{task_type.upper()}:")
            print(f"Completed: {stats['completed']}")
            print(f"Average completion time: {stats['avg_time']:.1f}")

        # 任务链统计
        print("\nTask Chain Statistics:")
        print(f"Completed chains: {metrics['task_chains']['completed_chains']}")
        if metrics['task_chains']['completed_chains'] > 0:
            print(f"Average chain completion time: {metrics['task_chains']['avg_chain_time']:.1f}")

        # 机器人性能统计
        print("\nRobot Performance:")
        print("Individual robot statistics:")
        for robot in self.robots:
            completed_count = len(robot.completed_tasks)
            if completed_count > 0:
                print(f"\nRobot {robot.id}:")
                print(f"Tasks completed: {completed_count}")
                print(f"Average tasks per time unit: {completed_count / self.global_time:.3f}")

        # 系统性能指标
        print("\nSystem Performance Metrics:")
        print(f"Average waiting time: {metrics['performance']['avg_waiting_time']:.1f}")
        print(f"Average execution time: {metrics['performance']['avg_execution_time']:.1f}")
        print(f"Overall throughput: {metrics['performance']['throughput']:.3f} tasks/time unit")
        print(f"Average robot utilization: {metrics['performance']['robot_utilization'] * 100:.1f}%")

    def visualize_map(self):
        """可视化当前地图状态"""
        plt.figure(figsize=(15, 15))

        # 创建自定义颜色映射
        colors = {
            'empty': 'white',
            'boundary': 'black',
            'material_warehouse': 'yellow',
            'product_warehouse': 'orange',
            'charging': 'red',
            'combing': 'blue',
            'drawing1': 'green',
            'drawing2': 'purple',
            'roving': 'brown',
            'robot': 'magenta',
            'pickup': 'cyan'
        }

        # 创建地图副本用于可视化
        viz_map = self.map_matrix.copy()

        # 绘制地图
        plt.imshow(viz_map, cmap='tab20')

        # 添加图例
        legend_elements = [plt.Rectangle((0, 0), 1, 1, facecolor=color, label=name)
                           for name, color in colors.items()]
        plt.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1, 0.5))

        # 添加标题和轴标签
        plt.title('Factory Layout')
        plt.xlabel('X coordinate')
        plt.ylabel('Y coordinate')

        # 显示网格
        plt.grid(True)
        plt.show()

    def save_metrics_history(self):
        """保存性能指标历史记录"""
        metrics_history = {
            'time': self.global_time,
            'metrics': self.calculate_metrics(),
            'task_assignments': self.task_assignments,
            'task_completions': self.task_completions
        }

        # 可以选择将数据保存到文件
        # with open(f'simulation_metrics_{time.strftime("%Y%m%d_%H%M%S")}.json', 'w') as f:
        #     json.dump(metrics_history, f, indent=2)

        return metrics_history


def main():
    """主函数：运行整个系统模拟"""
    # 1. 创建并初始化系统
    print("Initializing system...")
    system = System()
    system.initialize()

    # 2. 显示初始地图
    print("\nInitial factory layout:")
    system.visualize_map()

    # 3. 运行模拟
    print("\nStarting simulation...")
    try:
        system.run()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")

    # 4. 显示最终地图
    print("\nFinal factory layout:")
    system.visualize_map()

    # 5. 分析任务链完成情况
    print("\nAnalyzing task chains...")
    completed_chains = 0
    total_chains = 0
    for start_task_id, chain in system.task_tracker.task_chains.items():
        total_chains += 1
        if all(task_id in system.task_queues.completed for task_id in chain):
            completed_chains += 1

        print(f"\nChain starting with task {start_task_id}:")
        for task_id in chain:
            task = system.tasks[task_id - 1]
            status = "Completed" if task_id in system.task_queues.completed else "Pending"
            print(f"  Task {task_id} ({status}): "
                  f"Type {task.type}, "
                  f"Time: {task.completion_time - task.start_time if status == 'Completed' else 'N/A'}")

    print(f"\nTask chain completion rate: {completed_chains}/{total_chains} "
          f"({completed_chains / total_chains * 100:.1f}%)")

    # 6. 分析机器人性能
    print("\nAnalyzing robot performance...")
    robot_stats = defaultdict(lambda: {'tasks': 0, 'distance': 0, 'time_working': 0})

    for robot in system.robots:
        stats = robot_stats[robot.id]
        tasks_completed = robot.completed_tasks

        # 统计完成的任务
        stats['tasks'] = len(tasks_completed)

        # 计算总移动距离
        for task_id in tasks_completed:
            task = system.tasks[task_id - 1]
            stats['distance'] += (system.manhattan_distance(task.start, task.end) +
                                  system.manhattan_distance(robot.start_position, task.start))
            stats['time_working'] += task.execution_time

    # 打印机器人统计
    for robot_id, stats in sorted(robot_stats.items()):
        efficiency = stats['tasks'] / stats['time_working'] if stats['time_working'] > 0 else 0
        print(f"\nRobot {robot_id}:")
        print(f"  Tasks completed: {stats['tasks']}")
        print(f"  Total distance: {stats['distance']}")
        print(f"  Time working: {stats['time_working']}")
        print(f"  Efficiency: {efficiency:.3f} tasks/time unit")

    # 7. 生成性能报告
    print("\nGenerating performance report...")
    metrics = system.calculate_metrics()

    print("\nOverall System Performance:")
    print(f"Total simulation time: {system.global_time}")
    print(f"Tasks completed: {metrics['queue_status']['completed']}")
    print(f"Average robot utilization: {metrics['performance']['robot_utilization'] * 100:.1f}%")
    print(f"System throughput: {metrics['performance']['throughput']:.3f} tasks/time unit")

    # 8. 任务类型分析
    print("\nTask Type Analysis:")
    for task_type in ['type1', 'type2', 'type3']:
        stats = metrics['task_statistics'][task_type]
        if stats['completed'] > 0:
            print(f"\n{task_type.upper()}:")
            print(f"  Completed: {stats['completed']}")
            print(f"  Average completion time: {stats['avg_time']:.1f}")

    # 9. 保存性能指标历史
    metrics_history = system.save_metrics_history()

    # 10. 可视化关键性能指标
    plt.figure(figsize=(15, 5))

    # 任务完成时间分布
    plt.subplot(131)
    completion_times = [task.completion_time - task.start_time
                        for task in system.tasks if task.state == 'completed']
    plt.hist(completion_times, bins=20)
    plt.title('Task Completion Time Distribution')
    plt.xlabel('Time')
    plt.ylabel('Count')

    # 机器人工作负载分布
    plt.subplot(132)
    workloads = [len(robot.completed_tasks) for robot in system.robots]
    plt.bar(range(1, len(workloads) + 1), workloads)
    plt.title('Robot Workload Distribution')
    plt.xlabel('Robot ID')
    plt.ylabel('Tasks Completed')

    # 任务类型分布
    plt.subplot(133)
    task_types = [task.type for task in system.tasks if task.state == 'completed']
    type_counts = Counter(task_types)
    plt.pie([type_counts.get(i, 0) for i in [1, 2, 3]],
            labels=['Type 1', 'Type 2', 'Type 3'],
            autopct='%1.1f%%')
    plt.title('Task Type Distribution')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # 添加必要的导入
    from collections import defaultdict, Counter
    import matplotlib.pyplot as plt
    import time
    import json

    # 设置随机种子以保证可重复性
    random.seed(42)
    np.random.seed(42)

    # 运行模拟
    try:
        main()
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback

        traceback.print_exc()