"""
智能工厂多机器人协同调度仿真系统 V1.0
Intelligent Factory Multi-Robot Coordination Simulation System V1.0

功能特性:
1. 多机器人路径规划与任务分配
2. 基于CBS的冲突解决
3. 生产链任务管理
4. 实时性能监控
5. 可视化分析

Author: [Your Name]
Date: 2024.1
"""

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set
from enum import Enum
from collections import defaultdict
import heapq
import time
from datetime import datetime
import seaborn as sns
from matplotlib.animation import FuncAnimation
import json                 # 【新增】引入 json 模块以便保存结果
import traceback            # 【新增】引入 traceback 模块用于异常处理

# ---------------------------
# 基础数据类型定义
# ---------------------------
class AreaType(Enum):
    """区域类型定义"""
    EMPTY_BUCKET = "empty_bucket"      # 空桶区
    CARDING_WAITING = "carding_waiting"  # 梳棉待装填区
    CARDING_FILLED = "carding_filled"    # 梳棉已装填区
    DRAWING1_WAITING = "drawing1_waiting"  # 一并待装填区
    DRAWING1_FILLED = "drawing1_filled"    # 一并已装填区
    DRAWING2_WAITING = "drawing2_waiting"  # 二并待装填区
    DRAWING2_FILLED = "drawing2_filled"    # 二并已装填区
    ROVING_MATERIAL = "roving_material"    # 粗纱原料区
    CHARGING = "charging"                  # 充电区
    OBSTACLE = "obstacle"                  # 障碍物区域

class MaterialType(Enum):
    """物料类型定义"""
    EMPTY = "empty"    # 空桶
    GREEN = "green"    # 绿桶
    YELLOW = "yellow"  # 黄桶
    RED = "red"       # 红桶

class MachineType(Enum):
    """机器类型定义"""
    CARDING = "carding"    # 梳棉机
    DRAWING1 = "drawing1"  # 一并机
    DRAWING2 = "drawing2"  # 二并机
    ROVING = "roving"     # 粗纱机

@dataclass
class Position:
    """位置类，表示二维坐标"""
    x: int
    y: int

    def __eq__(self, other) -> bool:
        if not isinstance(other, Position):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self) -> int:
        return hash((self.x, self.y))

    def __str__(self) -> str:
        return f"Pos({self.x}, {self.y})"

    def distance_to(self, other: 'Position') -> float:
        """计算到另一个位置的欧氏距离"""
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

@dataclass
class Area:
    """区域类，定义工厂中的功能区域"""
    type: AreaType
    top_left: Position      # 区域左上角坐标
    bottom_right: Position  # 区域右下角坐标
    capacity: int = 0       # 区域容量
    current_count: int = 0  # 当前数量

    def contains(self, pos: Position) -> bool:
        """检查位置是否在区域内"""
        return (self.top_left.x <= pos.x <= self.bottom_right.x and
                self.top_left.y <= pos.y <= self.bottom_right.y)

    def get_random_position(self) -> Position:
        """获取区域内的随机位置"""
        x = np.random.randint(self.top_left.x, self.bottom_right.x + 1)
        y = np.random.randint(self.top_left.y, self.bottom_right.y + 1)
        return Position(x, y)

@dataclass
class Material:
    """物料类，表示生产系统中的物料"""
    id: int
    type: MaterialType
    position: Position
    state: str = "waiting"  # waiting, processing, completed
    batch_id: Optional[int] = None

@dataclass
class Robot:
    """机器人类，表示运输机器人"""
    id: int
    position: Position
    battery: float = 100.0  # 电池电量
    state: str = "idle"     # idle, moving, charging
    current_task: Optional['ProductionTask'] = None
    path: List[Position] = field(default_factory=list)
    total_distance: float = 0.0
    working_time: float = 0.0
    completed_tasks: int = 0

    def needs_charging(self) -> bool:
        """检查是否需要充电"""
        return self.battery < 20.0

    def update_statistics(self, distance: float, time: float) -> None:
        """更新统计数据"""
        self.total_distance += distance
        self.working_time += time
        if self.current_task and self.current_task.state == "completed":
            self.completed_tasks += 1


# ---------------------------
# 地图系统
# ---------------------------
class FactoryMap:
    """工厂地图类，管理整个工厂的布局和区域"""

    def __init__(self, width: int = 1000, height: int = 1000):
        self.width = width
        self.height = height
        self.areas: Dict[AreaType, Area] = {}
        self.machines: Dict[MachineType, List[Position]] = {
            MachineType.CARDING: [],  # 10台梳棉机
            MachineType.DRAWING1: [],  # 5台一并机
            MachineType.DRAWING2: [],  # 5台二并机
            MachineType.ROVING: []  # 5台粗纱机
        }
        self.obstacles: Set[Position] = set()
        self._initialize_areas()
        self._place_machines()

    def _initialize_areas(self) -> None:
        """初始化工厂各个功能区域"""
        # 定义各区域位置和容量
        area_configs = [
            # 区域类型, 左上角坐标, 右下角坐标, 容量
            (AreaType.EMPTY_BUCKET, Position(50, 50), Position(150, 150), 600),
            (AreaType.CARDING_WAITING, Position(200, 50), Position(300, 150), 100),
            (AreaType.CARDING_FILLED, Position(350, 50), Position(450, 150), 100),
            (AreaType.DRAWING1_WAITING, Position(500, 50), Position(600, 150), 100),
            (AreaType.DRAWING1_FILLED, Position(650, 50), Position(750, 150), 50),
            (AreaType.DRAWING2_WAITING, Position(800, 50), Position(900, 150), 50),
            (AreaType.DRAWING2_FILLED, Position(50, 200), Position(150, 300), 20),
            (AreaType.ROVING_MATERIAL, Position(200, 200), Position(300, 300), 20),
            (AreaType.CHARGING, Position(900, 900), Position(950, 950), 10)
        ]

        for area_type, top_left, bottom_right, capacity in area_configs:
            self.areas[area_type] = Area(area_type, top_left, bottom_right, capacity)

    def _place_machines(self) -> None:
        """放置各类机器"""
        # 定义各类机器的数量和基础位置
        machine_configs = [
            (MachineType.CARDING, 10, Position(200, 100)),  # 10台梳棉机
            (MachineType.DRAWING1, 5, Position(500, 100)),  # 5台一并机
            (MachineType.DRAWING2, 5, Position(800, 100)),  # 5台二并机
            (MachineType.ROVING, 5, Position(200, 250))  # 5台粗纱机
        ]

        for machine_type, count, base_pos in machine_configs:
            for i in range(count):
                # 机器间隔50个单位
                pos = Position(base_pos.x + i * 50, base_pos.y)
                self.machines[machine_type].append(pos)
                self.obstacles.add(pos)  # 机器位置作为障碍物

    def is_valid_position(self, pos: Position) -> bool:
        """检查位置是否有效（在地图范围内且不是障碍物）"""
        return (0 <= pos.x < self.width and
                0 <= pos.y < self.height and
                pos not in self.obstacles)


# ---------------------------
# 任务管理系统
# ---------------------------
@dataclass
class ProductionTask:
    """生产任务类，定义单个运输任务"""
    id: int
    material: Material
    start_pos: Position
    end_pos: Position
    priority: float = 0.0
    state: str = "pending"  # pending, assigned, executing, completed
    assigned_robot_id: Optional[int] = None
    start_time: Optional[float] = None
    completion_time: Optional[float] = None


@dataclass
class ProductionBatch:
    """生产批次类，定义一批物料的生产过程"""
    id: int
    input_materials: List[Material]
    output_type: MaterialType
    machine_type: MachineType
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    state: str = "waiting"  # waiting, processing, completed
    assigned_machine_id: Optional[int] = None


class ProductionChainManager:
    """生产链管理器，管理整个生产过程"""

    def __init__(self):
        # 各机器处理时间（秒）
        self.processing_times = {
            MachineType.CARDING: 4225.94,  # 梳棉机处理时间
            MachineType.DRAWING1: 2594.75,  # 一并机处理时间
            MachineType.DRAWING2: 2594.75  # 二并机处理时间
        }

        # 各机器批次大小
        self.batch_sizes = {
            MachineType.CARDING: 1,  # 梳棉机一次处理1个
            MachineType.DRAWING1: 12,  # 一并机一次处理12个
            MachineType.DRAWING2: 12  # 二并机一次处理12个
        }

        # 物料转换比率
        self.conversion_rates = {
            MachineType.CARDING: 1,  # 1:1转换
            MachineType.DRAWING1: 6,  # 6:1转换
            MachineType.DRAWING2: 6  # 6:1转换
        }

        # 当前物料数量
        self.material_counts = {
            MaterialType.EMPTY: 540,  # 初始空桶数量
            MaterialType.GREEN: 0,  # 初始绿桶数量
            MaterialType.YELLOW: 0,  # 初始黄桶数量
            MaterialType.RED: 0  # 初始红桶数量
        }

        self.batches: List[ProductionBatch] = []
        self.current_batch_id = 0

    def create_initial_materials(self, factory_map: FactoryMap) -> List[Material]:
        """创建初始物料"""
        materials = []
        empty_area = factory_map.areas[AreaType.EMPTY_BUCKET]

        for i in range(self.material_counts[MaterialType.EMPTY]):
            pos = empty_area.get_random_position()
            materials.append(Material(
                id=i,
                type=MaterialType.EMPTY,
                position=pos
            ))

        return materials

    def create_batch(self, input_materials: List[Material],
                     machine_type: MachineType) -> ProductionBatch:
        """创建生产批次"""
        self.current_batch_id += 1

        # 根据机器类型确定输出物料类型
        output_type = {
            MachineType.CARDING: MaterialType.GREEN,
            MachineType.DRAWING1: MaterialType.YELLOW,
            MachineType.DRAWING2: MaterialType.RED
        }[machine_type]

        return ProductionBatch(
            id=self.current_batch_id,
            input_materials=input_materials,
            output_type=output_type,
            machine_type=machine_type
        )

    def update_material_counts(self, batch: ProductionBatch) -> None:
        """更新物料数量统计"""
        if batch.state == "completed":
            input_type = batch.input_materials[0].type
            output_type = batch.output_type
            input_count = len(batch.input_materials)
            output_count = input_count // self.conversion_rates[batch.machine_type]

            self.material_counts[input_type] -= input_count
            self.material_counts[output_type] += output_count


# ---------------------------
# 路径规划系统
# ---------------------------
@dataclass
class PathNode:
    """路径节点类"""
    position: Position
    g_cost: float = float('inf')  # 从起点到当前节点的实际代价
    h_cost: float = 0.0  # 从当前节点到终点的估计代价
    parent: Optional['PathNode'] = None
    time_step: int = 0

    @property
    def f_cost(self) -> float:
        """获取总代价"""
        return self.g_cost + self.h_cost

    def __lt__(self, other: 'PathNode') -> bool:
        """比较运算符，用于优先队列"""
        return self.f_cost < other.f_cost


class AStarPathPlanner:
    """A*路径规划器"""

    def __init__(self, cost_map: 'EnhancedCostMap'):
        self.cost_map = cost_map
        self.directions = [
            Position(0, 1),  # 上
            Position(1, 0),  # 右
            Position(0, -1),  # 下
            Position(-1, 0),  # 左
            Position(1, 1),  # 右上
            Position(1, -1),  # 右下
            Position(-1, 1),  # 左上
            Position(-1, -1)  # 左下
        ]

    def find_path(self, start: Position, goal: Position,
                  constraints: List['CBSConstraint']) -> List[Position]:
        """查找路径"""
        start_node = PathNode(position=start, g_cost=0)
        start_node.h_cost = self._calculate_heuristic(start, goal)

        open_set = [start_node]
        closed_set = set()
        time_constraints = self._group_constraints_by_time(constraints)

        while open_set:
            current = heapq.heappop(open_set)

            if current.position == goal:
                return self._reconstruct_path(current)

            closed_set.add((current.position, current.time_step))

            for direction in self.directions:
                next_pos = Position(
                    current.position.x + direction.x,
                    current.position.y + direction.y
                )

                if not self._is_valid_move(next_pos, current.time_step + 1,
                                           time_constraints):
                    continue

                new_g_cost = current.g_cost + self._calculate_move_cost(
                    current.position, next_pos
                )

                if (next_pos, current.time_step + 1) in closed_set:
                    continue

                next_node = PathNode(
                    position=next_pos,
                    g_cost=new_g_cost,
                    h_cost=self._calculate_heuristic(next_pos, goal),
                    parent=current,
                    time_step=current.time_step + 1
                )

                heapq.heappush(open_set, next_node)

        return []  # 没找到路径

    def _calculate_heuristic(self, pos: Position, goal: Position) -> float:
        """计算启发式值（曼哈顿距离）"""
        return abs(pos.x - goal.x) + abs(pos.y - goal.y)

    def _calculate_move_cost(self, from_pos: Position, to_pos: Position) -> float:
        """计算移动代价"""
        base_cost = 1.0
        # 考虑地图代价
        return base_cost * self.cost_map.get_total_cost(to_pos)

    def _is_valid_move(self, pos: Position, time_step: int,
                       time_constraints: Dict[int, Set[Position]]) -> bool:
        """检查移动是否有效"""
        if not self.cost_map._is_valid_position(pos):
            return False

        # 检查时间步约束
        if time_step in time_constraints and pos in time_constraints[time_step]:
            return False

        return True

    def _group_constraints_by_time(self,
                                   constraints: List['CBSConstraint']
                                   ) -> Dict[int, Set[Position]]:
        """按时间步骤对约束进行分组"""
        time_constraints: Dict[int, Set[Position]] = defaultdict(set)
        for constraint in constraints:
            time_constraints[constraint.time_step].add(constraint.position)
        return time_constraints

    def _reconstruct_path(self, end_node: PathNode) -> List[Position]:
        """重建路径"""
        path = []
        current = end_node
        while current:
            path.append(current.position)
            current = current.parent
        return path[::-1]


@dataclass
class CBSConstraint:
    """CBS约束类"""
    robot_id: int
    position: Position
    time_step: int


@dataclass
class Conflict:
    """冲突类"""
    robot1_id: int
    robot2_id: int
    pos: Position
    time_step: int
    conflict_type: str  # 'vertex' 或 'edge'
    pos1: Optional[Position] = None  # 用于边冲突
    pos2: Optional[Position] = None  # 用于边冲突


class CBSNode:
    """CBS节点类"""

    def __init__(self):
        self.constraints: List[CBSConstraint] = []
        self.solution: Dict[int, List[Position]] = {}
        self.cost: float = 0.0
        self.parent: Optional['CBSNode'] = None

    def add_constraint(self, constraint: CBSConstraint) -> None:
        """添加约束"""
        self.constraints.append(constraint)


class ConflictBasedSearch:
    """基于冲突的搜索算法实现"""

    def __init__(self, cost_map: 'EnhancedCostMap'):
        self.cost_map = cost_map
        self.low_level_planner = AStarPathPlanner(cost_map)
        self.time_limit = 30.0  # 规划时间限制（秒）

    def plan_paths(self, robots: Dict[int, Robot],
                   goals: Dict[int, Position]) -> Dict[int, List[Position]]:
        """为多个机器人规划路径"""
        root = CBSNode()
        start_time = time.time()

        # 初始化根节点的解
        for robot_id, robot in robots.items():
            if robot_id in goals:
                path = self.low_level_planner.find_path(
                    robot.position,
                    goals[robot_id],
                    [c for c in root.constraints if c.robot_id == robot_id]
                )
                if path:
                    root.solution[robot_id] = path
                    root.cost += len(path)

        if not self._is_solution_valid(root.solution):
            return self._resolve_conflicts(root, robots, goals, start_time)
        return root.solution

    def _is_solution_valid(self, solution: Dict[int, List[Position]]) -> bool:
        """简单检查解的合法性（可根据需要扩展）"""
        return True

    def _find_first_conflict(self, solution: Dict[int, List[Position]]) -> Optional[Conflict]:
        """查找第一个冲突（简化示例）"""
        # 此处未实现具体冲突检测逻辑，可根据需要扩展
        return None

    def _resolve_conflicts(self, root: CBSNode,
                           robots: Dict[int, Robot],
                           goals: Dict[int, Position],
                           start_time: float) -> Dict[int, List[Position]]:
        """解决冲突"""
        open_set = [root]

        while open_set and (time.time() - start_time) < self.time_limit:
            node = min(open_set, key=lambda n: n.cost)
            open_set.remove(node)

            conflict = self._find_first_conflict(node.solution)
            if not conflict:
                return node.solution

            # 为冲突的机器人创建新的约束
            for robot_id in [conflict.robot1_id, conflict.robot2_id]:
                child = CBSNode()
                child.parent = node
                child.constraints = node.constraints.copy()
                child.add_constraint(CBSConstraint(
                    robot_id=robot_id,
                    position=conflict.pos,
                    time_step=conflict.time_step
                ))

                # 重新规划受影响机器人的路径
                child.solution = node.solution.copy()
                new_path = self.low_level_planner.find_path(
                    robots[robot_id].position,
                    goals[robot_id],
                    [c for c in child.constraints if c.robot_id == robot_id]
                )

                if new_path:
                    child.solution[robot_id] = new_path
                    child.cost = sum(len(path) for path in child.solution.values())
                    open_set.append(child)

        # 如果超时，返回最好的解决方案
        return min(open_set, key=lambda n: n.cost).solution if open_set else root.solution


# ---------------------------
# 【新增】定义 EnhancedCostMap 类
# ---------------------------
class EnhancedCostMap:
    """增强版的代价地图，用于路径规划"""

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        # 初始化所有位置的代价为 1
        self.cost_grid = np.ones((height, width))

    def get_total_cost(self, pos: Position) -> float:
        """获取指定位置的总代价"""
        if 0 <= pos.x < self.width and 0 <= pos.y < self.height:
            return self.cost_grid[pos.y, pos.x]
        else:
            return float('inf')

    def _is_valid_position(self, pos: Position) -> bool:
        """检查位置是否在地图内"""
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    def decay_densities(self) -> None:
        """衰减动态障碍物的密度（简单实现）"""
        self.cost_grid = np.maximum(self.cost_grid - 0.1, 1)


# ---------------------------
# 【新增】定义 ProductionScheduler 类
# ---------------------------
class ProductionScheduler:
    """任务调度器，用于分配任务给机器人"""

    def __init__(self, factory_map: FactoryMap, chain_manager: ProductionChainManager):
        self.factory_map = factory_map
        self.chain_manager = chain_manager
        self.task_queue: List[ProductionTask] = []  # 此处可以扩展任务队列

    def get_next_task(self, robot: Robot) -> Optional[ProductionTask]:
        """获取下一个任务（目前返回 None，需扩展实现）"""
        # 这里可以根据需求设计任务分派逻辑
        return None


# ---------------------------
# 性能监控系统
# ---------------------------
class PerformanceMetrics:
    """性能指标监控类"""

    def __init__(self):
        self.production_metrics = {
            'completion_rate': 0.0,
            'production_speed': 0.0,
            'theoretical_time': 0.0,
            'actual_time': 0.0,
            'efficiency': 0.0
        }

        self.robot_metrics = {
            'utilization_rate': {},  # 机器人利用率
            'average_task_time': {},  # 平均任务时间
            'total_distance': {},  # 总行驶距离
            'completed_tasks': {},  # 完成任务数
            'battery_efficiency': {}  # 电池效率
        }

        self.system_metrics = {
            'deadlock_count': 0,  # 死锁次数
            'conflict_count': 0,  # 冲突次数
            'resource_utilization': {},  # 资源利用率
            'area_congestion': {}  # 区域拥堵度
        }

        self.time_series = defaultdict(list)  # 时间序列数据

    def update_production_metrics(self, current_materials: Dict[MaterialType, int],
                                  target_materials: Dict[MaterialType, int],
                                  simulation_time: float) -> None:
        """更新生产指标"""
        total_completed = current_materials[MaterialType.RED]
        total_target = target_materials[MaterialType.RED]

        self.production_metrics['completion_rate'] = (
            total_completed / total_target if total_target > 0 else 0
        )
        self.production_metrics['production_speed'] = (
            total_completed / simulation_time if simulation_time > 0 else 0
        )

        # 记录时间序列数据
        self.time_series['completion_rate'].append(
            self.production_metrics['completion_rate']
        )
        self.time_series['production_speed'].append(
            self.production_metrics['production_speed']
        )

    def update_robot_metrics(self, robots: Dict[int, Robot]) -> None:
        """更新机器人指标"""
        for robot_id, robot in robots.items():
            # 计算利用率
            total_time = robot.working_time + robot.total_distance
            self.robot_metrics['utilization_rate'][robot_id] = (
                robot.working_time / total_time if total_time > 0 else 0
            )

            # 更新其他指标
            self.robot_metrics['completed_tasks'][robot_id] = robot.completed_tasks
            self.robot_metrics['total_distance'][robot_id] = robot.total_distance

        # 记录平均指标
        self.time_series['average_utilization'].append(
            np.mean(list(self.robot_metrics['utilization_rate'].values()))
        )

    def generate_report(self) -> Dict:
        """生成性能报告"""
        return {
            'production_metrics': self.production_metrics,
            'robot_metrics': self.robot_metrics,
            'system_metrics': self.system_metrics,
            'time_series_data': dict(self.time_series)
        }


# ---------------------------
# 主仿真系统
# ---------------------------
class ProductionSimulation:
    """生产仿真系统主类"""

    def __init__(self):
        # 初始化地图和系统组件
        self.factory_map = FactoryMap(1000, 1000)
        self.cost_map = EnhancedCostMap(1000, 1000)
        self.chain_manager = ProductionChainManager()
        self.scheduler = ProductionScheduler(self.factory_map, self.chain_manager)
        self.cbs_planner = ConflictBasedSearch(self.cost_map)

        # 初始化机器人
        self.robots = {
            i: Robot(id=i, position=self.factory_map.areas[AreaType.CHARGING].get_random_position())
            for i in range(10)  # 创建10个机器人
        }

        # 初始化性能监控
        self.metrics = PerformanceMetrics()

        # 仿真参数
        self.simulation_time = 0.0
        self.time_step = 1.0  # 1秒/步
        self.max_steps = 100000

        # 理论完成时间计算
        self.theoretical_time = self._calculate_theoretical_time()

    def _calculate_theoretical_time(self) -> float:
        """计算理论完成时间"""
        # 计算各阶段理论时间
        carding_time = (540 * 4225.94) / 10  # 10台梳棉机并行工作
        drawing1_time = (90 * 2594.75) / 5  # 5台一并机并行工作
        drawing2_time = (15 * 2594.75) / 5  # 5台二并机并行工作

        # 考虑物料运输时间（估计值）
        transport_time = 1000  # 估计值

        return carding_time + drawing1_time + drawing2_time + transport_time

    def run(self) -> Dict:
        """运行仿真"""
        print("Starting simulation...")
        start_time = time.time()

        # 初始化物料
        materials = self.chain_manager.create_initial_materials(self.factory_map)

        while self.simulation_time < self.max_steps:
            # 更新系统状态
            self._update_system_state()

            # 检查任务完成情况
            if self.chain_manager.material_counts[MaterialType.RED] >= 15:
                break

            # 更新性能指标
            self._update_metrics()

            # 推进仿真时间
            self.simulation_time += self.time_step

            # 定期打印进度
            if int(self.simulation_time) % 100 == 0:
                self._print_progress()

        # 计算最终效率
        actual_time = time.time() - start_time
        self.metrics.production_metrics['theoretical_time'] = self.theoretical_time
        self.metrics.production_metrics['actual_time'] = actual_time
        self.metrics.production_metrics['efficiency'] = (
            self.theoretical_time / actual_time * 100 if actual_time > 0 else 0
        )

        print("Simulation completed.")
        return self._generate_final_report()

    def _update_system_state(self) -> None:
        """更新系统状态"""
        # 更新机器人状态和任务分配
        self._update_robots()

        # 更新生产批次状态
        self._update_production_batches()

        # 更新代价地图
        self.cost_map.decay_densities()

        # 检测和处理冲突
        self._handle_conflicts()

    def _update_robots(self) -> None:
        """更新机器人状态"""
        for robot in self.robots.values():
            if robot.state == "idle":
                # 分配新任务
                task = self.scheduler.get_next_task(robot)
                if task:
                    self._assign_task_to_robot(robot, task)
            elif robot.state == "moving":
                # 更新机器人位置
                self._update_robot_position(robot)

    def _print_progress(self) -> None:
        """打印仿真进度"""
        print(f"Simulation time: {self.simulation_time:.1f}s")
        print(f"Completed materials: {self.chain_manager.material_counts}")
        print(f"Active robots: {sum(1 for r in self.robots.values() if r.state != 'idle')}")

    def _update_metrics(self) -> None:
        """更新所有性能指标（示例函数，可扩展）"""
        self.metrics.update_robot_metrics(self.robots)
        self.metrics.update_production_metrics(
            self.chain_manager.material_counts,
            {MaterialType.RED: 15},  # 假定目标红桶数量为 15
            self.simulation_time
        )

    def _update_production_batches(self) -> None:
        """更新生产批次状态（示例函数，可扩展）"""
        pass

    def _handle_conflicts(self) -> None:
        """检测和处理冲突（示例函数，可扩展）"""
        pass

    # ---------------------------
    # 【新增】新增任务分配和机器人的位置更新函数
    # ---------------------------
    def _assign_task_to_robot(self, robot: Robot, task: ProductionTask) -> None:
        """为机器人分配任务（新增实现示例）"""
        robot.current_task = task
        robot.state = "moving"
        # 使用冲突检测算法规划路径
        path_dict = self.cbs_planner.plan_paths({robot.id: robot}, {robot.id: task.end_pos})
        # 如果路径规划成功，则为机器人分配路径
        if robot.id in path_dict:
            robot.path = path_dict[robot.id]
        else:
            robot.path = []

    def _update_robot_position(self, robot: Robot) -> None:
        """更新机器人的位置（新增实现示例）"""
        if robot.path:
            next_pos = robot.path.pop(0)
            distance = robot.position.distance_to(next_pos)
            robot.total_distance += distance
            robot.position = next_pos
            if not robot.path:
                robot.state = "idle"
                if robot.current_task:
                    robot.current_task.state = "completed"
        else:
            robot.state = "idle"

    def _generate_final_report(self) -> Dict:
        """生成最终的报告"""
        return {
            'simulation_time': self.simulation_time,
            'metrics': self.metrics.generate_report(),
            'material_counts': self.chain_manager.material_counts,
            'theoretical_time': self.theoretical_time,
            'actual_time': self.metrics.production_metrics['actual_time'],
            'efficiency': self.metrics.production_metrics['efficiency']
        }


# ---------------------------
# 可视化系统
# ---------------------------
class SimulationVisualizer:
    """仿真可视化器"""

    def __init__(self, simulation: ProductionSimulation):
        self.simulation = simulation
        plt.style.use('seaborn')

    def create_animation(self) -> FuncAnimation:
        """创建动画"""
        fig, self.axs = plt.subplots(2, 2, figsize=(15, 15))
        fig.suptitle('Factory Simulation Visualization', fontsize=16)

        anim = FuncAnimation(
            fig,
            self._update_frame,
            frames=range(int(self.simulation.simulation_time)),
            interval=50,
            blit=False
        )
        return anim

    def _update_frame(self, frame: int) -> None:
        """更新动画帧"""
        self._plot_factory_state(self.axs[0, 0])
        self._plot_material_flow(self.axs[0, 1])
        self._plot_robot_metrics(self.axs[1, 0])
        self._plot_performance_metrics(self.axs[1, 1])

    def _plot_factory_state(self, ax: plt.Axes) -> None:
        """绘制工厂状态"""
        ax.clear()
        ax.set_title('Factory Layout')

        # 绘制区域
        for area_type, area in self.simulation.factory_map.areas.items():
            rect = plt.Rectangle(
                (area.top_left.x, area.top_left.y),
                area.bottom_right.x - area.top_left.x,
                area.bottom_right.y - area.top_left.y,
                fill=False,
                label=area_type.value
            )
            ax.add_patch(rect)

        # 绘制机器
        for machine_type, positions in self.simulation.factory_map.machines.items():
            x_coords = [pos.x for pos in positions]
            y_coords = [pos.y for pos in positions]
            ax.scatter(x_coords, y_coords, marker='s', label=machine_type.value)

        # 绘制机器人
        for robot in self.simulation.robots.values():
            color = 'g' if robot.state == 'idle' else 'r'
            ax.scatter(robot.position.x, robot.position.y,
                       color=color, marker='o', s=100)

        ax.grid(True)
        ax.legend()

    def _plot_material_flow(self, ax: plt.Axes) -> None:
        """绘制物料流动情况（示例函数，可扩展）"""
        ax.clear()
        ax.set_title('Material Flow')
        ax.grid(True)

    def _plot_robot_metrics(self, ax: plt.Axes) -> None:
        """绘制机器人指标（示例函数，可扩展）"""
        ax.clear()
        ax.set_title('Robot Metrics')
        ax.grid(True)

    def _plot_performance_metrics(self, ax: plt.Axes) -> None:
        """绘制系统性能指标（示例函数，可扩展）"""
        ax.clear()
        ax.set_title('Performance Metrics')
        ax.grid(True)

    def plot_final_results(self) -> None:
        """绘制最终结果"""
        plt.figure(figsize=(15, 10))

        # 绘制完成率随时间变化
        plt.subplot(2, 2, 1)
        plt.plot(self.simulation.metrics.time_series['completion_rate'])
        plt.title('Completion Rate Over Time')
        plt.xlabel('Time Steps')
        plt.ylabel('Completion Rate')

        # 绘制机器人利用率
        plt.subplot(2, 2, 2)
        robot_util = self.simulation.metrics.robot_metrics['utilization_rate']
        plt.bar(range(len(robot_util)), list(robot_util.values()))
        plt.title('Robot Utilization Rates')
        plt.xlabel('Robot ID')
        plt.ylabel('Utilization Rate')

        # 绘制生产效率
        plt.subplot(2, 2, 3)
        efficiency = self.simulation.metrics.production_metrics['efficiency']
        plt.text(0.5, 0.5, f'Overall Efficiency: {efficiency:.2f}%',
                 horizontalalignment='center', verticalalignment='center')
        plt.title('Production Efficiency')

        # 绘制物料分布
        plt.subplot(2, 2, 4)
        material_counts = self.simulation.chain_manager.material_counts
        plt.pie(list(material_counts.values()), labels=[mt.value for mt in material_counts.keys()],
                autopct='%1.1f%%')
        plt.title('Material Distribution')

        plt.tight_layout()
        plt.show()


def save_results(simulation: ProductionSimulation, filename: str) -> None:
    """保存仿真结果"""
    results = {
        'simulation_time': simulation.simulation_time,
        'metrics': simulation.metrics.generate_report(),
        'material_counts': simulation.chain_manager.material_counts,
        'theoretical_time': simulation.theoretical_time,
        'actual_time': simulation.metrics.production_metrics['actual_time']
    }

    with open(filename, 'w') as f:
        json.dump(results, f, indent=4)


def main():
    """主函数"""
    # 设置随机种子确保可重复性
    np.random.seed(42)

    # 创建并运行仿真
    print("Initializing simulation...")
    simulation = ProductionSimulation()

    print("Running simulation...")
    results = simulation.run()

    # 创建可视化
    print("Generating visualization...")
    visualizer = SimulationVisualizer(simulation)
    visualizer.plot_final_results()

    # 保存结果
    print("Saving results...")
    save_results(simulation, 'simulation_results.json')

    # 打印关键指标
    print("\nSimulation Results:")
    print(f"Total simulation time: {simulation.simulation_time:.2f} seconds")
    print(f"Theoretical completion time: {simulation.theoretical_time:.2f} seconds")
    print(f"Actual completion time: {results['actual_time']:.2f} seconds")
    print(f"Overall efficiency: {results['efficiency']:.2f}%")
    print("\nFinal material counts:")
    for material_type, count in simulation.chain_manager.material_counts.items():
        print(f"{material_type.value}: {count}")

    print("\nSimulation completed successfully!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error occurred: {e}")
        traceback.print_exc()