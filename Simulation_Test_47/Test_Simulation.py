import unittest
import heapq
import numpy as np
import networkx as nx
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set
from enum import Enum

# ---------------------------
# 基本数据类型与工具类
# ---------------------------
class Position:
    """二维坐标类"""
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def __eq__(self, other) -> bool:
        if not isinstance(other, Position):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self) -> int:
        return hash((self.x, self.y))

    def __str__(self) -> str:
        return f"Pos({self.x}, {self.y})"

    def __lt__(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) < (other.x, other.y)

# ---------------------------
# 代价地图与A*路径规划
# ---------------------------
class CostMap:
    """代价地图类，支持静态与动态代价"""
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.base_costs = np.ones((height, width), dtype=float)
        self.dynamic_costs = np.zeros((height, width), dtype=float)
        self.safety_margin = 2
        self.obstacles: Set[Position] = set()

    def update_base_cost(self, pos: Position, cost: float) -> None:
        if self._is_valid_position(pos):
            self.base_costs[pos.y, pos.x] = cost
            if cost == float('inf'):
                self.obstacles.add(pos)
            self._update_safety_costs(pos)

    def add_dynamic_cost(self, pos: Position, cost: float) -> None:
        if self._is_valid_position(pos):
            self.dynamic_costs[pos.y, pos.x] += cost

    def get_cost(self, pos: Position) -> float:
        if self._is_valid_position(pos):
            return self.base_costs[pos.y, pos.x] + self.dynamic_costs[pos.y, pos.x]
        return float('inf')

    def _is_valid_position(self, pos: Position) -> bool:
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    def _update_safety_costs(self, pos: Position) -> None:
        if self.base_costs[pos.y, pos.x] == float('inf'):
            for dy in range(-self.safety_margin, self.safety_margin + 1):
                for dx in range(-self.safety_margin, self.safety_margin + 1):
                    new_pos = Position(pos.x + dx, pos.y + dy)
                    if self._is_valid_position(new_pos):
                        distance = (dx ** 2 + dy ** 2) ** 0.5
                        if distance > 0:
                            safety_cost = 1.0 / distance
                            self.dynamic_costs[new_pos.y, new_pos.x] += safety_cost

    def reset_dynamic_costs(self) -> None:
        self.dynamic_costs.fill(0)
        for obs in self.obstacles:
            self._update_safety_costs(obs)

class PathNode:
    """A*搜索使用的节点"""
    def __init__(self, pos: Position, f_score: float):
        self.pos = pos
        self.f_score = f_score

    def __lt__(self, other):
        if not isinstance(other, PathNode):
            return NotImplemented
        return self.f_score < other.f_score

class BasicPathPlanner:
    """使用A*算法基础路径规划器"""
    def __init__(self, cost_map: CostMap):
        self.cost_map = cost_map
        self.directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

    def plan_path(self, start: Position, goal: Position) -> Optional[List[Position]]:
        if not (self.cost_map._is_valid_position(start) and self.cost_map._is_valid_position(goal)):
            return None

        open_set = []
        heapq.heappush(open_set, PathNode(start, self._heuristic(start, goal)))
        came_from: Dict[Position, Position] = {}
        g_score: Dict[Position, float] = {start: 0}
        f_score: Dict[Position, float] = {start: self._heuristic(start, goal)}

        while open_set:
            current_node = heapq.heappop(open_set)
            current = current_node.pos

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy in self.directions:
                neighbor = Position(current.x + dx, current.y + dy)
                if not self.cost_map._is_valid_position(neighbor):
                    continue

                cost = self.cost_map.get_cost(neighbor)
                if cost == float('inf'):
                    continue

                move_cost = cost * (2 ** 0.5 if dx != 0 and dy != 0 else 1)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, PathNode(neighbor, f_score[neighbor]))
        return None

    def _heuristic(self, pos: Position, goal: Position) -> float:
        return abs(pos.x - goal.x) + abs(pos.y - goal.y)

    def _reconstruct_path(self, came_from: Dict[Position, Position], current: Position) -> List[Position]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

# ---------------------------
# 冲突检测模块
# ---------------------------
@dataclass
class Conflict:
    robot1_id: int
    robot2_id: int
    pos: Position
    time_step: int
    type: str = 'vertex'  # vertex或edge
    pos1_prev: Optional[Position] = None
    pos2_prev: Optional[Position] = None

    def __str__(self):
        if self.type == 'vertex':
            return f"顶点冲突: 机器人{self.robot1_id}与{self.robot2_id}在时刻{self.time_step}于{self.pos}发生冲突"
        else:
            return f"边冲突: 机器人{self.robot1_id}与{self.robot2_id}在时刻{self.time_step}沿交叉路径发生冲突"

class ConflictDetector:
    """冲突检测工具"""
    @staticmethod
    def detect_vertex_conflict(pos1: Position, pos2: Position) -> bool:
        return pos1 == pos2

    @staticmethod
    def detect_edge_conflict(pos1_curr: Position, pos1_prev: Position,
                             pos2_curr: Position, pos2_prev: Position) -> bool:
        return pos1_curr == pos2_prev and pos2_curr == pos1_prev

    @staticmethod
    def detect_conflicts(robot_paths: Dict[int, List[Position]]) -> List[Conflict]:
        conflicts = []
        robot_ids = list(robot_paths.keys())
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                id1 = robot_ids[i]
                id2 = robot_ids[j]
                path1 = robot_paths[id1]
                path2 = robot_paths[id2]
                max_time = max(len(path1), len(path2))
                for t in range(max_time):
                    pos1 = path1[min(t, len(path1) - 1)]
                    pos2 = path2[min(t, len(path2) - 1)]
                    if ConflictDetector.detect_vertex_conflict(pos1, pos2):
                        conflicts.append(Conflict(id1, id2, pos1, t, type='vertex'))
                        continue
                    if t > 0:
                        pos1_prev = path1[min(t - 1, len(path1) - 1)]
                        pos2_prev = path2[min(t - 1, len(path2) - 1)]
                        if ConflictDetector.detect_edge_conflict(pos1, pos1_prev, pos2, pos2_prev):
                            conflicts.append(Conflict(id1, id2, pos1, t, type='edge',
                                                        pos1_prev=pos1_prev, pos2_prev=pos2_prev))
        return conflicts

# ---------------------------
# CBS路径规划简易实现
# ---------------------------
class CBSPlanner:
    """简易的CBS路径规划器，内置低层A*规划，若检测到冲突，则为冲突机器人增加等待时间"""
    def __init__(self, planner: BasicPathPlanner):
        self.planner = planner

    def plan_paths(self, robot_tasks: Dict[int, Tuple[Position, Position]]) -> Dict[int, List[Position]]:
        paths = {}
        for r, (start, goal) in robot_tasks.items():
            p = self.planner.plan_path(start, goal)
            paths[r] = p if p is not None else [start]
        conflicts = ConflictDetector.detect_conflicts(paths)
        if conflicts:
            for conflict in conflicts:
                # 简单策略：选择编号较大的机器人等待一时刻
                loser = max(conflict.robot1_id, conflict.robot2_id)
                paths[loser] = [paths[loser][0]] + paths[loser]
        return paths

# ---------------------------
# 死锁检测模块（简易版）
# ---------------------------
@dataclass
class SimRobot:
    id: str
    current_resources: Set[str] = field(default_factory=set)
    requested_resource: Optional[str] = None

@dataclass
class Resource:
    id: str
    capacity: int = 1
    occupied_by: Set[str] = field(default_factory=set)
    waiting_robots: List[str] = field(default_factory=list)

class DeadlockDetector:
    def __init__(self):
        self.robots: Dict[str, SimRobot] = {}
        self.resources: Dict[str, Resource] = {}
        self.wait_graph = nx.DiGraph()
        self.deadlock_history: List[Tuple[str, Set[str]]] = []

    def add_robot(self, robot: SimRobot) -> None:
        self.robots[robot.id] = robot

    def add_resource(self, resource: Resource) -> None:
        self.resources[resource.id] = resource

    def request_resource(self, robot_id: str, resource_id: str) -> bool:
        if resource_id not in self.resources or robot_id not in self.robots:
            return False
        robot = self.robots[robot_id]
        resource = self.resources[resource_id]
        if resource_id in robot.current_resources:
            return True
        if len(resource.occupied_by) < resource.capacity:
            resource.occupied_by.add(robot_id)
            robot.current_resources.add(resource_id)
            return True
        else:
            if robot_id not in resource.waiting_robots:
                resource.waiting_robots.append(robot_id)
            robot.requested_resource = resource_id
            self._update_wait_graph()
            return False

    def _update_wait_graph(self) -> None:
        self.wait_graph.clear()
        for rid in self.robots:
            self.wait_graph.add_node(rid)
        for rid, robot in self.robots.items():
            if robot.requested_resource:
                resource = self.resources[robot.requested_resource]
                for blocking in resource.occupied_by:
                    self.wait_graph.add_edge(rid, blocking)

    def detect_deadlock(self) -> Tuple[str, Set[str]]:
        cycles = list(nx.simple_cycles(self.wait_graph))
        if cycles:
            involved = set().union(*cycles)
            deadlock_type = "DIRECT" if any(len(cycle) == 2 for cycle in cycles) else "INDIRECT"
            return (deadlock_type, involved)
        return ("NONE", set())

    def resolve_deadlock(self) -> List[str]:
        d_type, involved = self.detect_deadlock()
        if d_type == "NONE":
            return []
        self.deadlock_history.append((d_type, involved))
        victim = min(involved)
        for resource in self.resources.values():
            if victim in resource.waiting_robots:
                resource.waiting_robots.remove(victim)
        self.robots[victim].requested_resource = None
        self._update_wait_graph()
        return [victim]

# ---------------------------
# 性能监控模块
# ---------------------------
@dataclass
class PerformanceMetrics:
    completion_rate: float = 0.0
    production_speed: float = 0.0
    deadlock_rate: float = 0.0
    resource_utilization: float = 0.0
    robot_utilization: float = 0.0
    task_success_rate: float = 0.0
    efficiency: float = 0.0

# ---------------------------
# 生产系统与仿真整合
# ---------------------------
class ProductionSimulation:
    def __init__(self):
        # 初始化代价地图及区域（忽略充电区域）
        self.cost_map = CostMap(20, 20)
        # 定义区域（格式：(x1, y1, x2, y2)）
        self.work_area = (5, 5, 15, 15)
        self.pickup_area = (0, 0, 4, 4)
        self.storage_area = (16, 16, 19, 19)
        # 添加部分障碍物
        obstacles = [Position(10, 10), Position(10, 11), Position(11, 10)]
        for obs in obstacles:
            self.cost_map.update_base_cost(obs, float('inf'))
        # 初始化基础路径规划器与CBS规划器
        self.basic_planner = BasicPathPlanner(self.cost_map)
        self.cbs_planner = CBSPlanner(self.basic_planner)
        # 初始化机器人任务：3个机器人从取料区到工作区
        self.robot_tasks: Dict[int, Tuple[Position, Position]] = {
            1: (Position(2, 2), Position(7, 7)),
            2: (Position(3, 3), Position(8, 8)),
            3: (Position(1, 1), Position(9, 9))
        }
        self.robot_positions: Dict[int, Position] = {rid: start for rid, (start, _) in self.robot_tasks.items()}
        self.robot_paths: Dict[int, List[Position]] = {}
        # 初始化生产任务：物料转换链（绿->黄->红）
        self.green_tasks = 540
        self.yellow_tasks = 0
        self.red_tasks = 0
        self.production_goal = 15  # 目标：15个红桶产品
        # 仿真时间参数
        self.simulation_time = 0
        self.max_steps = 200
        self.planned_time = 100  # 理论流水线理想完成时间（步数）
        # 初始化简易死锁检测系统（用于模拟设备资源争抢）
        self.deadlock_detector = DeadlockDetector()
        for rid in ["Robot1", "Robot2"]:
            self.deadlock_detector.add_robot(SimRobot(rid))
        resource = Resource("Machine1", capacity=1)
        self.deadlock_detector.add_resource(resource)
        # 初始化性能指标
        self.metrics = PerformanceMetrics()

    def update_production_chain(self):
        # 每5步将10个绿桶任务转为黄桶任务（如果充足）
        if self.simulation_time % 5 == 0 and self.green_tasks >= 10:
            self.green_tasks -= 10
            self.yellow_tasks += 10
        # 每10步将2个黄桶任务转为红桶任务
        if self.simulation_time % 10 == 0 and self.yellow_tasks >= 2:
            self.yellow_tasks -= 2
            self.red_tasks += 2

    def update_robot_paths(self):
        # 如果机器人到达目标，则重新分配新任务（从取料区到工作区）
        new_robot_tasks = {}
        for rid, (start, goal) in self.robot_tasks.items():
            pos = self.robot_positions[rid]
            if pos == goal:
                new_start = Position((self.pickup_area[0] + self.pickup_area[2]) // 2,
                                     (self.pickup_area[1] + self.pickup_area[3]) // 2)
                new_goal = Position((self.work_area[0] + self.work_area[2]) // 2,
                                    (self.work_area[1] + self.work_area[3]) // 2)
                new_robot_tasks[rid] = (new_start, new_goal)
                self.robot_positions[rid] = new_start
            else:
                new_robot_tasks[rid] = (pos, goal)
        self.robot_tasks = new_robot_tasks
        self.robot_paths = self.cbs_planner.plan_paths(self.robot_tasks)

    def move_robots(self):
        # 每步沿路径前进一步（如果路径存在）
        for rid, path in self.robot_paths.items():
            if not path or len(path) < 2:
                continue
            next_pos = path[1]
            self.robot_positions[rid] = next_pos
            self.robot_paths[rid] = path[1:]

    def update_deadlock(self):
        # 模拟：位于工作区域的机器人请求设备资源
        for rid, pos in self.robot_positions.items():
            if self.work_area[0] <= pos.x <= self.work_area[2] and self.work_area[1] <= pos.y <= self.work_area[3]:
                self.deadlock_detector.request_resource("Robot1", "Machine1")
        d_type, involved = self.deadlock_detector.detect_deadlock()
        if d_type != "NONE":
            self.deadlock_detector.resolve_deadlock()

    def update_performance_metrics(self):
        self.metrics.production_speed = self.red_tasks / (self.simulation_time + 1)
        self.metrics.completion_rate = self.red_tasks / self.production_goal if self.production_goal else 0
        self.metrics.efficiency = self.simulation_time / self.planned_time if self.planned_time > 0 else 0

    def run(self):
        for step in range(self.max_steps):
            self.simulation_time = step
            self.update_production_chain()
            self.update_robot_paths()
            self.move_robots()
            self.update_deadlock()
            self.update_performance_metrics()
            if self.red_tasks >= self.production_goal:
                break
        return {
            "simulation_time": self.simulation_time,
            "green_tasks": self.green_tasks,
            "yellow_tasks": self.yellow_tasks,
            "red_tasks": self.red_tasks,
            "metrics": self.metrics
        }

# ---------------------------
# 综合系统测试（集成测试）
# ---------------------------
class TestIntegratedProductionSystem(unittest.TestCase):
    def test_simulation(self):
        sim = ProductionSimulation()
        result = sim.run()
        print("Simulation Result:")
        print(f"Simulation Time: {result['simulation_time']}")
        print(f"Green Tasks Remaining: {result['green_tasks']}")
        print(f"Yellow Tasks Remaining: {result['yellow_tasks']}")
        print(f"Red Tasks Completed: {result['red_tasks']}")
        print("Performance Metrics:")
        print(f"Completion Rate: {result['metrics'].completion_rate:.2f}")
        print(f"Production Speed: {result['metrics'].production_speed:.2f} tasks/step")
        print(f"Efficiency: {result['metrics'].efficiency:.2f}")
        # 验证生产目标达到
        self.assertGreaterEqual(result['red_tasks'], 15, "生产目标未达成")
        # 验证效率值：不超过2倍理论规划时间
        self.assertLessEqual(result['metrics'].efficiency, 2.0, "效率过低")

if __name__ == "__main__":
    unittest.main(verbosity=2)