import unittest
import heapq
import numpy as np
import networkx as nx
from collections import defaultdict
from dataclasses import dataclass, field
from typing import List, Dict, Set, Optional, Tuple
from enum import Enum
import matplotlib.pyplot as plt


# ---------------------------
# 共同基础类定义
# ---------------------------

@dataclass
class Position:
    """二维坐标类"""
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

    def __lt__(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) < (other.x, other.y)


# ---------------------------
# 代价地图与A*路径规划
# ---------------------------

class CostMap:
    """代价地图类，包含基础与动态代价，并考虑障碍物及安全距离"""

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.base_costs = np.ones((height, width), dtype=float)
        self.dynamic_costs = np.zeros((height, width), dtype=float)
        self.safety_margin = 2
        self.obstacles = set()

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
    """A*算法使用的路径节点"""

    def __init__(self, pos: Position, f_score: float):
        self.pos = pos
        self.f_score = f_score

    def __lt__(self, other):
        if not isinstance(other, PathNode):
            return NotImplemented
        return self.f_score < other.f_score


class BasicPathPlanner:
    """基础路径规划器，使用A*算法进行路径搜索"""

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
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

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

    def _reconstruct_path(self, came_from: dict, current: Position) -> List[Position]:
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
    """表示路径冲突信息，支持顶点冲突和边缘冲突"""
    robot1_id: int
    robot2_id: int
    pos: Position
    time_step: int
    type: str = 'vertex'  # 'vertex' 或 'edge'
    pos1_prev: Optional[Position] = None
    pos2_prev: Optional[Position] = None

    def __str__(self):
        if self.type == 'vertex':
            return (f"顶点冲突: 机器人{self.robot1_id}与{self.robot2_id}在时刻{self.time_step}于{self.pos}"
                    f"发生冲突")
        else:
            return (f"边缘冲突: 机器人{self.robot1_id}与{self.robot2_id}在时刻{self.time_step}沿路径交叉冲突")


class ConflictDetector:
    """多机器人路径冲突检测器"""

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
                    # 顶点冲突
                    if ConflictDetector.detect_vertex_conflict(pos1, pos2):
                        conflicts.append(Conflict(id1, id2, pos1, t, type='vertex'))
                        continue
                    # 边缘冲突
                    if t > 0:
                        pos1_prev = path1[min(t - 1, len(path1) - 1)]
                        pos2_prev = path2[min(t - 1, len(path2) - 1)]
                        if ConflictDetector.detect_edge_conflict(pos1, pos1_prev, pos2, pos2_prev):
                            conflicts.append(Conflict(id1, id2, pos1, t, type='edge',
                                                      pos1_prev=pos1_prev, pos2_prev=pos2_prev))
        return conflicts


# ---------------------------
# 死锁检测与解决模块
# ---------------------------

class DeadlockType(Enum):
    DIRECT = "direct"  # 直接死锁
    INDIRECT = "indirect"  # 间接死锁
    RESOURCE = "resource"  # 资源死锁
    NONE = "none"  # 无死锁


class ResourceType(Enum):
    MACHINE = "machine"
    STORAGE = "storage"
    PATH = "path"
    CHARGING = "charging"


@dataclass
class Resource:
    """资源类，表示生产系统中某种可共享的资源"""
    id: str
    type: ResourceType
    capacity: int = 1
    occupied_by: Set[str] = None
    waiting_robots: List[str] = None

    def __post_init__(self):
        if self.occupied_by is None:
            self.occupied_by = set()
        if self.waiting_robots is None:
            self.waiting_robots = []

    @property
    def is_full(self) -> bool:
        return len(self.occupied_by) >= self.capacity


@dataclass
class Robot:
    """机器人类，为死锁检测提供基本属性"""
    id: str
    current_resources: Set[str] = None
    requested_resource: Optional[str] = None

    def __post_init__(self):
        if self.current_resources is None:
            self.current_resources = set()


class DeadlockDetector:
    """死锁检测器，构建等待图以检测资源死锁并支持解决策略"""

    def __init__(self):
        self.resources: Dict[str, Resource] = {}
        self.robots: Dict[str, Robot] = {}
        self.wait_graph = nx.DiGraph()
        self.deadlock_history: List[Tuple[DeadlockType, Set[str]]] = []

    def add_resource(self, resource: Resource) -> None:
        self.resources[resource.id] = resource

    def add_robot(self, robot: Robot) -> None:
        self.robots[robot.id] = robot

    def request_resource(self, robot_id: str, resource_id: str) -> bool:
        if resource_id not in self.resources or robot_id not in self.robots:
            return False

        robot = self.robots[robot_id]
        resource = self.resources[resource_id]

        # 如果机器人已占用该资源
        if resource_id in robot.current_resources:
            return True

        # 如果资源未满，则直接分配
        if not resource.is_full and robot_id not in resource.waiting_robots:
            resource.occupied_by.add(robot_id)
            robot.current_resources.add(resource_id)
            return True
        else:
            if robot_id not in resource.waiting_robots:
                resource.waiting_robots.append(robot_id)
            robot.requested_resource = resource_id
            self._update_wait_graph()
            return False

    def release_resource(self, robot_id: str, resource_id: str) -> bool:
        if resource_id not in self.resources or robot_id not in self.robots:
            return False

        resource = self.resources[resource_id]
        robot = self.robots[robot_id]

        if robot_id not in resource.occupied_by:
            return False

        resource.occupied_by.remove(robot_id)
        robot.current_resources.remove(resource_id)

        if resource.waiting_robots and not resource.is_full:
            next_robot_id = resource.waiting_robots.pop(0)
            next_robot = self.robots[next_robot_id]
            resource.occupied_by.add(next_robot_id)
            next_robot.current_resources.add(resource_id)
            next_robot.requested_resource = None

        self._update_wait_graph()
        return True

    def _update_wait_graph(self) -> None:
        self.wait_graph.clear()
        for robot_id in self.robots:
            self.wait_graph.add_node(robot_id)
        for robot_id, robot in self.robots.items():
            if robot.requested_resource:
                resource = self.resources[robot.requested_resource]
                for blocking_robot in resource.occupied_by:
                    self.wait_graph.add_edge(robot_id, blocking_robot)

    def detect_deadlock(self) -> Tuple[DeadlockType, Set[str]]:
        try:
            cycles = list(nx.simple_cycles(self.wait_graph))
        except Exception:
            cycles = []
        if cycles:
            involved = set().union(*[set(cycle) for cycle in cycles])
            if any(len(cycle) == 2 for cycle in cycles):
                return DeadlockType.DIRECT, involved
            return DeadlockType.INDIRECT, involved

        for resource in self.resources.values():
            if len(resource.waiting_robots) > 2 * resource.capacity:
                return DeadlockType.RESOURCE, set(resource.waiting_robots)
        return DeadlockType.NONE, set()

    def resolve_deadlock(self) -> List[str]:
        d_type, involved = self.detect_deadlock()
        if d_type == DeadlockType.NONE:
            return []
        self.deadlock_history.append((d_type, involved))
        affected = []
        if d_type in [DeadlockType.DIRECT, DeadlockType.INDIRECT]:
            robot_to_backoff = next(iter(involved))
            affected.append(robot_to_backoff)
            robot = self.robots[robot_to_backoff]
            if robot.requested_resource:
                resource = self.resources[robot.requested_resource]
                if robot_to_backoff in resource.waiting_robots:
                    resource.waiting_robots.remove(robot_to_backoff)
                robot.requested_resource = None
        elif d_type == DeadlockType.RESOURCE:
            for resource in self.resources.values():
                if len(resource.waiting_robots) > 2 * resource.capacity:
                    robots_to_remove = resource.waiting_robots[:resource.capacity]
                    affected.extend(robots_to_remove)
                    for r_id in robots_to_remove:
                        resource.waiting_robots.remove(r_id)
                        self.robots[r_id].requested_resource = None
        self._update_wait_graph()
        return affected


# ---------------------------
# 综合测试用例：路径规划、冲突检测与死锁检测
# ---------------------------

class TestIntegratedSystem(unittest.TestCase):
    def setUp(self) -> None:
        # 初始化代价地图，设置地图尺寸、添加障碍物
        self.cost_map = CostMap(10, 10)
        obstacles = [
            Position(3, 3),
            Position(3, 4),
            Position(3, 5),
            Position(4, 3),
            Position(5, 3)
        ]
        for obs in obstacles:
            self.cost_map.update_base_cost(obs, float('inf'))
        self.planner = BasicPathPlanner(self.cost_map)

        # 定义多个机器人的起点和目标点
        self.robot_positions = {
            1: (Position(1, 1), Position(8, 8)),
            2: (Position(8, 1), Position(1, 8)),
            3: (Position(1, 8), Position(8, 1))
        }
        # 根据A*算法规划路径（每个机器人一个路径）
        self.robot_paths = {}
        for r_id, (start, goal) in self.robot_positions.items():
            path = self.planner.plan_path(start, goal)
            self.robot_paths[r_id] = path

    def test_path_planning(self):
        """测试所有机器人是否能规划出有效路径"""
        for r_id, path in self.robot_paths.items():
            self.assertIsNotNone(path, f"机器人 {r_id} 无法规划出路径")
            self.assertGreater(len(path), 0, f"机器人 {r_id} 的路径为空")
            print(f"机器人 {r_id} 的路径: {[str(p) for p in path]}")

    def test_conflict_detection(self):
        """测试机器人路径之间的冲突检测"""
        conflicts = ConflictDetector.detect_conflicts(self.robot_paths)
        print("检测到的冲突:")
        for conflict in conflicts:
            print(conflict)
        # 根据规划路径，预期至少检测到一个冲突（顶点或边缘冲突）
        self.assertGreaterEqual(len(conflicts), 1, "未检测到预期的冲突")

    def test_deadlock_detection_and_resolution(self):
        """构造直接死锁场景，并测试死锁检测与解决"""
        detector = DeadlockDetector()
        # 添加2个共享资源
        detector.add_resource(Resource("R1", ResourceType.MACHINE))
        detector.add_resource(Resource("R2", ResourceType.MACHINE))
        # 添加2个机器人
        for i in range(1, 3):
            detector.add_robot(Robot(f"Robot{i}"))
        # 构造死锁：Robot1先占有R1，Robot2先占有R2，
        # 然后Robot1请求R2，Robot2请求R1，形成直接死锁
        detector.request_resource("Robot1", "R1")
        detector.request_resource("Robot2", "R2")
        detector.request_resource("Robot1", "R2")
        detector.request_resource("Robot2", "R1")

        d_type, involved = detector.detect_deadlock()
        print(f"检测到死锁：类型={d_type.value}, 涉及机器人={involved}")
        self.assertEqual(d_type, DeadlockType.DIRECT, "未检测到预期的直接死锁")

        # 调用死锁解决策略
        affected = detector.resolve_deadlock()
        print(f"解决死锁后回退的机器人: {affected}")
        new_type, _ = detector.detect_deadlock()
        self.assertEqual(new_type, DeadlockType.NONE, "死锁未正确解决")


if __name__ == "__main__":
    unittest.main(verbosity=2)