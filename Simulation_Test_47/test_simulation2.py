# -*- coding: utf-8 -*-
import heapq
import numpy as np
import networkx as nx
import unittest
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set


# ========================
# 基础数据类型
# ========================
class Position:
    """二维坐标类"""

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def __eq__(self, other) -> bool:
        return isinstance(other, Position) and self.x == other.x and self.y == other.y

    def __hash__(self) -> int:
        return hash((self.x, self.y))

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y) if isinstance(other, Position) else NotImplementedError


# ========================
# 代价地图与路径规划
# ========================
class CostMap:
    """代价地图类，支持静态与动态代价"""

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.base_costs = np.ones((height, width), dtype=float)
        self.dynamic_costs = np.zeros((height, width), dtype=float)
        self.safety_margin = 2
        self.obstacles = set()

    def update_base_cost(self, pos: Position, cost: float) -> None:
        if 0 <= pos.x < self.width and 0 <= pos.y < self.height:
            self.base_costs[pos.y, pos.x] = cost
            if cost == float('inf'):
                self.obstacles.add(pos)
            self._update_safety_costs(pos)

    def add_dynamic_cost(self, pos: Position, cost: float) -> None:
        if 0 <= pos.x < self.width and 0 <= pos.y < self.height:
            self.dynamic_costs[pos.y, pos.x] += cost

    def get_cost(self, pos: Position) -> float:
        return self.base_costs[pos.y, pos.x] + self.dynamic_costs[pos.y, pos.x] if self._is_valid(pos) else float('inf')

    def _is_valid(self, pos: Position) -> bool:
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    def _update_safety_costs(self, pos: Position) -> None:
        if self.base_costs[pos.y, pos.x] == float('inf'):
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    x, y = pos.x + dx, pos.y + dy
                    if 0 <= x < self.width and 0 <= y < self.height:
                        distance = (dx​ ** ​2 + dy​ ** ​2)​ ** ​0.5
                        if distance > 0:
                            self.dynamic_costs[y, x] += 1.0 / distance


class BasicPathPlanner:
    """A*路径规划器"""

    def __init__(self, cost_map: CostMap):
        self.cost_map = cost_map
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def plan_path(self, start: Position, goal: Position) -> Optional[List[Position]]:
        if not (self.cost_map.get_cost(start) < float('inf') and self.cost_map.get_cost(goal) < float('inf')):
            return None

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy in self.directions:
                neighbor = Position(current.x + dx, current.y + dy)
                if self.cost_map.get_cost(neighbor) >= float('inf'):
                    continue

                tentative_g = g_score[current] + self.cost_map.get_cost(neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def _heuristic(self, a: Position, b: Position) -> float:
        return abs(a.x - b.x) + abs(a.y - b.y)

    def _reconstruct_path(self, came_from: dict, current: Position) -> List[Position]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]


# ========================
# 冲突检测系统
# ========================
class Conflict:
    """冲突检测记录"""

    def __init__(self, robot1: int, robot2: int, pos: Position, time: int, collision_type: str = 'vertex'):
        self.robot1 = robot1
        self.robot2 = robot2
        self.pos = pos
        self.time = time
        self.type = collision_type


class ConflictDetector:
    """冲突检测器"""

    @staticmethod
    def detect_conflicts(paths: Dict[int, List[Position]]) -> List[Conflict]:
        conflicts = []
        robot_ids = list(paths.keys())
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                path1 = paths[robot_ids[i]]
                path2 = paths[robot_ids[j]]
                for t in range(max(len(path1), len(path2))):
                    pos1 = path1[min(t, len(path1) - 1)]
                    pos2 = path2[min(t, len(path2) - 1)]
                    if pos1 == pos2:
                        conflicts.append(Conflict(robot_ids[i], robot_ids[j], pos1, t, 'vertex'))
                    if t > 0:
                        prev1 = path1[min(t - 1, len(path1) - 1)]
                        prev2 = path2[min(t - 1, len(path2) - 1)]
                        if prev1 == pos2 and prev2 == pos1:
                            conflicts.append(Conflict(robot_ids[i], robot_ids[j], pos1, t, 'edge'))
        return conflicts


# ========================
# 死锁检测系统
# ========================
class DeadlockDetector:
    """死锁检测器"""

    def __init__(self):
        self.robots = {}
        self.resources = {}
        self.wait_graph = nx.DiGraph()

    def add_robot(self, robot_id: str):
        self.robots[robot_id] = {'resources': set(), 'requesting': None}

    def add_resource(self, resource_id: str, capacity: int = 1):
        self.resources[resource_id] = {'capacity': capacity, 'users': set()}

    def request(self, robot_id: str, resource_id: str):
        if resource_id not in self.resources:
            return False
        res = self.resources[resource_id]
        if robot_id in res['users']:
            return True
        if len(res['users']) < res['capacity']:
            res['users'].add(robot_id)
            self.robots[robot_id]['resources'].add(resource_id)
            return True
        self.wait_graph.add_edge(robot_id, next(iter(res['users'])))
        self.robots[robot_id]['requesting'] = resource_id
        return False

    def resolve(self):
        try:
            cycle = nx.find_cycle(self.wait_graph, orientation='original')
            victim = min(cycle[0][0], cycle[-1][1])
            for res in self.resources.values():
                if victim in res['users']:
                    res['users'].remove(victim)
            del self.robots[victim]['requesting']
            self.wait_graph.remove_node(victim)
            return victim
        except nx.NetworkXNoCycle:
            return None


# ========================
# 生产系统仿真器
# ========================
class ProductionSimulator:
    """完整生产系统仿真器"""

    def __init__(self):
        self.cost_map = CostMap(20, 20)
        self.cost_map.update_base_cost(Position(10, 10), float('inf'))
        self.robots = {i: Position(2 + i // 3, 2 + i % 3) for i in range(1, 4)}
        self.tasks = {i: ((2 + i // 3, 2 + i % 3), (7 + i // 3, 7 + i % 3)) for i in range(1, 4)}
        self.planner = BasicPathPlanner(self.cost_map)
        self.deadlock_detector = DeadlockDetector()
        for rid in self.robots:
            self.deadlock_detector.add_robot(str(rid))
        self.deadlock_detector.add_resource('machine1', 1)
        self.metrics = {
            'steps': 0, 'red_tasks': 0, 'green_tasks': 540,
            'yellow_tasks': 0, 'production_rate': 0.0
        }

    def step(self):
        self.metrics['steps'] += 1
        self._update_production()
        self._move_robots()
        self._check_deadlocks()
        return self.metrics['steps'] < 200

    def _update_production(self):
        if self.metrics['steps'] % 5 == 0:
            self.metrics['green_tasks'] = max(0, self.metrics['green_tasks'] - 10)
            self.metrics['yellow_tasks'] += 10
        if self.metrics['steps'] % 10 == 0:
            self.metrics['yellow_tasks'] = max(0, self.metrics['yellow_tasks'] - 2)
            self.metrics['red_tasks'] += 2

    def _move_robots(self):
        paths = self.planner.plan(self.tasks)
        for rid, path in paths.items():
            if path and self.robots[rid] == path[0]:
                self.robots[rid] = path[1]

    def _check_deadlocks(self):
        if loser := self.deadlock_detector.resolve():
            del self.tasks[loser]

    def generate_report(self):
        return f"""生产系统状态报告
步骤数: {self.metrics['steps']}
红桶任务: {self.metrics['red_tasks']}/15
设备利用率: {(self.metrics['green_tasks'] + self.metrics['yellow_tasks']) / 540:.2%}
"""


# ========================
# 测试系统
# ========================
class TestSystem(unittest.TestCase):
    def test_basic_operations(self):
        sim = ProductionSimulator()
        for _ in range(100):
            self.assertTrue(sim.step())
        self.assertLess(sim.metrics['steps'], 200)
        self.assertGreater(sim.metrics['red_tasks'], 10)


if __name__ == "__main__":
    # 运行仿真
    sim = ProductionSimulator()
    print("=== 仿真启动 ===")
    while sim.step():
        print(f"步骤 {sim.metrics['steps']}: {sim.generate_report()}")
    print("
          == = 最终报告 == = ")
    print(sim.generate_report())

    # 运行测试
    print("
          == = 测试执行 == = ")
    unittest.main(argv=[''], exit=False)