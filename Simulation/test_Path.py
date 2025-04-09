"""
基础路径规划与代价地图系统 V1.0
Base Path Planning and Cost Map System V1.0

功能/Features:
1. 基础代价地图管理
2. A*路径规划
3. 障碍物安全距离
4. 场景可视化

Author: [Your Name]
Date: 2024.1
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import heapq

@dataclass
class Position:
    """
    位置类，表示二维坐标
    Position class for 2D coordinates
    """
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
        """
        比较运算符，用于优先队列排序
        Comparison operator for priority queue sorting
        """
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) < (other.x, other.y)

class CostMap:
    """
    代价地图类
    Cost Map Class
    """
    def __init__(self, width: int, height: int):
        """
        初始化代价地图
        Initialize cost map

        Args:
            width (int): 地图宽度 Width of the map
            height (int): 地图高度 Height of the map
        """
        self.width = width
        self.height = height
        # 基础代价地图 Base cost map
        self.base_costs = np.ones((height, width), dtype=float)
        # 动态代价地图 Dynamic cost map
        self.dynamic_costs = np.zeros((height, width), dtype=float)
        # 安全距离 Safety margin
        self.safety_margin = 2
        # 障碍物列表 List of obstacles
        self.obstacles = set()

    def update_base_cost(self, pos: Position, cost: float) -> None:
        """
        更新基础代价
        Update base cost

        Args:
            pos (Position): 位置 Position
            cost (float): 代价值 Cost value
        """
        if self._is_valid_position(pos):
            self.base_costs[pos.y, pos.x] = cost
            if cost == float('inf'):
                self.obstacles.add(pos)
            self._update_safety_costs(pos)

    def add_dynamic_cost(self, pos: Position, cost: float) -> None:
        """
        添加动态代价
        Add dynamic cost

        Args:
            pos (Position): 位置 Position
            cost (float): 代价值 Cost value
        """
        if self._is_valid_position(pos):
            self.dynamic_costs[pos.y, pos.x] += cost

    def get_cost(self, pos: Position) -> float:
        """
        获取总代价
        Get total cost

        Args:
            pos (Position): 位置 Position

        Returns:
            float: 总代价值 Total cost value
        """
        if self._is_valid_position(pos):
            return self.base_costs[pos.y, pos.x] + self.dynamic_costs[pos.y, pos.x]
        return float('inf')

    def _is_valid_position(self, pos: Position) -> bool:
        """
        检查位置是否有效
        Check if position is valid

        Args:
            pos (Position): 位置 Position

        Returns:
            bool: 位置是否有效 Whether position is valid
        """
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    def _update_safety_costs(self, pos: Position) -> None:
        """
        更新安全距离代价
        Update safety margin costs

        Args:
            pos (Position): 位置 Position
        """
        if self.base_costs[pos.y, pos.x] == float('inf'):
            for dy in range(-self.safety_margin, self.safety_margin + 1):
                for dx in range(-self.safety_margin, self.safety_margin + 1):
                    new_pos = Position(pos.x + dx, pos.y + dy)
                    if self._is_valid_position(new_pos):
                        distance = (dx ** 2 + dy ** 2) ** 0.5
                        if distance > 0:  # 不更新障碍物本身
                            safety_cost = 1.0 / distance
                            self.dynamic_costs[new_pos.y, new_pos.x] += safety_cost

    def reset_dynamic_costs(self) -> None:
        """
        重置动态代价
        Reset dynamic costs
        """
        self.dynamic_costs.fill(0)
        # 重新应用所有障碍物的安全距离
        for obs in self.obstacles:
            self._update_safety_costs(obs)

class PathNode:
    """
    路径节点类，用于A*算法
    Path node class for A* algorithm
    """
    def __init__(self, pos: Position, f_score: float):
        self.pos = pos
        self.f_score = f_score

    def __lt__(self, other):
        if not isinstance(other, PathNode):
            return NotImplemented
        return self.f_score < other.f_score

class BasicPathPlanner:
    """
    基础路径规划器
    Basic Path Planner
    """
    def __init__(self, cost_map: CostMap):
        self.cost_map = cost_map
        self.directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 四个基本方向
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # 四个对角方向
        ]

    def plan_path(self, start: Position, goal: Position) -> Optional[List[Position]]:
        """
        A*路径规划
        A* path planning
        """
        if not (self.cost_map._is_valid_position(start) and
                self.cost_map._is_valid_position(goal)):
            return None

        # 使用PathNode来处理优先队列
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

                # 对角线移动的代价增加
                move_cost = cost * (2 ** 0.5 if dx != 0 and dy != 0 else 1)
                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, PathNode(neighbor, f_score[neighbor]))

        return None

    def _heuristic(self, pos: Position, goal: Position) -> float:
        """
        启发式函数（曼哈顿距离）
        Heuristic function (Manhattan distance)
        """
        return abs(pos.x - goal.x) + abs(pos.y - goal.y)

    def _reconstruct_path(self, came_from: dict, current: Position) -> List[Position]:
        """
        重建路径
        Reconstruct path
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

def visualize_scenario(cost_map: CostMap, path: Optional[List[Position]] = None,
                      start: Optional[Position] = None, goal: Optional[Position] = None):
    """
    可视化场景
    Visualize scenario
    """
    plt.figure(figsize=(10, 10))

    # 显示代价地图 Display cost map
    total_costs = cost_map.base_costs + cost_map.dynamic_costs
    plt.imshow(total_costs, cmap='YlOrRd')
    plt.colorbar(label='Cost')

    # 显示路径 Display path
    if path:
        path_x = [pos.x for pos in path]
        path_y = [pos.y for pos in path]
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Planned Path')

    # 显示起点和终点 Display start and goal
    if start:
        plt.plot(start.x, start.y, 'go', markersize=15, label='Start')
    if goal:
        plt.plot(goal.x, goal.y, 'ro', markersize=15, label='Goal')

    plt.grid(True)
    plt.legend()
    plt.title('Cost Map and Path Planning Visualization')
    plt.show()

def test_basic_scenario():
    """
    测试基本场景
    Test basic scenario
    """
    print("开始基础路径规划测试...")

    # 创建10x10的代价地图
    cost_map = CostMap(10, 10)

    # 添加障碍物
    obstacles = [
        Position(3, 3),
        Position(3, 4),
        Position(3, 5),
        Position(4, 3),
        Position(5, 3)
    ]

    print("添加障碍物...")
    for obs in obstacles:
        cost_map.update_base_cost(obs, float('inf'))

    # 创建路径规划器
    planner = BasicPathPlanner(cost_map)

    # 设置起点和终点
    start = Position(1, 1)
    goal = Position(8, 8)

    print(f"规划路径: 从 {start} 到 {goal}")

    # 规划路径
    path = planner.plan_path(start, goal)

    # 输出结果
    if path:
        print(f"找到路径，长度为: {len(path)}")
        print("路径点:", [(pos.x, pos.y) for pos in path])
    else:
        print("未找到有效路径")

    # 可视化结果
    print("显示可视化结果...")
    visualize_scenario(cost_map, path, start, goal)

if __name__ == "__main__":
    test_basic_scenario()