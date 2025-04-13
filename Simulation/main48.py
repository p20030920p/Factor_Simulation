import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
import networkx as nx
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set, Any
from enum import Enum
import random
from collections import defaultdict, deque
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import matplotlib.font_manager as fm
from scipy.optimize import linear_sum_assignment
import threading
import os
import pickle
from datetime import datetime

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体显示中文
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


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

    def __repr__(self) -> str:
        return self.__str__()

    def __lt__(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) < (other.x, other.y)

    def distance_to(self, other) -> float:
        """计算到另一个位置的欧几里得距离"""
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def manhattan_distance(self, other) -> int:
        """计算到另一个位置的曼哈顿距离"""
        return abs(self.x - other.x) + abs(self.y - other.y)


class TaskType(Enum):
    """任务类型枚举"""
    EMPTY_TO_CARDING = 1  # 空桶到梳棉待装填区
    CARDING_TO_FIRST = 2  # 梳棉已装填区(绿桶)到一并待装填区
    FIRST_TO_SECOND = 3  # 一并已装填区(黄桶)到二并待装填区
    SECOND_TO_ROVING = 4  # 二并已装填区(红桶)到粗纱区

    @staticmethod
    def get_priority(task_type) -> int:
        """获取任务类型的优先级，数值越小优先级越高"""
        priority_map = {
            TaskType.SECOND_TO_ROVING: 0,  # 最高优先级
            TaskType.FIRST_TO_SECOND: 1,
            TaskType.CARDING_TO_FIRST: 2,
            TaskType.EMPTY_TO_CARDING: 3  # 最低优先级
        }
        return priority_map.get(task_type, 999)

    def __str__(self) -> str:
        name_map = {
            TaskType.EMPTY_TO_CARDING: "空桶→梳棉",
            TaskType.CARDING_TO_FIRST: "梳棉→一并",
            TaskType.FIRST_TO_SECOND: "一并→二并",
            TaskType.SECOND_TO_ROVING: "二并→粗纱"
        }
        return name_map.get(self, "未知任务")


class TaskStatus(Enum):
    """任务状态枚举"""
    PENDING = 0  # 等待分配
    ASSIGNED = 1  # 已分配给机器人
    IN_PROGRESS = 2  # 执行中
    COMPLETED = 3  # 已完成
    FAILED = 4  # 失败

    def __str__(self) -> str:
        name_map = {
            TaskStatus.PENDING: "待分配",
            TaskStatus.ASSIGNED: "已分配",
            TaskStatus.IN_PROGRESS: "执行中",
            TaskStatus.COMPLETED: "已完成",
            TaskStatus.FAILED: "失败"
        }
        return name_map.get(self, "未知状态")


class ZoneType(Enum):
    """区域类型枚举"""
    EMPTY_BUCKET = 0  # 空桶区
    CARDING_LOAD = 1  # 梳棉待装填区
    CARDING_UNLOAD = 2  # 梳棉已装填区(绿桶)
    FIRST_DRAWING_LOAD = 3  # 一并待装填区
    FIRST_DRAWING_UNLOAD = 4  # 一并已装填区(黄桶)
    SECOND_DRAWING_LOAD = 5  # 二并待装填区
    SECOND_DRAWING_UNLOAD = 6  # 二并已装填区(红桶)
    ROVING = 7  # 粗纱区
    ROBOT_IDLE = 8  # 机器人空闲区

    def __str__(self) -> str:
        name_map = {
            ZoneType.EMPTY_BUCKET: "空桶区",
            ZoneType.CARDING_LOAD: "梳棉待装填区",
            ZoneType.CARDING_UNLOAD: "梳棉已装填区",
            ZoneType.FIRST_DRAWING_LOAD: "一并待装填区",
            ZoneType.FIRST_DRAWING_UNLOAD: "一并已装填区",
            ZoneType.SECOND_DRAWING_LOAD: "二并待装填区",
            ZoneType.SECOND_DRAWING_UNLOAD: "二并已装填区",
            ZoneType.ROVING: "粗纱区",
            ZoneType.ROBOT_IDLE: "机器人空闲区"
        }
        return name_map.get(self, "未知区域")


class MachineType(Enum):
    """机器类型枚举"""
    CARDING = 0  # 梳棉机
    FIRST_DRAWING = 1  # 一并机
    SECOND_DRAWING = 2  # 二并机
    ROVING = 3  # 粗纱机

    def __str__(self) -> str:
        name_map = {
            MachineType.CARDING: "梳棉机",
            MachineType.FIRST_DRAWING: "一并机",
            MachineType.SECOND_DRAWING: "二并机",
            MachineType.ROVING: "粗纱机"
        }
        return name_map.get(self, "未知机器")


class MachineStatus(Enum):
    """机器状态枚举"""
    IDLE = 0  # 空闲
    LOADING = 1  # 装载中
    PROCESSING = 2  # 加工中
    UNLOADING = 3  # 卸载中
    BREAKDOWN = 4  # 故障

    def __str__(self) -> str:
        status_map = {
            MachineStatus.IDLE: "空闲",
            MachineStatus.LOADING: "装载中",
            MachineStatus.PROCESSING: "加工中",
            MachineStatus.UNLOADING: "卸载中",
            MachineStatus.BREAKDOWN: "故障"
        }
        return status_map.get(self, "未知状态")


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
        self.safety_margin = 2  # 安全距离
        self.obstacles: Set[Position] = set()
        self.zone_map = np.zeros((height, width), dtype=int)  # 区域类型地图

    def update_base_cost(self, pos: Position, cost: float) -> None:
        """更新基础代价，用于设置障碍物等"""
        if self._is_valid_position(pos):
            self.base_costs[pos.y, pos.x] = cost
            if cost == float('inf'):
                self.obstacles.add(pos)
                self._update_safety_costs(pos)

    def add_dynamic_cost(self, pos: Position, cost: float) -> None:
        """添加动态代价，用于临时避让等"""
        if self._is_valid_position(pos):
            self.dynamic_costs[pos.y, pos.x] += cost

    def get_cost(self, pos: Position) -> float:
        """获取位置的总代价"""
        if self._is_valid_position(pos):
            return self.base_costs[pos.y, pos.x] + self.dynamic_costs[pos.y, pos.x]
        return float('inf')

    def _is_valid_position(self, pos: Position) -> bool:
        """检查位置是否有效"""
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    def _update_safety_costs(self, pos: Position) -> None:
        """更新障碍物周围的安全代价"""
        if pos in self.obstacles:
            for dy in range(-self.safety_margin, self.safety_margin + 1):
                for dx in range(-self.safety_margin, self.safety_margin + 1):
                    new_pos = Position(pos.x + dx, pos.y + dy)
                    if self._is_valid_position(new_pos) and new_pos not in self.obstacles:
                        distance = (dx ** 2 + dy ** 2) ** 0.5
                        if distance > 0:
                            # 安全代价随距离增加而减小
                            safety_cost = 5.0 / (distance ** 2)
                            self.dynamic_costs[new_pos.y, new_pos.x] += safety_cost

    def reset_dynamic_costs(self) -> None:
        """重置动态代价"""
        self.dynamic_costs.fill(0)
        for obs in self.obstacles:
            self._update_safety_costs(obs)

    def set_zone(self, pos: Position, zone_type: ZoneType) -> None:
        """设置位置所属的区域类型"""
        if self._is_valid_position(pos):
            self.zone_map[pos.y, pos.x] = zone_type.value

    def get_zone(self, pos: Position) -> Optional[ZoneType]:
        """获取位置所属的区域类型"""
        if self._is_valid_position(pos):
            zone_value = self.zone_map[pos.y, pos.x]
            return ZoneType(zone_value) if zone_value != 0 else None
        return None

    def set_zone_rect(self, x1: int, y1: int, x2: int, y2: int, zone_type: ZoneType) -> None:
        """设置矩形区域的区域类型"""
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                self.set_zone(Position(x, y), zone_type)


class PathNode:
    """A*搜索使用的节点"""

    def __init__(self, pos: Position, g_score: float, h_score: float):
        self.pos = pos
        self.g_score = g_score  # 起点到当前节点的代价
        self.h_score = h_score  # 启发式估计(当前节点到终点)
        self.f_score = g_score + h_score  # 总估计代价

    def __lt__(self, other):
        if not isinstance(other, PathNode):
            return NotImplemented
        return self.f_score < other.f_score


class PathPlanner:
    """基于A*算法的路径规划器"""

    def __init__(self, cost_map: CostMap):
        self.cost_map = cost_map
        # 八个方向移动(上下左右 + 四个对角线)
        self.directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

    def plan_path(self, start: Position, goal: Position) -> Optional[List[Position]]:
        """使用A*算法规划从起点到终点的路径"""
        if not (self.cost_map._is_valid_position(start) and self.cost_map._is_valid_position(goal)):
            return None

        # 处理起点或终点是障碍物的情况
        if self.cost_map.get_cost(start) == float('inf') or self.cost_map.get_cost(goal) == float('inf'):
            return None

        # 如果起点和终点相同，直接返回
        if start == goal:
            return [start]

        # A*算法的开放和关闭集
        open_set = []
        closed_set = set()
        heapq.heappush(open_set, PathNode(start, 0, self._heuristic(start, goal)))

        # 记录从起点到各节点的路径
        came_from: Dict[Position, Position] = {}
        g_score: Dict[Position, float] = {start: 0}

        while open_set:
            current_node = heapq.heappop(open_set)
            current = current_node.pos

            if current == goal:
                return self._reconstruct_path(came_from, current)

            closed_set.add(current)

            for dx, dy in self.directions:
                neighbor = Position(current.x + dx, current.y + dy)
                if not self.cost_map._is_valid_position(neighbor) or neighbor in closed_set:
                    continue

                # 获取移动到邻居的代价
                cost = self.cost_map.get_cost(neighbor)
                if cost == float('inf'):
                    continue

                # 对角线移动时需要确保两个相邻格子不是障碍物(防止穿墙)
                if dx != 0 and dy != 0:
                    corner1 = Position(current.x + dx, current.y)
                    corner2 = Position(current.x, current.y + dy)
                    if (self.cost_map.get_cost(corner1) == float('inf') or
                            self.cost_map.get_cost(corner2) == float('inf')):
                        continue

                # 对角线移动时距离要乘以根号2
                move_cost = cost * ((2 ** 0.5) if (dx != 0 and dy != 0) else 1)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)

                    # 检查节点是否已在开放集中
                    in_open_set = False
                    for i, node in enumerate(open_set):
                        if node.pos == neighbor:
                            in_open_set = True
                            # 如果发现更好的路径，更新节点
                            if f_score < node.f_score:
                                open_set[i] = PathNode(neighbor, tentative_g, self._heuristic(neighbor, goal))
                                heapq.heapify(open_set)
                            break

                    if not in_open_set:
                        heapq.heappush(open_set, PathNode(neighbor, tentative_g, self._heuristic(neighbor, goal)))

        # 没有找到路径
        return None

    def _heuristic(self, pos: Position, goal: Position) -> float:
        """计算启发式函数(曼哈顿距离)"""
        return pos.manhattan_distance(goal)

    def _reconstruct_path(self, came_from: Dict[Position, Position], current: Position) -> List[Position]:
        """重建从起点到终点的路径"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]  # 反转路径，从起点到终点


# ---------------------------
# 冲突检测与CBS路径规划
# ---------------------------
@dataclass
class Conflict:
    """路径冲突描述类"""
    robot1_id: int
    robot2_id: int
    pos: Position
    time_step: int
    type: str = 'vertex'  # 'vertex'(顶点冲突)或'edge'(边冲突)
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
        """检测顶点冲突(两机器人同时到达同一位置)"""
        return pos1 == pos2

    @staticmethod
    def detect_edge_conflict(pos1_curr: Position, pos1_prev: Position,
                             pos2_curr: Position, pos2_prev: Position) -> bool:
        """检测边冲突(两机器人交叉)"""
        return pos1_curr == pos2_prev and pos2_curr == pos1_prev

    @staticmethod
    def detect_conflicts(robot_paths: Dict[int, List[Position]]) -> List[Conflict]:
        """检测多机器人路径中的所有冲突"""
        conflicts = []
        robot_ids = list(robot_paths.keys())

        # 为每条路径不足的部分用终点填充
        max_path_length = max(len(path) for path in robot_paths.values()) if robot_paths else 0
        padded_paths = {}
        for rid, path in robot_paths.items():
            if not path:  # 空路径
                continue
            padded_path = path.copy()
            while len(padded_path) < max_path_length:
                padded_path.append(padded_path[-1])  # 用最后一个位置填充
            padded_paths[rid] = padded_path

        # 检测各时间步的冲突
        for i in range(len(robot_ids)):
            id1 = robot_ids[i]
            path1 = padded_paths.get(id1)
            if not path1:
                continue

            for j in range(i + 1, len(robot_ids)):
                id2 = robot_ids[j]
                path2 = padded_paths.get(id2)
                if not path2:
                    continue

                # 检测顶点冲突和边冲突
                for t in range(max_path_length):
                    pos1 = path1[t]
                    pos2 = path2[t]

                    # 顶点冲突
                    if ConflictDetector.detect_vertex_conflict(pos1, pos2):
                        conflicts.append(Conflict(id1, id2, pos1, t, type='vertex'))

                    # 边冲突(需要检查前一步的位置)
                    if t > 0:
                        pos1_prev = path1[t - 1]
                        pos2_prev = path2[t - 1]
                        if ConflictDetector.detect_edge_conflict(pos1, pos1_prev, pos2, pos2_prev):
                            conflicts.append(Conflict(id1, id2, pos1, t, 'edge', pos1_prev, pos2_prev))

        return conflicts


class CBSPlanner:
    """冲突基础搜索(CBS)路径规划器"""

    def __init__(self, base_planner: PathPlanner):
        self.base_planner = base_planner
        self.constraint_tree = nx.DiGraph()  # 约束树
        self.solution_paths = {}  # 最终求解的路径
        self.max_iterations = 100  # 最大迭代次数
        self.current_node_id = 0  # 约束树节点ID

    def plan_paths(self, robot_tasks: Dict[int, Tuple[Position, Position]],
                   robot_priorities: Optional[Dict[int, int]] = None) -> Dict[int, List[Position]]:
        """为多个机器人规划无冲突路径（增强版）"""
        # 初始规划: 为每个机器人规划路径，忽略冲突
        paths = {}
        for r_id, (start, goal) in robot_tasks.items():
            path = self.base_planner.plan_path(start, goal)
            if path:
                paths[r_id] = path
            else:
                # 如果找不到路径，保持原地不动
                paths[r_id] = [start]

        # 如果只有一个机器人，直接返回路径
        if len(paths) <= 1:
            return paths

        # 检测冲突
        conflicts = ConflictDetector.detect_conflicts(paths)

        # 如果没有冲突，直接返回路径
        if not conflicts:
            return paths

        # 如果提供了优先级，则按优先级处理冲突
        if robot_priorities:
            # 按优先级排序机器人 (优先级值越低，优先级越高)
            sorted_robots = sorted(paths.keys(), key=lambda rid: robot_priorities.get(rid, 999))

            # 优先级处理：从高到低依次固定路径
            final_paths = {}
            for rid in sorted_robots:
                # 将该机器人的路径添加到最终路径中
                final_paths[rid] = paths[rid]

                # 检查是否与已固定的路径冲突
                new_conflicts = []
                for other_rid in final_paths:
                    if rid != other_rid:
                        temp_conflicts = ConflictDetector.detect_conflicts(
                            {rid: paths[rid], other_rid: final_paths[other_rid]})
                        new_conflicts.extend(temp_conflicts)

                # 如果有冲突，为当前机器人重新规划路径
                if new_conflicts:
                    # 为每个冲突时间步添加约束
                    constraints = set()
                    for conflict in new_conflicts:
                        if conflict.robot1_id == rid:
                            # 避免在特定时间步到达冲突位置
                            constraints.add((conflict.time_step, conflict.pos))
                        elif conflict.robot2_id == rid:
                            constraints.add((conflict.time_step, conflict.pos))

                    # 使用约束重新规划路径
                    if constraints:
                        # 简化处理：为避开约束，给路径添加等待步骤
                        modified_path = [paths[rid][0]]  # 从起点开始

                        # 遍历原路径，根据约束添加等待
                        for i in range(1, len(paths[rid])):
                            # 检查当前步骤是否与约束冲突
                            if any((i, pos) in constraints for pos in [paths[rid][i]]):
                                # 冲突：再等待一步（重复上一个位置）
                                modified_path.append(modified_path[-1])

                            # 添加原路径中的下一步
                            modified_path.append(paths[rid][i])

                        final_paths[rid] = modified_path

            return final_paths

        # 按任务优先级排序冲突
        if robot_priorities:
            conflicts.sort(key=lambda c: min(
                robot_priorities.get(c.robot1_id, 999),
                robot_priorities.get(c.robot2_id, 999)
            ))

        # 简化的CBS方法：处理每个冲突，让优先级较低的机器人等待
        for conflict in conflicts:
            # 确定需要调整路径的机器人
            loser_id = conflict.robot1_id
            if robot_priorities:
                if robot_priorities.get(conflict.robot1_id, 999) < robot_priorities.get(conflict.robot2_id, 999):
                    loser_id = conflict.robot2_id
            else:
                # 如果没有提供优先级，使用ID较大的作为低优先级
                loser_id = max(conflict.robot1_id, conflict.robot2_id)

            # 让低优先级机器人在路径起点等待一个时间步
            if loser_id in paths and paths[loser_id]:
                paths[loser_id].insert(0, paths[loser_id][0])

        # 验证最终路径是否还有冲突
        final_conflicts = ConflictDetector.detect_conflicts(paths)
        if final_conflicts:
            # 仍有冲突，再次进行调整，让每个冲突的低优先级机器人多等待几步
            for conflict in final_conflicts:
                loser_id = max(conflict.robot1_id, conflict.robot2_id)
                if robot_priorities:
                    if robot_priorities.get(conflict.robot1_id, 999) < robot_priorities.get(conflict.robot2_id, 999):
                        loser_id = conflict.robot2_id
                    else:
                        loser_id = conflict.robot1_id

                # 让低优先级机器人多等待2步
                if loser_id in paths and paths[loser_id]:
                    for _ in range(2):
                        paths[loser_id].insert(0, paths[loser_id][0])

        return paths


# ---------------------------
# 死锁检测与管理
# ---------------------------
@dataclass
class Resource:
    """资源类，表示工厂中的物理资源，如机器、通道等"""
    id: str
    capacity: int = 1  # 资源容量(可同时容纳的机器人或任务数)
    occupied_by: Set[int] = field(default_factory=set)  # 当前占用资源的机器人ID
    waiting_robots: List[int] = field(default_factory=list)  # 等待使用资源的机器人队列


class DeadlockManager:
    """死锁检测与管理类"""

    def __init__(self):
        self.wait_graph = nx.DiGraph()  # 等待图，表示机器人之间的等待关系
        self.resources: Dict[str, Resource] = {}  # 资源集合
        self.robot_resources: Dict[int, Set[str]] = defaultdict(set)  # 每个机器人已占用的资源
        self.robot_waiting: Dict[int, str] = {}  # 每个机器人等待的资源
        self.deadlock_history: List[Tuple[str, Set[int]]] = []  # 死锁历史记录
        self.position_map: Dict[Position, int] = {}  # 位置到机器人ID的映射
        self.target_positions: Dict[int, Position] = {}  # 机器人ID到目标位置的映射
        self.deadlock_count = 0  # 死锁计数
        self.resolution_strategies = ["PRIORITY", "ABORT", "REPLAN"]  # 死锁解决策略
        self.current_strategy = "PRIORITY"  # 当前使用的解决策略

    def register_resource(self, resource_id: str, capacity: int = 1) -> None:
        """注册资源"""
        self.resources[resource_id] = Resource(resource_id, capacity)

    def register_position_resource(self, pos: Position) -> None:
        """注册位置资源"""
        resource_id = f"pos_{pos.x}_{pos.y}"
        self.register_resource(resource_id, 1)

    def update_robot_position(self, robot_id: int, position: Position) -> None:
        """更新机器人位置，占用位置资源"""
        # 释放之前的位置资源
        old_pos = None
        for pos, rid in list(self.position_map.items()):
            if rid == robot_id:
                old_pos = pos
                del self.position_map[pos]
                resource_id = f"pos_{pos.x}_{pos.y}"
                self.release_resource(robot_id, resource_id)

        # 占用新位置资源
        resource_id = f"pos_{position.x}_{position.y}"
        if resource_id not in self.resources:
            self.register_resource(resource_id, 1)

        success = self.request_resource(robot_id, resource_id)
        if success:
            self.position_map[position] = robot_id
        elif old_pos:  # 如果占用失败，回到原位置
            self.position_map[old_pos] = robot_id
            old_resource_id = f"pos_{old_pos.x}_{old_pos.y}"
            self.request_resource(robot_id, old_resource_id)

    def update_robot_target(self, robot_id: int, target: Position) -> None:
        """更新机器人的目标位置"""
        self.target_positions[robot_id] = target

    def request_resource(self, robot_id: int, resource_id: str) -> bool:
        """机器人请求资源，返回是否成功"""
        if resource_id not in self.resources:
            return False

        resource = self.resources[resource_id]

        # 如果机器人已经占用了该资源，直接返回成功
        if robot_id in resource.occupied_by:
            return True

        # 如果资源还有容量，分配给机器人
        if len(resource.occupied_by) < resource.capacity:
            resource.occupied_by.add(robot_id)
            self.robot_resources[robot_id].add(resource_id)

            # 从等待队列中移除(如果存在)
            if robot_id in resource.waiting_robots:
                resource.waiting_robots.remove(robot_id)
            if robot_id in self.robot_waiting and self.robot_waiting[robot_id] == resource_id:
                del self.robot_waiting[robot_id]

            return True
        else:
            # 资源已满，将机器人加入等待队列
            if robot_id not in resource.waiting_robots:
                resource.waiting_robots.append(robot_id)

            self.robot_waiting[robot_id] = resource_id
            self._update_wait_graph()
            return False

    def release_resource(self, robot_id: int, resource_id: str) -> bool:
        """机器人释放资源，返回是否成功"""
        if resource_id not in self.resources:
            return False

        resource = self.resources[resource_id]

        # 如果机器人占用了该资源，释放它
        if robot_id in resource.occupied_by:
            resource.occupied_by.remove(robot_id)
            if resource_id in self.robot_resources[robot_id]:
                self.robot_resources[robot_id].remove(resource_id)

            # 尝试将资源分配给等待队列中的下一个机器人
            if resource.waiting_robots:
                next_robot = resource.waiting_robots.pop(0)
                resource.occupied_by.add(next_robot)
                self.robot_resources[next_robot].add(resource_id)

                if next_robot in self.robot_waiting and self.robot_waiting[next_robot] == resource_id:
                    del self.robot_waiting[next_robot]

            self._update_wait_graph()
            return True
        return False

    def _update_wait_graph(self) -> None:
        """更新等待图"""
        self.wait_graph.clear()

        # 添加所有机器人节点
        robot_ids = set()
        for rid in self.robot_resources.keys():
            robot_ids.add(rid)
        for rid in self.robot_waiting.keys():
            robot_ids.add(rid)

        for rid in robot_ids:
            self.wait_graph.add_node(rid)

        # 添加等待边：如果机器人A等待的资源被机器人B占用，则添加A->B的边
        for waiting_robot, resource_id in self.robot_waiting.items():
            resource = self.resources.get(resource_id)
            if resource:
                for occupying_robot in resource.occupied_by:
                    self.wait_graph.add_edge(waiting_robot, occupying_robot)

    def detect_deadlock(self) -> Tuple[str, Set[int]]:
        """检测死锁，返回死锁类型和涉及的机器人集合"""
        try:
            cycles = list(nx.simple_cycles(self.wait_graph))
            if cycles:
                involved_robots = set()
                for cycle in cycles:
                    for robot_id in cycle:
                        involved_robots.add(robot_id)

                # 确定死锁类型
                if any(len(cycle) == 2 for cycle in cycles):
                    deadlock_type = "DIRECT"  # 直接死锁(两个机器人互相等待)
                else:
                    deadlock_type = "INDIRECT"  # 间接死锁(多个机器人形成等待环)

                return deadlock_type, involved_robots

            # 检查资源死锁(单个资源过载)
            for resource_id, resource in self.resources.items():
                if len(resource.waiting_robots) > 2 * resource.capacity:
                    return "RESOURCE", set(resource.waiting_robots)

            return "NONE", set()
        except Exception as e:
            print(f"死锁检测错误: {e}")
            return "ERROR", set()

    def resolve_deadlock(self, robot_priorities: Dict[int, int] = None) -> Set[int]:
        """解决死锁，返回被影响的机器人集合"""
        deadlock_type, involved_robots = self.detect_deadlock()
        if deadlock_type == "NONE" or not involved_robots:
            return set()

        self.deadlock_history.append((deadlock_type, involved_robots))
        self.deadlock_count += 1
        affected_robots = set()

        # 根据当前策略解决死锁
        if self.current_strategy == "PRIORITY":
            # 基于优先级解决环形等待
            if robot_priorities:
                # 找出优先级最低的机器人
                victim = max(involved_robots, key=lambda rid: robot_priorities.get(rid, 999))
            else:
                # 选择ID较大的机器人作为牺牲者
                victim = max(involved_robots)

            affected_robots.add(victim)

            # 放弃该机器人的资源请求
            if victim in self.robot_waiting:
                resource_id = self.robot_waiting[victim]
                resource = self.resources.get(resource_id)
                if resource and victim in resource.waiting_robots:
                    resource.waiting_robots.remove(victim)
                del self.robot_waiting[victim]

        elif self.current_strategy == "ABORT":
            # 终止所有涉及机器人的任务
            for rid in involved_robots:
                affected_robots.add(rid)

                # 清除所有等待
                if rid in self.robot_waiting:
                    resource_id = self.robot_waiting[rid]
                    resource = self.resources.get(resource_id)
                    if resource and rid in resource.waiting_robots:
                        resource.waiting_robots.remove(rid)
                    del self.robot_waiting[rid]

        elif self.current_strategy == "REPLAN":
            # 重规划策略：将在调用方的代码中实现路径重规划
            # 这里仅标记需要重规划的机器人
            affected_robots = involved_robots

        # 更新等待图
        self._update_wait_graph()

        # 如果死锁次数增加过快，尝试切换策略
        if self.deadlock_count % 10 == 0:
            self._switch_strategy()

        return affected_robots

    def _switch_strategy(self) -> None:
        """切换死锁解决策略"""
        current_index = self.resolution_strategies.index(self.current_strategy)
        next_index = (current_index + 1) % len(self.resolution_strategies)
        self.current_strategy = self.resolution_strategies[next_index]
        print(f"切换死锁解决策略为: {self.current_strategy}")

    def get_position_owner(self, pos: Position) -> Optional[int]:
        """获取位置的占用者"""
        return self.position_map.get(pos)

    def is_position_available(self, pos: Position) -> bool:
        """检查位置是否可用(没有被占用)"""
        return pos not in self.position_map

    def update_robot_paths(self, robot_paths: Dict[int, List[Position]]) -> None:
        """更新机器人路径，用于死锁检测"""
        # 清除旧的目标位置
        self.target_positions.clear()

        # 更新目标位置：使用每个路径的下一个位置作为目标
        for robot_id, path in robot_paths.items():
            if len(path) > 1:
                self.update_robot_target(robot_id, path[1])

    def get_deadlock_count(self) -> int:
        """获取死锁总数"""
        return self.deadlock_count

    def get_deadlock_history(self) -> List[Tuple[str, Set[int]]]:
        """获取死锁历史"""
        return self.deadlock_history


# ---------------------------
# 工厂区域定义
# ---------------------------
@dataclass
class Zone:
    """工厂区域类"""
    zone_type: ZoneType
    x1: int
    y1: int
    x2: int
    y2: int
    capacity: int  # 最大容量
    current_count: int = 0  # 当前数量

    def contains(self, pos: Position) -> bool:
        """检查位置是否在区域内"""
        return self.x1 <= pos.x <= self.x2 and self.y1 <= pos.y <= self.y2

    def get_center(self) -> Position:
        """获取区域中心点"""
        return Position((self.x1 + self.x2) // 2, (self.y1 + self.y2) // 2)

    def get_random_position(self) -> Position:
        """获取区域内随机位置"""
        x = random.randint(self.x1, self.x2)
        y = random.randint(self.y1, self.y2)
        return Position(x, y)

    def get_available_space(self) -> int:
        """获取区域内可用空间"""
        return max(0, self.capacity - self.current_count)

    def __str__(self) -> str:
        return f"{self.zone_type}({self.current_count}/{self.capacity})"


# ---------------------------
# 任务定义与管理
# ---------------------------
@dataclass
class Task:
    """任务类，表示一个运输任务"""
    id: int
    task_type: TaskType
    start_zone: ZoneType
    end_zone: ZoneType
    start_pos: Position
    end_pos: Position
    priority: int
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot: Optional[int] = None
    creation_time: int = 0
    completion_time: Optional[int] = None
    batch_size: int = 1  # 单次运输的批次大小
    expected_completion_time: Optional[int] = None  # 预期完成时间

    def __lt__(self, other):
        if not isinstance(other, Task):
            return NotImplemented
        # 首先按优先级比较，然后按创建时间比较
        return (self.priority, self.creation_time) < (other.priority, other.creation_time)

    def __str__(self) -> str:
        return f"Task({self.id}): {self.task_type}, 优先级={self.priority}, 状态={self.status}, " \
               f"从{self.start_zone}到{self.end_zone}"

    def get_progress(self) -> float:
        """获取任务进度"""
        if self.status == TaskStatus.COMPLETED:
            return 1.0
        elif self.status in [TaskStatus.PENDING, TaskStatus.ASSIGNED]:
            return 0.0
        elif self.status == TaskStatus.IN_PROGRESS:
            if self.expected_completion_time and self.creation_time:
                current_time = time.time()
                total_time = self.expected_completion_time - self.creation_time
                elapsed_time = current_time - self.creation_time
                if total_time > 0:
                    return min(1.0, max(0.0, elapsed_time / total_time))
        return 0.0


class TaskManager:
    """任务管理器"""

    def __init__(self):
        self.tasks: Dict[int, Task] = {}  # 所有任务
        self.pending_tasks = []  # 等待分配的任务(优先队列)
        self.next_task_id = 1
        self.tasks_by_type: Dict[TaskType, List[int]] = {
            task_type: [] for task_type in TaskType
        }
        self.tasks_by_status: Dict[TaskStatus, List[int]] = {
            status: [] for status in TaskStatus
        }
        self.task_history: List[Task] = []  # 任务历史记录

    def create_task(self, task_type: TaskType, start_zone: ZoneType, end_zone: ZoneType,
                    start_pos: Position, end_pos: Position, current_time: int,
                    batch_size: int = 1) -> int:
        """创建新任务"""
        priority = TaskType.get_priority(task_type)

        # 估计完成时间：基于起点到终点的距离
        distance = start_pos.manhattan_distance(end_pos)
        # 假设机器人每步移动1个单位，每个操作(装载/卸载)需要10个时间单位
        estimated_time = distance + 20  # 移动时间 + 装载卸载时间
        expected_completion_time = current_time + estimated_time

        task = Task(
            self.next_task_id, task_type, start_zone, end_zone,
            start_pos, end_pos, priority, TaskStatus.PENDING,
            creation_time=current_time, batch_size=batch_size,
            expected_completion_time=expected_completion_time
        )

        self.tasks[task.id] = task
        heapq.heappush(self.pending_tasks, task)
        self.tasks_by_type[task_type].append(task.id)
        self.tasks_by_status[TaskStatus.PENDING].append(task.id)
        self.next_task_id += 1
        return task.id

    def get_next_task(self) -> Optional[Task]:
        """获取下一个待处理任务"""
        while self.pending_tasks:
            task = heapq.heappop(self.pending_tasks)
            if task.status == TaskStatus.PENDING:
                return task
        return None

    def assign_task(self, task_id: int, robot_id: int) -> bool:
        """分配任务给机器人"""
        if task_id not in self.tasks:
            return False

        task = self.tasks[task_id]
        if task.status != TaskStatus.PENDING:
            return False

        # 更新任务状态
        self._update_task_status(task, TaskStatus.ASSIGNED)
        task.assigned_robot = robot_id
        return True

    def start_task(self, task_id: int) -> bool:
        """开始执行任务"""
        if task_id not in self.tasks:
            return False

        task = self.tasks[task_id]
        if task.status != TaskStatus.ASSIGNED:
            return False

        # 更新任务状态
        self._update_task_status(task, TaskStatus.IN_PROGRESS)
        return True

    def complete_task(self, task_id: int, current_time: int) -> bool:
        """完成任务"""
        if task_id not in self.tasks:
            return False

        task = self.tasks[task_id]
        if task.status != TaskStatus.IN_PROGRESS:
            return False

        # 更新任务状态
        self._update_task_status(task, TaskStatus.COMPLETED)
        task.completion_time = current_time

        # 添加到任务历史
        self.task_history.append(task)
        return True

    def fail_task(self, task_id: int) -> bool:
        """任务失败"""
        if task_id not in self.tasks:
            return False

        task = self.tasks[task_id]
        self._update_task_status(task, TaskStatus.FAILED)

        # 添加到任务历史
        self.task_history.append(task)
        return True

    def _update_task_status(self, task: Task, new_status: TaskStatus) -> None:
        """更新任务状态并维护索引"""
        if task.status == new_status:
            return

        # 从旧状态列表中移除
        if task.id in self.tasks_by_status[task.status]:
            self.tasks_by_status[task.status].remove(task.id)

        # 添加到新状态列表
        task.status = new_status
        self.tasks_by_status[new_status].append(task.id)

    def get_tasks_by_status(self, status: TaskStatus) -> List[Task]:
        """获取指定状态的所有任务"""
        task_ids = self.tasks_by_status.get(status, [])
        return [self.tasks[tid] for tid in task_ids if tid in self.tasks]

    def get_tasks_by_type(self, task_type: TaskType) -> List[Task]:
        """获取指定类型的所有任务"""
        task_ids = self.tasks_by_type.get(task_type, [])
        return [self.tasks[tid] for tid in task_ids if tid in self.tasks]

    def get_task_count_by_type(self, task_type: TaskType) -> int:
        """获取指定类型的任务数量"""
        return len(self.tasks_by_type.get(task_type, []))

    def get_completed_task_count(self) -> int:
        """获取已完成任务数量"""
        return len(self.tasks_by_status.get(TaskStatus.COMPLETED, []))

    def get_completion_rate(self) -> float:
        """获取任务完成率"""
        total_tasks = sum(len(tasks) for tasks in self.tasks_by_status.values())
        if total_tasks == 0:
            return 0.0
        completed_tasks = len(self.tasks_by_status.get(TaskStatus.COMPLETED, []))
        return completed_tasks / total_tasks

    def get_average_completion_time(self) -> float:
        """获取平均任务完成时间"""
        completed_tasks = self.get_tasks_by_status(TaskStatus.COMPLETED)
        if not completed_tasks:
            return 0.0

        total_time = sum(
            task.completion_time - task.creation_time
            for task in completed_tasks
            if task.completion_time is not None
        )
        return total_time / len(completed_tasks)

    def get_task_success_rate(self) -> float:
        """获取任务成功率"""
        completed = len(self.tasks_by_status.get(TaskStatus.COMPLETED, []))
        failed = len(self.tasks_by_status.get(TaskStatus.FAILED, []))
        total = completed + failed

        if total == 0:
            return 1.0  # 如果没有已完成或失败的任务，返回100%

        return completed / total


# ---------------------------
# 生产设备定义
# ---------------------------
class Machine:
    """生产设备类"""

    def __init__(self, machine_id: int, machine_type: MachineType, position: Position, processing_time: float):
        self.id = machine_id
        self.type = machine_type
        self.position = position
        self.processing_time = processing_time  # 处理时间(秒)
        self.status = MachineStatus.IDLE
        self.remaining_time = 0  # 剩余处理时间
        self.processed_batches = 0  # 已处理批次
        self.current_batch_size = 0  # 当前批次大小(已装填的数量)
        self.max_batch_size = 1  # 一次处理的最大批次大小
        self.total_processing_time = 0  # 总加工时间
        self.total_idle_time = 0  # 总空闲时间

        # 根据机器类型设置批次大小
        if machine_type == MachineType.FIRST_DRAWING:
            self.max_batch_size = 12  # 一并机一次处理12个绿桶
        elif machine_type == MachineType.SECOND_DRAWING:
            self.max_batch_size = 12  # 二并机一次处理12个黄桶

    def start_processing(self) -> bool:
        """开始处理批次"""
        if self.status != MachineStatus.IDLE or self.current_batch_size == 0:
            return False

        self.status = MachineStatus.PROCESSING
        self.remaining_time = self.processing_time
        return True

    def load_material(self, count: int = 1) -> int:
        """装载物料，返回实际装载的数量"""
        if self.status != MachineStatus.IDLE:
            return 0

        available_space = self.max_batch_size - self.current_batch_size
        loaded = min(count, available_space)
        self.current_batch_size += loaded

        # 如果批次已满，自动开始处理
        if self.current_batch_size >= self.max_batch_size:
            self.start_processing()

        return loaded

    def update(self, is_idle: bool = False) -> bool:
        """更新机器状态，返回是否有批次处理完成"""
        if is_idle:
            self.total_idle_time += 1

        if self.status == MachineStatus.PROCESSING:
            self.total_processing_time += 1
            self.remaining_time -= 1
            if self.remaining_time <= 0:
                self.status = MachineStatus.IDLE
                self.processed_batches += 1
                processed = self.current_batch_size
                self.current_batch_size = 0
                return True  # 批次已处理完成
        return False  # 继续处理或空闲

    def get_utilization(self, total_time: int) -> float:
        """计算机器利用率"""
        if total_time == 0:
            return 0
        return min(1.0, self.total_processing_time / total_time)

    def __str__(self) -> str:
        return f"{self.type}({self.id}): {self.status}, " \
               f"已处理批次: {self.processed_batches}, 当前批次: {self.current_batch_size}/{self.max_batch_size}"


# ---------------------------
# 机器人定义
# ---------------------------
class Robot:
    """运输机器人类"""

    def __init__(self, robot_id: int, initial_position: Position):
        self.id = robot_id
        self.position = initial_position
        self.original_position = initial_position  # 初始位置(空闲时返回)
        self.status = "idle"  # idle, moving, loading, unloading
        self.task: Optional[Task] = None  # 当前任务
        self.path: List[Position] = []  # 当前路径
        self.carrying_material = False  # 是否携带物料
        self.material_type: Optional[str] = None  # 物料类型(空桶/绿桶/黄桶/红桶)
        self.material_count: int = 0  # 携带的物料数量
        self.total_distance = 0  # 总移动距离
        self.total_tasks_completed = 0  # 完成任务总数
        self.idle_time = 0  # 空闲时间
        self.working_time = 0  # 工作时间
        self.task_history: List[int] = []  # 历史任务ID列表
        self.waiting_time = 0  # 等待时间(由于死锁等)
        self.task_type_counts = {task_type: 0 for task_type in TaskType}  # 各类型任务完成数量

    def assign_task(self, task: Task) -> bool:
        """分配任务给机器人"""
        if self.status != "idle" or task.status != TaskStatus.PENDING:
            return False

        self.task = task
        self.status = "assigned"
        return True

    def set_path(self, path: List[Position]) -> None:
        """设置当前路径"""
        self.path = path if path else []
        if self.path:
            self.status = "moving"

    def move(self) -> bool:
        """沿着路径移动一步，返回是否移动成功"""
        if self.status != "moving" or not self.path or len(self.path) < 2:
            return False

        # 当前位置和下一个位置
        current = self.path[0]
        next_pos = self.path[1]

        # 计算移动距离
        distance = current.distance_to(next_pos)
        self.total_distance += distance

        # 更新位置和路径
        self.position = next_pos
        self.path = self.path[1:]

        # 如果到达目的地
        if len(self.path) == 1:  # 只剩下当前位置
            if not self.carrying_material:
                self.status = "loading"  # 到达起点，准备装载
            else:
                self.status = "unloading"  # 到达终点，准备卸载

        return True

    def load_material(self, material_type: str, count: int = 1) -> bool:
        """装载物料"""
        if self.status != "loading" or self.carrying_material:
            return False

        self.carrying_material = True
        self.material_type = material_type
        self.material_count = count
        self.status = "moving"  # 装载后继续移动
        return True

    def unload_material(self) -> Tuple[Optional[str], int]:
        """卸载物料，返回物料类型和数量"""
        if self.status != "unloading" or not self.carrying_material:
            return None, 0

        material = self.material_type
        count = self.material_count
        self.carrying_material = False
        self.material_type = None
        self.material_count = 0
        self.status = "idle"  # 卸载后变为空闲

        if self.task:
            self.total_tasks_completed += 1
            self.task_type_counts[self.task.task_type] = self.task_type_counts.get(self.task.task_type, 0) + 1
            self.task_history.append(self.task.id)
            self.task = None

        return material, count

    def wait(self) -> None:
        """让机器人等待"""
        self.waiting_time += 1

    def update(self, is_working: bool = False) -> None:
        """更新机器人状态"""
        if is_working:
            self.working_time += 1
        else:
            self.idle_time += 1

    def get_utilization(self) -> float:
        """计算机器人利用率"""
        total_time = self.idle_time + self.working_time
        if total_time == 0:
            return 0
        return self.working_time / total_time

    def get_waiting_ratio(self) -> float:
        """计算等待时间占比"""
        total_time = self.idle_time + self.working_time
        if total_time == 0:
            return 0
        return self.waiting_time / total_time

    def get_task_type_distribution(self) -> Dict[TaskType, float]:
        """获取任务类型分布比例"""
        total = sum(self.task_type_counts.values())
        if total == 0:
            return {task_type: 0.0 for task_type in TaskType}

        return {task_type: count / total for task_type, count in self.task_type_counts.items()}

    def get_efficiency(self) -> float:
        """计算机器人效率(任务/工作时间)"""
        if self.working_time == 0:
            return 0
        return self.total_tasks_completed / (self.working_time / 3600)  # 每小时完成任务数

    def __str__(self) -> str:
        status_str = f"机器人{self.id}: {self.status}, 位置={self.position}"
        if self.carrying_material:
            status_str += f", 携带={self.material_type}({self.material_count})"
        if self.task:
            status_str += f", 任务={self.task.id}"
        return status_str


# ---------------------------
# 性能监控
# ---------------------------
class PerformanceMonitor:
    """性能监控类"""

    def __init__(self):
        self.start_time = 0
        self.current_time = 0
        self.total_tasks_completed = 0
        self.machine_utilization: Dict[MachineType, List[float]] = {
            mt: [] for mt in MachineType
        }
        self.robot_stats: Dict[int, Dict[str, Any]] = {}
        self.material_counts = {
            "empty": 540,  # 初始540个空桶
            "green": 0,  # 绿桶(梳棉产出)
            "yellow": 0,  # 黄桶(一并产出)
            "red": 0  # 红桶(二并产出)
        }
        self.material_history = {
            "time": [],
            "empty": [],
            "green": [],
            "yellow": [],
            "red": []
        }
        self.deadlock_count = 0
        self.deadlock_history = []
        self.task_throughput = []  # 每个时间点的任务吞吐量
        self.theoretical_time = 0  # 理论生产时间
        self.actual_time = 0  # 实际生产时间

        # 记录每种产品的首次生产时间
        self.first_production_time = {
            "green": None,
            "yellow": None,
            "red": None
        }

        # 用于计算吞吐量的队列
        self.throughput_window = deque(maxlen=100)  # 最近100个时间单位的完成任务数

    def start(self, time: int) -> None:
        """开始监控"""
        self.start_time = time
        self.current_time = time
        self.record_material_counts()

        # 计算理论生产时间
        # 假设:
        # 1. 梳棉机处理时间: 4225.94秒
        # 2. 并条机处理时间: 2594.75秒
        # 3. 从空桶到红桶的转换比例: 36:1 (每36个空桶产生1个红桶)
        # 4. 目标是15个红桶

        # 计算纯加工时间
        carding_time = 4225.94  # 梳棉时间
        drawing_time = 2594.75  # 并条时间(一并和二并相同)

        # 计算完整的处理时间线:
        # 1. 540个空桶需要分成15个红桶组(每个红桶组需要36个空桶)
        # 2. 每个红桶组需要经过梳棉(多个批次)→一并→二并
        # 每个红桶组的完整处理时间:
        single_pipeline_time = carding_time + drawing_time + drawing_time

        # 但由于有多台设备并行工作，实际上是流水线生产
        # 假设足够的并行度，理论时间由最长的环节决定
        theoretical_cycle_time = max(carding_time, drawing_time)

        # 15个红桶的理论时间 = 第一个红桶的完整处理时间 + 14个红桶的循环时间
        self.theoretical_time = single_pipeline_time + (15 - 1) * theoretical_cycle_time

    def update(self, time: int, machines: Dict[MachineType, List[Machine]],
               robots: List[Robot], deadlock_info: Optional[Tuple[str, Set[int]]] = None) -> None:
        """更新监控数据"""
        self.current_time = time

        # 更新吞吐量窗口
        completed_in_interval = self.total_tasks_completed - sum(self.throughput_window)
        self.throughput_window.append(completed_in_interval)
        self.task_throughput.append(sum(self.throughput_window) / len(self.throughput_window))

        # 更新机器利用率
        for machine_type, machine_list in machines.items():
            if machine_list:
                avg_utilization = sum(m.get_utilization(time) for m in machine_list) / len(machine_list)
                self.machine_utilization[machine_type].append(avg_utilization)

        # 更新机器人统计数据
        for robot in robots:
            if robot.id not in self.robot_stats:
                self.robot_stats[robot.id] = {
                    "distance": [],
                    "tasks_completed": [],
                    "utilization": [],
                    "waiting_ratio": [],
                    "efficiency": []
                }

            stats = self.robot_stats[robot.id]
            stats["distance"].append(robot.total_distance)
            stats["tasks_completed"].append(robot.total_tasks_completed)
            stats["utilization"].append(robot.get_utilization())
            stats["waiting_ratio"].append(robot.get_waiting_ratio())
            stats["efficiency"].append(robot.get_efficiency())

        # 记录死锁信息
        if deadlock_info and deadlock_info[0] != "NONE":
            self.deadlock_count += 1
            self.deadlock_history.append((time, deadlock_info))

    def update_material_counts(self, empty: int, green: int, yellow: int, red: int) -> None:
        """更新物料数量"""
        self.material_counts["empty"] = empty
        self.material_counts["green"] = green
        self.material_counts["yellow"] = yellow
        self.material_counts["red"] = red

        # 记录各类型物料首次生产时间
        if green > 0 and self.first_production_time["green"] is None:
            self.first_production_time["green"] = self.current_time
        if yellow > 0 and self.first_production_time["yellow"] is None:
            self.first_production_time["yellow"] = self.current_time
        if red > 0 and self.first_production_time["red"] is None:
            self.first_production_time["red"] = self.current_time

        self.record_material_counts()

    def record_material_counts(self) -> None:
        """记录当前物料数量"""
        self.material_history["time"].append(self.current_time)
        for material in ["empty", "green", "yellow", "red"]:
            self.material_history[material].append(self.material_counts[material])

    def get_material_count(self, material_type: str) -> int:
        """获取当前物料数量"""
        return self.material_counts.get(material_type, 0)

    def get_simulation_time(self) -> int:
        """获取模拟运行时间"""
        return self.current_time - self.start_time

    def get_average_machine_utilization(self, machine_type: MachineType) -> float:
        """获取指定类型机器的平均利用率"""
        utils = self.machine_utilization.get(machine_type, [])
        return sum(utils) / len(utils) if utils else 0

    def get_robot_distance(self, robot_id: int) -> float:
        """获取机器人的总移动距离"""
        stats = self.robot_stats.get(robot_id, {})
        distances = stats.get("distance", [])
        return distances[-1] if distances else 0

    def get_robot_completed_tasks(self, robot_id: int) -> int:
        """获取机器人完成的任务数"""
        stats = self.robot_stats.get(robot_id, {})
        tasks = stats.get("tasks_completed", [])
        return tasks[-1] if tasks else 0

    def get_robot_utilization(self, robot_id: int) -> float:
        """获取机器人的利用率"""
        stats = self.robot_stats.get(robot_id, {})
        utils = stats.get("utilization", [])
        return utils[-1] if utils else 0

    def get_total_deadlocks(self) -> int:
        """获取总死锁次数"""
        return self.deadlock_count

    def get_red_bucket_production_rate(self) -> float:
        """获取红桶生产率(每小时)"""
        sim_time_hours = self.get_simulation_time() / 3600  # 转换为小时
        if sim_time_hours > 0:
            return self.material_counts["red"] / sim_time_hours
        return 0

    def get_material_history(self) -> Dict[str, List]:
        """获取物料历史数据"""
        return self.material_history

    def get_efficiency_ratio(self) -> float:
        """获取生产效率比: 理论时间/实际时间"""
        if self.current_time == 0 or self.theoretical_time == 0:
            return 0.0

        self.actual_time = self.get_simulation_time()
        if self.material_counts["red"] >= 15:
            # 如果达到目标，使用实际时间
            return self.theoretical_time / self.actual_time

        # 如果未达到目标，基于当前进度预测
        completion_percentage = self.material_counts["red"] / 15
        if completion_percentage > 0:
            estimated_total_time = self.actual_time / completion_percentage
            return self.theoretical_time / estimated_total_time

        return 0.0

    def get_production_bottleneck(self) -> Tuple[str, float]:
        """分析生产瓶颈"""
        utilization_data = {}
        for machine_type in MachineType:
            util = self.get_average_machine_utilization(machine_type)
            utilization_data[str(machine_type)] = util

        # 添加机器人利用率
        avg_robot_util = sum(
            self.get_robot_utilization(rid)
            for rid in self.robot_stats.keys()
        ) / len(self.robot_stats) if self.robot_stats else 0

        utilization_data["Robots"] = avg_robot_util

        # 找出利用率最高的环节
        bottleneck = max(utilization_data.items(), key=lambda x: x[1])
        return bottleneck

    def get_production_lead_times(self) -> Dict[str, float]:
        """获取各类产品的生产提前期"""
        lead_times = {}

        # 计算各类型产品从开始到首次生产的时间
        for material, first_time in self.first_production_time.items():
            if first_time is not None:
                lead_times[material] = first_time - self.start_time
            else:
                lead_times[material] = None

        return lead_times

    def generate_report(self) -> str:
        """生成性能报告"""
        sim_time = self.get_simulation_time()
        efficiency_ratio = self.get_efficiency_ratio()
        bottleneck, bottleneck_util = self.get_production_bottleneck()
        lead_times = self.get_production_lead_times()

        report = [
            "===== 工厂模拟性能报告 =====",
            f"总模拟时间: {sim_time}秒 ({sim_time / 3600:.2f}小时)",
            f"物料数量: 空桶={self.material_counts['empty']}, 绿桶={self.material_counts['green']}, "
            f"黄桶={self.material_counts['yellow']}, 红桶={self.material_counts['red']}",
            f"红桶生产率: {self.get_red_bucket_production_rate():.2f}个/小时",
            f"流水线效率: {efficiency_ratio:.2%} (理论时间/实际时间)",
            f"生产瓶颈: {bottleneck}, 利用率: {bottleneck_util:.2%}",
            f"总死锁次数: {self.deadlock_count}",
            "\n生产提前期:",
        ]

        for material, lead_time in lead_times.items():
            if lead_time is not None:
                report.append(f"  {material}: {lead_time}秒")
            else:
                report.append(f"  {material}: 未生产")

        report.append("\n机器利用率:")
        for machine_type in MachineType:
            util = self.get_average_machine_utilization(machine_type)
            report.append(f"  {machine_type}: {util:.2%}")

        report.append("\n机器人统计:")
        for robot_id in sorted(self.robot_stats.keys()):
            report.append(f"  机器人{robot_id}:")
            report.append(f"    总移动距离: {self.get_robot_distance(robot_id):.2f}")
            report.append(f"    完成任务数: {self.get_robot_completed_tasks(robot_id)}")
            report.append(f"    利用率: {self.get_robot_utilization(robot_id):.2%}")

        return "\n".join(report)


# ---------------------------
# 可视化工具
# ---------------------------
class Visualizer:
    """工厂模拟可视化工具"""

    def __init__(self, factory):
        self.factory = factory
        self.fig = None
        self.ax = None
        self.robot_markers = {}
        self.machine_markers = {}
        self.zone_patches = {}
        self.material_plot = None
        self.animation = None
        self.last_update_time = time.time()
        self.update_interval = 0.5  # 秒

        # 区域颜色定义
        self.zone_colors = {
            ZoneType.EMPTY_BUCKET: 'lightgray',
            ZoneType.CARDING_LOAD: 'lightblue',
            ZoneType.CARDING_UNLOAD: 'lightgreen',
            ZoneType.FIRST_DRAWING_LOAD: 'bisque',
            ZoneType.FIRST_DRAWING_UNLOAD: 'khaki',
            ZoneType.SECOND_DRAWING_LOAD: 'lightcoral',
            ZoneType.SECOND_DRAWING_UNLOAD: 'lightpink',
            ZoneType.ROVING: 'plum',
            ZoneType.ROBOT_IDLE: 'lightcyan'
        }

        # 机器人与机器标记样式
        self.robot_marker = 'o'
        self.machine_markers_style = {
            MachineType.CARDING: 's',  # 正方形
            MachineType.FIRST_DRAWING: '^',  # 三角形
            MachineType.SECOND_DRAWING: 'D',  # 菱形
            MachineType.ROVING: 'X'  # X形
        }

    def setup_plot(self):
        """设置绘图环境"""
        self.fig, self.ax = plt.subplots(1, 1, figsize=(15, 15))
        self.ax.set_xlim(0, self.factory.width)
        self.ax.set_ylim(0, self.factory.height)
        self.ax.set_aspect('equal')
        self.ax.set_title('工厂自动化模拟')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        # 绘制区域
        for zone_type, zone in self.factory.zones.items():
            rect = patches.Rectangle(
                (zone.x1, zone.y1), zone.x2 - zone.x1, zone.y2 - zone.y1,
                linewidth=1, edgecolor='black', facecolor=self.zone_colors.get(zone_type, 'white'),
                alpha=0.5, label=str(zone_type)
            )
            self.zone_patches[zone_type] = rect
            self.ax.add_patch(rect)

            # 添加区域标签
            self.ax.text(
                (zone.x1 + zone.x2) / 2, (zone.y1 + zone.y2) / 2,
                str(zone_type),
                ha='center', va='center'
            )

        # 绘制机器
        for machine_type, machines in self.factory.machines.items():
            for machine in machines:
                marker = self.ax.scatter(
                    machine.position.x, machine.position.y,
                    marker=self.machine_markers_style.get(machine_type, 'o'),
                    s=100, color='black', label=f"{machine_type}_{machine.id}"
                )
                self.machine_markers[machine.id] = marker

        # 绘制机器人
        for robot in self.factory.robots:
            marker = self.ax.scatter(
                robot.position.x, robot.position.y,
                marker=self.robot_marker, s=80,
                color=f"C{robot.id % 10}", label=f"Robot_{robot.id}"
            )
            self.robot_markers[robot.id] = marker

        # 添加图例
        self.ax.legend(loc='upper left', bbox_to_anchor=(1, 1))

        plt.tight_layout()

    def setup_material_plot(self):
        """设置物料历史图"""
        self.material_fig, self.material_ax = plt.subplots(figsize=(10, 6))
        self.material_ax.set_title('物料数量历史')
        self.material_ax.set_xlabel('时间(秒)')
        self.material_ax.set_ylabel('数量')
        self.material_lines = {}

        material_history = self.factory.performance_monitor.get_material_history()
        for material, color in [('empty', 'gray'), ('green', 'green'),
                                ('yellow', 'gold'), ('red', 'red')]:
            if material in material_history and len(material_history[material]) > 0:
                line, = self.material_ax.plot(
                    material_history['time'], material_history[material],
                    label=material, color=color
                )
                self.material_lines[material] = line

        self.material_ax.legend()
        plt.tight_layout()

    def update_plot(self):
        """更新绘图"""
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return

        self.last_update_time = current_time

        # 更新机器人位置
        for robot in self.factory.robots:
            if robot.id in self.robot_markers:
                self.robot_markers[robot.id].set_offsets([robot.position.x, robot.position.y])

        # 更新区域信息
        for zone_type, zone in self.factory.zones.items():
            if zone_type in self.zone_patches:
                self.ax.text(
                    (zone.x1 + zone.x2) / 2, (zone.y1 + zone.y2) / 2 - 10,
                    f"{zone.current_count}/{zone.capacity}",
                    ha='center', va='center', color='blue'
                )

        # 更新标题信息
        self.ax.set_title(f'工厂自动化模拟 - 时间: {self.factory.global_time}秒, '
                          f'红桶: {self.factory.performance_monitor.get_material_count("red")}/15')

        self.fig.canvas.draw()

    def update_material_plot(self):
        """更新物料历史图"""
        material_history = self.factory.performance_monitor.get_material_history()
        for material in ['empty', 'green', 'yellow', 'red']:
            if material in self.material_lines and material in material_history:
                self.material_lines[material].set_data(
                    material_history['time'], material_history[material]
                )

        self.material_ax.relim()
        self.material_ax.autoscale_view()
        self.material_fig.canvas.draw()

    def animate_factory(self):
        """创建工厂动画"""

        def update(frame):
            # 更新机器人位置
            for robot in self.factory.robots:
                if robot.id in self.robot_markers:
                    self.robot_markers[robot.id].set_offsets([robot.position.x, robot.position.y])

            # 更新标题
            self.ax.set_title(f'工厂自动化模拟 - 时间: {self.factory.global_time}秒, '
                              f'红桶: {self.factory.performance_monitor.get_material_count("red")}/15')

            return list(self.robot_markers.values())

        self.animation = FuncAnimation(
            self.fig, update, frames=range(1000),
            interval=200, blit=True
        )

    def plot_robot_stats(self):
        """绘制机器人统计图"""
        # 创建机器人统计图
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        # 绘制机器人移动距离
        robot_ids = []
        distances = []
        for robot_id, stats in self.factory.performance_monitor.robot_stats.items():
            robot_ids.append(robot_id)
            distances.append(self.factory.performance_monitor.get_robot_distance(robot_id))

        ax1.bar(robot_ids, distances)
        ax1.set_title('机器人移动距离')
        ax1.set_xlabel('机器人ID')
        ax1.set_ylabel('距离')
        ax1.grid(True, linestyle='--', alpha=0.7)

        # 绘制机器人任务完成数
        tasks = []
        for robot_id in robot_ids:
            tasks.append(self.factory.performance_monitor.get_robot_completed_tasks(robot_id))

        ax2.bar(robot_ids, tasks)
        ax2.set_title('机器人完成任务数')
        ax2.set_xlabel('机器人ID')
        ax2.set_ylabel('任务数')
        ax2.grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout()
        return fig

    def plot_machine_utilization(self):
        """绘制机器利用率"""
        fig, ax = plt.subplots(figsize=(10, 6))

        machine_types = []
        utilizations = []
        for machine_type in MachineType:
            machine_types.append(str(machine_type))
            utilizations.append(
                self.factory.performance_monitor.get_average_machine_utilization(machine_type))

        ax.bar(machine_types, utilizations)
        ax.set_title('机器利用率')
        ax.set_xlabel('机器类型')
        ax.set_ylabel('平均利用率')
        ax.set_ylim(0, 1)
        ax.grid(True, linestyle='--', alpha=0.7)

        for i, v in enumerate(utilizations):
            ax.text(i, v + 0.02, f'{v:.2%}', ha='center')

        plt.tight_layout()
        return fig

    def plot_task_distribution(self):
        """绘制任务分布"""
        # 获取任务类型统计
        task_type_counts = {}
        for task_type in TaskType:
            count = self.factory.task_manager.get_task_count_by_type(task_type)
            task_type_counts[str(task_type)] = count

        # 创建饼图
        fig, ax = plt.subplots(figsize=(10, 8))
        labels = list(task_type_counts.keys())
        sizes = list(task_type_counts.values())

        if sum(sizes) > 0:  # 确保有任务
            ax.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=90)
            ax.axis('equal')  # 保持饼图为圆形
            ax.set_title('任务类型分布')
        else:
            ax.text(0.5, 0.5, '无任务数据', ha='center', va='center')
            ax.axis('off')

        plt.tight_layout()
        return fig

    def plot_material_flow(self):
        """绘制物料流动图"""
        material_history = self.factory.performance_monitor.get_material_history()

        fig, ax = plt.subplots(figsize=(12, 6))

        for material, color, label in [
            ('empty', 'gray', '空桶'),
            ('green', 'green', '绿桶'),
            ('yellow', 'gold', '黄桶'),
            ('red', 'red', '红桶')
        ]:
            if material in material_history and len(material_history[material]) > 1:
                ax.plot(
                    material_history['time'],
                    material_history[material],
                    label=label,
                    color=color,
                    linewidth=2
                )

        ax.set_title('物料流动趋势')
        ax.set_xlabel('时间(秒)')
        ax.set_ylabel('数量')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()

        plt.tight_layout()
        return fig

    def plot_efficiency_metrics(self):
        """绘制效率指标图"""
        # 计算理论与实际时间
        theoretical_time = self.factory.performance_monitor.theoretical_time
        actual_time = self.factory.performance_monitor.get_simulation_time()
        efficiency_ratio = self.factory.performance_monitor.get_efficiency_ratio()

        # 创建柱状图
        fig, ax = plt.subplots(figsize=(10, 6))

        times = ['理论时间', '实际时间']
        values = [theoretical_time, actual_time]

        bars = ax.bar(times, values)
        ax.set_title('生产时间比较')
        ax.set_ylabel('时间(秒)')
        ax.grid(True, axis='y', linestyle='--', alpha=0.7)

        # 添加效率比例标签
        ax.text(0.5, 0.9, f'效率比: {efficiency_ratio:.2%}',
                transform=ax.transAxes, ha='center',
                bbox=dict(facecolor='white', alpha=0.8))

        # 为每个柱子添加标签
        for bar in bars:
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width() / 2., height,
                    f'{height:.1f}',
                    ha='center', va='bottom')

        plt.tight_layout()
        return fig

    def plot_deadlock_analysis(self):
        """绘制死锁分析图"""
        deadlock_history = self.factory.deadlock_manager.get_deadlock_history()
        deadlock_count = self.factory.deadlock_manager.get_deadlock_count()

        if not deadlock_history:
            fig, ax = plt.subplots(figsize=(8, 6))
            ax.text(0.5, 0.5, '无死锁数据', ha='center', va='center')
            ax.axis('off')
            return fig

        # 统计不同类型的死锁数量
        deadlock_types = {}
        for time_point, (d_type, robots) in deadlock_history:
            if d_type not in deadlock_types:
                deadlock_types[d_type] = 0
            deadlock_types[d_type] += 1

        # 创建饼图
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

        # 死锁类型分布
        labels = list(deadlock_types.keys())
        sizes = list(deadlock_types.values())

        ax1.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=90)
        ax1.axis('equal')
        ax1.set_title('死锁类型分布')

        # 死锁发生时间
        times = [time_point for time_point, _ in deadlock_history]
        ax2.hist(times, bins=min(20, len(times)), color='skyblue', edgecolor='black')
        ax2.set_title(f'死锁发生时间点分布 (总计: {deadlock_count})')
        ax2.set_xlabel('时间(秒)')
        ax2.set_ylabel('死锁次数')
        ax2.grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout()
        return fig

    def save_plots(self, prefix='factory_simulation'):
        """保存所有图表"""
        # 确保输出目录存在
        output_dir = f"output_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(output_dir, exist_ok=True)

        # 保存工厂布局图
        if self.fig:
            self.fig.savefig(f'{output_dir}/{prefix}_layout.png')

        # 保存物料历史图
        if hasattr(self, 'material_fig'):
            self.material_fig.savefig(f'{output_dir}/{prefix}_materials.png')

        # 生成并保存机器人统计图
        robot_stats_fig = self.plot_robot_stats()
        robot_stats_fig.savefig(f'{output_dir}/{prefix}_robot_stats.png')

        # 生成并保存机器利用率图
        machine_util_fig = self.plot_machine_utilization()
        machine_util_fig.savefig(f'{output_dir}/{prefix}_machine_utilization.png')

        # 生成并保存任务分布图
        task_dist_fig = self.plot_task_distribution()
        task_dist_fig.savefig(f'{output_dir}/{prefix}_task_distribution.png')

        # 生成并保存物料流动图
        material_flow_fig = self.plot_material_flow()
        material_flow_fig.savefig(f'{output_dir}/{prefix}_material_flow.png')

        # 生成并保存效率指标图
        efficiency_fig = self.plot_efficiency_metrics()
        efficiency_fig.savefig(f'{output_dir}/{prefix}_efficiency.png')

        # 生成并保存死锁分析图
        deadlock_fig = self.plot_deadlock_analysis()
        deadlock_fig.savefig(f'{output_dir}/{prefix}_deadlock_analysis.png')

        # 保存性能报告文本
        report = self.factory.performance_monitor.generate_report()
        with open(f'{output_dir}/{prefix}_report.txt', 'w', encoding='utf-8') as f:
            f.write(report)

        # 保存模拟数据
        sim_data = {
            'material_history': self.factory.performance_monitor.material_history,
            'machine_utilization': self.factory.performance_monitor.machine_utilization,
            'robot_stats': self.factory.performance_monitor.robot_stats,
            'deadlock_history': self.factory.performance_monitor.deadlock_history,
            'simulation_time': self.factory.performance_monitor.get_simulation_time(),
            'theoretical_time': self.factory.performance_monitor.theoretical_time
        }

        with open(f'{output_dir}/{prefix}_data.pkl', 'wb') as f:
            pickle.dump(sim_data, f)

        print(f"所有图表和数据已保存到目录: {output_dir}")

    def show(self):
        """显示所有图表"""
        plt.show()


# ---------------------------
# 匈牙利算法实现任务分配优化
# ---------------------------
class TaskAssignmentOptimizer:
    """使用匈牙利算法优化任务分配"""

    def __init__(self, robots, task_manager):
        self.robots = robots
        self.task_manager = task_manager

    def compute_cost_matrix(self, tasks, robots):
        """计算任务分配的成本矩阵"""
        # 根据机器人到任务起点的距离和任务优先级计算成本
        cost_matrix = np.zeros((len(robots), len(tasks)))

        for i, robot in enumerate(robots):
            for j, task in enumerate(tasks):
                # 距离成本: 机器人到任务起点的距离
                distance = robot.position.manhattan_distance(task.start_pos)

                # 优先级成本: 高优先级任务成本低
                priority_factor = task.priority * 50  # 优先级影响系数

                # 总成本 = 距离 + 优先级因子
                cost_matrix[i, j] = distance + priority_factor

        return cost_matrix

    def optimize_assignment(self):
        """使用匈牙利算法优化任务分配"""
        # 获取空闲机器人
        idle_robots = [robot for robot in self.robots if robot.status == "idle"]
        if not idle_robots:
            return []

        # 获取待分配任务
        pending_tasks = self.task_manager.get_tasks_by_status(TaskStatus.PENDING)
        if not pending_tasks:
            return []

        # 如果任务数大于机器人数，选择优先级高的任务
        if len(pending_tasks) > len(idle_robots):
            pending_tasks.sort()  # 按优先级排序
            pending_tasks = pending_tasks[:len(idle_robots)]

        # 计算成本矩阵
        cost_matrix = self.compute_cost_matrix(pending_tasks, idle_robots)

        # 使用匈牙利算法求解最优分配
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # 构造分配结果
        assignments = []
        for i, j in zip(row_ind, col_ind):
            robot = idle_robots[i]
            task = pending_tasks[j]
            assignments.append((robot, task))

        return assignments


# ---------------------------
# 工厂模拟系统主类
# ---------------------------
class FactorySimulation:
    """工厂自动化生产模拟系统"""

    def __init__(self, width=1000, height=1000):
        # 工厂尺寸
        self.width = width
        self.height = height

        # 初始化代价地图
        self.cost_map = CostMap(width, height)

        # 初始化区域
        self.zones: Dict[ZoneType, Zone] = {}
        self.init_zones()

        # 初始化死锁管理器
        self.deadlock_manager = DeadlockManager()

        # 初始化机器
        self.machines: Dict[MachineType, List[Machine]] = {
            MachineType.CARDING: [],
            MachineType.FIRST_DRAWING: [],
            MachineType.SECOND_DRAWING: [],
            MachineType.ROVING: []
        }
        self.init_machines()

        # 初始化路径规划器
        self.path_planner = PathPlanner(self.cost_map)
        self.cbs_planner = CBSPlanner(self.path_planner)

        # 初始化任务管理器
        self.task_manager = TaskManager()

        # 初始化机器人
        self.robots: List[Robot] = []
        self.init_robots()

        # 初始化任务分配优化器
        self.task_optimizer = TaskAssignmentOptimizer(self.robots, self.task_manager)

        # 初始化物料
        self.material_counts = {
            "empty": 540,  # 初始540个空桶
            "green": 0,  # 绿桶(梳棉产出)
            "yellow": 0,  # 黄桶(一并产出)
            "red": 0  # 红桶(二并产出)
        }

        # 初始化性能监控器
        self.performance_monitor = PerformanceMonitor()

        # 初始化可视化工具
        self.visualizer = None

        # 全局时间(秒)
        self.global_time = 0

        # 绘制间隔(多少步更新一次可视化)
        self.visualization_interval = 100

        # 批次记录(用于一并和二并的6:1转换)
        self.batch_records = {
            "green": 0,  # 记录已生产但未被一并处理的绿桶数量
            "yellow": 0  # 记录已生产但未被二并处理的黄桶数量
        }

        # 设置日志记录
        if not os.path.exists('logs'):
            os.makedirs('logs')
        self.log_file = f'logs/simulation_{time.strftime("%Y%m%d-%H%M%S")}.log'

        # 多线程支持
        self.running = True
        self.visualization_thread = None

    def init_zones(self):
        """初始化工厂各区域"""
        # 区域尺寸和位置参数
        zone_width = self.width // 9  # 调整区域宽度
        zone_height = self.height // 5  # 调整区域高度
        margin = 50

        # 从左到右定义区域
        # 空桶区
        self.zones[ZoneType.EMPTY_BUCKET] = Zone(
            ZoneType.EMPTY_BUCKET,
            margin, margin,
            margin + zone_width, margin + zone_height,
            capacity=600  # 初始有540个空桶，容量设为600
        )
        self.zones[ZoneType.EMPTY_BUCKET].current_count = 540

        # 梳棉待装填区
        self.zones[ZoneType.CARDING_LOAD] = Zone(
            ZoneType.CARDING_LOAD,
            margin + zone_width + margin, margin,
            margin + 2 * zone_width, margin + zone_height,
            capacity=100  # 增加容量
        )

        # 梳棉已装填区(绿桶区)
        self.zones[ZoneType.CARDING_UNLOAD] = Zone(
            ZoneType.CARDING_UNLOAD,
            margin + 2 * zone_width + margin, margin,
            margin + 3 * zone_width, margin + zone_height,
            capacity=150  # 增加容量
        )

        # 一并待装填区
        self.zones[ZoneType.FIRST_DRAWING_LOAD] = Zone(
            ZoneType.FIRST_DRAWING_LOAD,
            margin + 3 * zone_width + margin, margin,
            margin + 4 * zone_width, margin + zone_height,
            capacity=100  # 增加容量
        )

        # 一并已装填区(黄桶区)
        self.zones[ZoneType.FIRST_DRAWING_UNLOAD] = Zone(
            ZoneType.FIRST_DRAWING_UNLOAD,
            margin + 4 * zone_width + margin, margin,
            margin + 5 * zone_width, margin + zone_height,
            capacity=60  # 增加容量
        )

        # 二并待装填区
        self.zones[ZoneType.SECOND_DRAWING_LOAD] = Zone(
            ZoneType.SECOND_DRAWING_LOAD,
            margin + 5 * zone_width + margin, margin,
            margin + 6 * zone_width, margin + zone_height,
            capacity=60  # 增加容量
        )

        # 二并已装填区(红桶区)
        self.zones[ZoneType.SECOND_DRAWING_UNLOAD] = Zone(
            ZoneType.SECOND_DRAWING_UNLOAD,
            margin + 6 * zone_width + margin, margin,
            margin + 7 * zone_width, margin + zone_height,
            capacity=40  # 增加容量
        )

        # 粗纱区
        self.zones[ZoneType.ROVING] = Zone(
            ZoneType.ROVING,
            margin + 7 * zone_width + margin, margin,
            margin + 8 * zone_width, margin + zone_height,
            capacity=20  # 目标是15个红桶
        )

        # 机器人空闲区
        self.zones[ZoneType.ROBOT_IDLE] = Zone(
            ZoneType.ROBOT_IDLE,
            margin, self.height - margin - zone_height,
                    self.width - margin, self.height - margin,
            capacity=20  # 机器人数量上限
        )

        # 更新代价地图中的区域信息
        for zone_type, zone in self.zones.items():
            self.cost_map.set_zone_rect(zone.x1, zone.y1, zone.x2, zone.y2, zone_type)

    def init_machines(self):
        """初始化工厂的加工设备"""
        # 10台梳棉机，处理时间4225.94秒
        for i in range(10):
            x = self.zones[ZoneType.CARDING_LOAD].x1 + 30 + (i % 5) * 60
            y = self.zones[ZoneType.CARDING_LOAD].y2 + 50 + (i // 5) * 70
            position = Position(x, y)
            machine = Machine(i, MachineType.CARDING, position, 4225.94)
            self.machines[MachineType.CARDING].append(machine)
            # 将机器位置设为障碍物
            self.cost_map.update_base_cost(position, float('inf'))

        # 5台一并机，处理时间2594.75秒
        for i in range(5):
            x = self.zones[ZoneType.FIRST_DRAWING_LOAD].x1 + 30 + i * 60
            y = self.zones[ZoneType.FIRST_DRAWING_LOAD].y2 + 50
            position = Position(x, y)
            machine = Machine(i, MachineType.FIRST_DRAWING, position, 2594.75)
            self.machines[MachineType.FIRST_DRAWING].append(machine)
            # 将机器位置设为障碍物
            self.cost_map.update_base_cost(position, float('inf'))

        # 5台二并机，处理时间2594.75秒
        for i in range(5):
            x = self.zones[ZoneType.SECOND_DRAWING_LOAD].x1 + 30 + i * 60
            y = self.zones[ZoneType.SECOND_DRAWING_LOAD].y2 + 50
            position = Position(x, y)
            machine = Machine(i, MachineType.SECOND_DRAWING, position, 2594.75)
            self.machines[MachineType.SECOND_DRAWING].append(machine)
            # 将机器位置设为障碍物
            self.cost_map.update_base_cost(position, float('inf'))

        # 5台粗纱机
        for i in range(5):
            x = self.zones[ZoneType.ROVING].x1 + 30 + i * 60
            y = self.zones[ZoneType.ROVING].y2 + 50
            position = Position(x, y)
            machine = Machine(i, MachineType.ROVING, position, 0)  # 粗纱机无处理时间
            self.machines[MachineType.ROVING].append(machine)
            # 将机器位置设为障碍物
            self.cost_map.update_base_cost(position, float('inf'))

    def init_robots(self):
        """初始化运输机器人"""
        # 创建10个机器人，均匀分布在空闲区
        robot_zone = self.zones[ZoneType.ROBOT_IDLE]
        robot_spacing_x = (robot_zone.x2 - robot_zone.x1) // 11
        robot_spacing_y = (robot_zone.y2 - robot_zone.y1) // 3

        for i in range(10):
            x = robot_zone.x1 + robot_spacing_x * (i % 5 + 1)
            y = robot_zone.y1 + robot_spacing_y * (i // 5 + 1)
            position = Position(x, y)
            self.robots.append(Robot(i, position))

            # 注册机器人位置到死锁管理器
            self.deadlock_manager.update_robot_position(i, position)

    def log(self, message: str):
        """记录日志"""
        with open(self.log_file, 'a', encoding='utf-8') as f:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"[{timestamp}] {message}\n")

    def generate_tasks(self):
        """生成运输任务"""
        # 根据当前物料状态生成新任务
        tasks_generated = 0

        # 1. 空桶到梳棉待装填区(如果梳棉待装填区有空间，且空桶区有桶)
        empty_zone = self.zones[ZoneType.EMPTY_BUCKET]
        carding_load_zone = self.zones[ZoneType.CARDING_LOAD]

        if empty_zone.current_count > 0 and carding_load_zone.current_count < carding_load_zone.capacity:
            # 可以创建从空桶区到梳棉待装填区的任务
            available_space = carding_load_zone.capacity - carding_load_zone.current_count
            count = min(empty_zone.current_count, available_space, 10)  # 一次最多运10个

            if count > 0:
                start_pos = empty_zone.get_random_position()
                end_pos = carding_load_zone.get_random_position()

                task_id = self.task_manager.create_task(
                    TaskType.EMPTY_TO_CARDING,
                    ZoneType.EMPTY_BUCKET,
                    ZoneType.CARDING_LOAD,
                    start_pos, end_pos,
                    self.global_time,
                    batch_size=count
                )
                tasks_generated += 1
                self.log(f"生成任务 {task_id}: 空桶到梳棉待装填区，批次大小={count}")

        # 2. 梳棉已装填区(绿桶)到一并待装填区
        carding_unload_zone = self.zones[ZoneType.CARDING_UNLOAD]
        first_load_zone = self.zones[ZoneType.FIRST_DRAWING_LOAD]

        if carding_unload_zone.current_count > 0 and first_load_zone.current_count < first_load_zone.capacity:
            # 可以创建从梳棉已装填区到一并待装填区的任务
            available_space = first_load_zone.capacity - first_load_zone.current_count
            count = min(carding_unload_zone.current_count, available_space, 12)  # 一次最多运12个

            if count > 0 and count >= 6:  # 确保一次至少运6个，符合6:1转换比例
                start_pos = carding_unload_zone.get_random_position()
                end_pos = first_load_zone.get_random_position()

                task_id = self.task_manager.create_task(
                    TaskType.CARDING_TO_FIRST,
                    ZoneType.CARDING_UNLOAD,
                    ZoneType.FIRST_DRAWING_LOAD,
                    start_pos, end_pos,
                    self.global_time,
                    batch_size=count
                )
                tasks_generated += 1
                self.log(f"生成任务 {task_id}: 梳棉已装填区到一并待装填区，批次大小={count}")

        # 3. 一并已装填区(黄桶)到二并待装填区
        first_unload_zone = self.zones[ZoneType.FIRST_DRAWING_UNLOAD]
        second_load_zone = self.zones[ZoneType.SECOND_DRAWING_LOAD]

        if first_unload_zone.current_count > 0 and second_load_zone.current_count < second_load_zone.capacity:
            # 可以创建从一并已装填区到二并待装填区的任务
            available_space = second_load_zone.capacity - second_load_zone.current_count
            count = min(first_unload_zone.current_count, available_space, 12)  # 一次最多运12个

            if count > 0 and count >= 6:  # 确保一次至少运6个，符合6:1转换比例
                start_pos = first_unload_zone.get_random_position()
                end_pos = second_load_zone.get_random_position()

                task_id = self.task_manager.create_task(
                    TaskType.FIRST_TO_SECOND,
                    ZoneType.FIRST_DRAWING_UNLOAD,
                    ZoneType.SECOND_DRAWING_LOAD,
                    start_pos, end_pos,
                    self.global_time,
                    batch_size=count
                )
                tasks_generated += 1
                self.log(f"生成任务 {task_id}: 一并已装填区到二并待装填区，批次大小={count}")

        # 4. 二并已装填区(红桶)到粗纱区
        second_unload_zone = self.zones[ZoneType.SECOND_DRAWING_UNLOAD]
        roving_zone = self.zones[ZoneType.ROVING]

        if second_unload_zone.current_count > 0 and roving_zone.current_count < roving_zone.capacity:
            # 可以创建从二并已装填区到粗纱区的任务
            available_space = roving_zone.capacity - roving_zone.current_count
            count = min(second_unload_zone.current_count, available_space, 3)  # 一次最多运3个

            if count > 0:
                start_pos = second_unload_zone.get_random_position()
                end_pos = roving_zone.get_random_position()

                task_id = self.task_manager.create_task(
                    TaskType.SECOND_TO_ROVING,
                    ZoneType.SECOND_DRAWING_UNLOAD,
                    ZoneType.ROVING,
                    start_pos, end_pos,
                    self.global_time,
                    batch_size=count
                )
                tasks_generated += 1
                self.log(f"生成任务 {task_id}: 二并已装填区到粗纱区，批次大小={count}")

        return tasks_generated

    def assign_tasks(self):
        """优化分配任务给空闲机器人"""
        # 使用匈牙利算法优化任务分配
        assignments = self.task_optimizer.optimize_assignment()

        assigned_count = 0
        for robot, task in assignments:
            # 分配任务
            if self.task_manager.assign_task(task.id, robot.id):
                robot.assign_task(task)

                # 规划从当前位置到任务起点的路径
                path = self.path_planner.plan_path(robot.position, task.start_pos)
                if path:
                    robot.set_path(path)
                    assigned_count += 1
                    self.log(f"分配任务 {task.id} 给机器人 {robot.id}")
                else:
                    # 路径规划失败，取消任务分配
                    self.task_manager.fail_task(task.id)
                    robot.task = None
                    robot.status = "idle"
                    self.log(f"路径规划失败: 机器人 {robot.id} 无法到达任务 {task.id} 的起点")

        return assigned_count

    def update_machines(self):
        """更新所有机器状态"""
        completed_batches = {
            MachineType.CARDING: 0,
            MachineType.FIRST_DRAWING: 0,
            MachineType.SECOND_DRAWING: 0
        }

        for machine_type, machine_list in self.machines.items():
            for machine in machine_list:
                # 机器当前状态影响是否是空闲
                is_idle = machine.status == MachineStatus.IDLE and machine.current_batch_size == 0

                if machine.update(is_idle):  # 如果机器完成了批次处理
                    # 处理不同类型机器的产出
                    if machine_type == MachineType.CARDING:
                        # 梳棉机: 空桶 -> 绿桶
                        produced = machine.current_batch_size  # 本次处理的批次大小
                        self.material_counts["green"] += produced
                        self.batch_records["green"] += produced
                        self.zones[ZoneType.CARDING_UNLOAD].current_count += produced
                        completed_batches[MachineType.CARDING] += 1
                        self.log(f"梳棉机 {machine.id} 完成批次处理: {produced}个空桶 -> {produced}个绿桶")

                    elif machine_type == MachineType.FIRST_DRAWING:
                        # 一并机: 6个绿桶 -> 1个黄桶(6:1)
                        green_consumed = machine.current_batch_size
                        yellow_produced = green_consumed // 6
                        self.material_counts["green"] -= green_consumed
                        self.material_counts["yellow"] += yellow_produced
                        self.batch_records["yellow"] += yellow_produced
                        self.zones[ZoneType.FIRST_DRAWING_UNLOAD].current_count += yellow_produced
                        completed_batches[MachineType.FIRST_DRAWING] += 1
                        self.log(f"一并机 {machine.id} 完成批次处理: {green_consumed}个绿桶 -> {yellow_produced}个黄桶")

                    elif machine_type == MachineType.SECOND_DRAWING:
                        # 二并机: 6个黄桶 -> 1个红桶(6:1)
                        yellow_consumed = machine.current_batch_size
                        red_produced = yellow_consumed // 6
                        self.material_counts["yellow"] -= yellow_consumed
                        self.material_counts["red"] += red_produced
                        self.zones[ZoneType.SECOND_DRAWING_UNLOAD].current_count += red_produced
                        completed_batches[MachineType.SECOND_DRAWING] += 1
                        self.log(f"二并机 {machine.id} 完成批次处理: {yellow_consumed}个黄桶 -> {red_produced}个红桶")

        return completed_batches

    def update_robots(self):
        """更新所有机器人状态"""
        # 收集当前所有机器人的任务优先级
        robot_priorities = {}
        for robot in self.robots:
            if robot.task:
                priority = robot.task.priority
                robot_priorities[robot.id] = priority

        # 收集有任务的机器人当前路径
        robot_paths = {}

        for robot in self.robots:
            if robot.task and robot.path and len(robot.path) > 1:
                robot_paths[robot.id] = robot.path
                # 更新死锁管理器中的目标位置
                self.deadlock_manager.update_robot_target(robot.id, robot.path[1])

        # 如果有多个机器人有路径，使用CBS规划器检测和解决冲突
        if len(robot_paths) > 1:
            # 准备CBS的输入：机器人当前位置和目标位置
            robot_tasks_for_cbs = {}
            for rid, path in robot_paths.items():
                if len(path) >= 2:
                    robot_tasks_for_cbs[rid] = (path[0], path[-1])

            # 应用CBS规划
            if robot_tasks_for_cbs:
                adjusted_paths = self.cbs_planner.plan_paths(robot_tasks_for_cbs, robot_priorities)

                # 更新机器人路径
                for rid, path in adjusted_paths.items():
                    robot = next((r for r in self.robots if r.id == rid), None)
                    if robot:
                        robot.path = path

        # 按任务优先级对机器人排序(数值越小优先级越高)
        sorted_robots = sorted(
            [r for r in self.robots if r.task and r.path and len(r.path) > 1],
            key=lambda r: r.task.priority if r.task else 999
        )

        # 位置处理标记(确保高优先级机器人先移动)
        positions_handled = set()

        # 更新机器人位置(高优先级先更新)
        for robot in sorted_robots:
            if robot.status == "moving" and robot.path and len(robot.path) > 1:
                next_pos = robot.path[1]

                # 检查下一个位置是否已被占用
                if next_pos in positions_handled:
                    # 位置已被占用，机器人等待
                    robot.wait()
                    self.log(f"机器人 {robot.id} 等待：目标位置 {next_pos} 已被占用")
                    continue

                # 进行移动，更新死锁管理器中的位置
                if robot.move():
                    self.deadlock_manager.update_robot_position(robot.id, robot.position)
                    positions_handled.add(robot.position)
                    robot.update(is_working=True)
                    self.log(f"机器人 {robot.id} 移动到 {robot.position}")

            elif robot.status == "loading":
                # 机器人到达起点，装载物料
                if robot.task:
                    # 根据任务类型确定装载的物料
                    material_type = None
                    source_zone = None

                    if robot.task.task_type == TaskType.EMPTY_TO_CARDING:
                        material_type = "empty"
                        source_zone = ZoneType.EMPTY_BUCKET
                    elif robot.task.task_type == TaskType.CARDING_TO_FIRST:
                        material_type = "green"
                        source_zone = ZoneType.CARDING_UNLOAD
                    elif robot.task.task_type == TaskType.FIRST_TO_SECOND:
                        material_type = "yellow"
                        source_zone = ZoneType.FIRST_DRAWING_UNLOAD
                    elif robot.task.task_type == TaskType.SECOND_TO_ROVING:
                        material_type = "red"
                        source_zone = ZoneType.SECOND_DRAWING_UNLOAD

                    if material_type and source_zone in self.zones:
                        # 更新区域物料数量
                        zone = self.zones[source_zone]
                        if zone.current_count >= robot.task.batch_size:
                            zone.current_count -= robot.task.batch_size

                            # 装载物料
                            robot.load_material(material_type, robot.task.batch_size)
                            self.log(f"机器人 {robot.id} 装载 {robot.task.batch_size}个{material_type}桶")

                            # 规划到终点的路径
                            path = self.path_planner.plan_path(robot.position, robot.task.end_pos)
                            if path:
                                robot.set_path(path)

                                # 开始执行任务
                                self.task_manager.start_task(robot.task.id)
                                robot.update(is_working=True)
                            else:
                                # 无法到达目标，任务失败
                                self.task_manager.fail_task(robot.task.id)
                                robot.task = None
                                robot.status = "idle"
                                self.log(f"路径规划失败: 机器人 {robot.id} 无法规划到任务 {robot.task.id} 的终点")
                        else:
                            # 区域物料不足，任务失败
                            self.task_manager.fail_task(robot.task.id)
                            robot.task = None
                            robot.status = "idle"
                            self.log(f"物料不足: 区域 {source_zone} 中物料不足以完成任务 {robot.task.id}")

            elif robot.status == "unloading":
                # 机器人到达终点，卸载物料
                if robot.task:
                    # 根据任务类型确定目标区域
                    target_zone = None

                    if robot.task.task_type == TaskType.EMPTY_TO_CARDING:
                        target_zone = ZoneType.CARDING_LOAD
                    elif robot.task.task_type == TaskType.CARDING_TO_FIRST:
                        target_zone = ZoneType.FIRST_DRAWING_LOAD
                    elif robot.task.task_type == TaskType.FIRST_TO_SECOND:
                        target_zone = ZoneType.SECOND_DRAWING_LOAD
                    elif robot.task.task_type == TaskType.SECOND_TO_ROVING:
                        target_zone = ZoneType.ROVING

                    if target_zone in self.zones:
                        # 更新区域物料数量
                        zone = self.zones[target_zone]
                        zone.current_count += robot.task.batch_size

                        # 完成任务
                        self.task_manager.complete_task(robot.task.id, self.global_time)
                        self.performance_monitor.total_tasks_completed += 1

                        # 卸载物料
                        material_type, count = robot.unload_material()
                        self.log(f"机器人 {robot.id} 卸载 {count}个{material_type}桶到 {target_zone}")
                        robot.update(is_working=True)

            else:
                # 其他状态下的机器人更新
                robot.update(is_working=robot.status != "idle")

    def detect_and_resolve_deadlocks(self):
        """检测和解决死锁"""
        # 获取机器人优先级 - 基于任务优先级
        robot_priorities = {}
        for robot in self.robots:
            if robot.task:
                priority = robot.task.priority
                robot_priorities[robot.id] = priority
            else:
                robot_priorities[robot.id] = 999  # 无任务的机器人优先级最低

        # 检测死锁
        deadlock_info = self.deadlock_manager.detect_deadlock()
        if deadlock_info[0] != "NONE":
            # 记录死锁情况
            self.log(f"检测到死锁: 类型={deadlock_info[0]}, 涉及机器人={deadlock_info[1]}")

            # 解决死锁
            affected_robots = self.deadlock_manager.resolve_deadlock(robot_priorities)

            # 对受影响的机器人进行处理(重规划路径或取消任务)
            for robot_id in affected_robots:
                robot = next((r for r in self.robots if r.id == robot_id), None)
                if robot and robot.task:
                    self.log(f"死锁解决: 重规划机器人 {robot_id} 的路径")

                    # 根据当前策略处理
                    if self.deadlock_manager.current_strategy == "PRIORITY":
                        # 优先级策略: 低优先级机器人等待
                        if robot.status == "moving" and len(robot.path) > 1:
                            # 给路径开头添加等待步骤
                            robot.path.insert(0, robot.path[0])
                            self.log(f"机器人 {robot_id} 等待一步以解决死锁")

                    elif self.deadlock_manager.current_strategy == "ABORT":
                        # 终止策略: 取消任务
                        self.task_manager.fail_task(robot.task.id)
                        robot.task = None
                        robot.status = "idle"
                        self.log(f"取消机器人 {robot_id} 的任务以解决死锁")

                    elif self.deadlock_manager.current_strategy == "REPLAN":
                        # 重规划策略: 尝试找新路径
                        if robot.status == "moving":
                            current_pos = robot.position
                            goal_pos = robot.path[-1] if robot.path else None

                            if goal_pos:
                                # 尝试重新规划路径
                                new_path = self.path_planner.plan_path(current_pos, goal_pos)
                                if new_path:
                                    robot.set_path(new_path)
                                    self.log(f"为机器人 {robot_id} 重规划路径以解决死锁")
                                else:
                                    # 无法规划新路径，取消任务
                                    self.task_manager.fail_task(robot.task.id)
                                    robot.task = None
                                    robot.status = "idle"
                                    self.log(f"无法为机器人 {robot_id} 重规划路径，取消任务")

        return deadlock_info

    def process_carding_machines(self):
        """处理梳棉机的物料输入"""
        # 如果梳棉待装填区有空桶，尝试分配给空闲梳棉机
        carding_load_zone = self.zones[ZoneType.CARDING_LOAD]
        machines_loaded = 0

        if carding_load_zone.current_count > 0:
            for machine in self.machines[MachineType.CARDING]:
                if machine.status == MachineStatus.IDLE and machine.current_batch_size == 0:
                    # 梳棉机空闲且未装料，分配一个空桶
                    if carding_load_zone.current_count > 0:
                        carding_load_zone.current_count -= 1
                        machine.load_material(1)
                        self.material_counts["empty"] -= 1
                        machines_loaded += 1

                        # 如果装载后批次已满，会自动开始处理
                        if machine.status == MachineStatus.PROCESSING:
                            self.log(f"梳棉机 {machine.id} 开始处理批次")

        return machines_loaded

    def process_first_drawing_machines(self):
        """处理一并机的物料输入"""
        # 如果一并待装填区有绿桶，尝试分配给空闲一并机
        first_load_zone = self.zones[ZoneType.FIRST_DRAWING_LOAD]
        machines_loaded = 0

        if first_load_zone.current_count >= 6:  # 需要6个绿桶产生1个黄桶
            for machine in self.machines[MachineType.FIRST_DRAWING]:
                if machine.status == MachineStatus.IDLE and machine.current_batch_size < machine.max_batch_size:
                    # 一并机空闲或未满载，分配6个绿桶
                    available = min(first_load_zone.current_count, 6,
                                    machine.max_batch_size - machine.current_batch_size)
                    if available >= 6:  # 确保有完整的6个绿桶可以处理
                        loaded = machine.load_material(6)  # 一次只加载6个
                        first_load_zone.current_count -= loaded
                        machines_loaded += 1

                        if machine.status == MachineStatus.PROCESSING:
                            self.log(f"一并机 {machine.id} 开始处理批次: {loaded}个绿桶")

                        # 如果机器还有容量，且区域还有足够物料，继续装载
                        if machine.status == MachineStatus.IDLE and first_load_zone.current_count >= 6:
                            remaining_capacity = machine.max_batch_size - machine.current_batch_size
                            if remaining_capacity >= 6:
                                additional_load = min(first_load_zone.current_count, 6)
                                loaded = machine.load_material(additional_load)
                                first_load_zone.current_count -= loaded

                                if machine.status == MachineStatus.PROCESSING:
                                    self.log(
                                        f"一并机 {machine.id} 继续装载并开始处理批次: 总计{machine.current_batch_size}个绿桶")

        return machines_loaded

    def process_second_drawing_machines(self):
        """处理二并机的物料输入"""
        # 如果二并待装填区有黄桶，尝试分配给空闲二并机
        second_load_zone = self.zones[ZoneType.SECOND_DRAWING_LOAD]
        machines_loaded = 0

        if second_load_zone.current_count >= 6:  # 需要6个黄桶产生1个红桶
            for machine in self.machines[MachineType.SECOND_DRAWING]:
                if machine.status == MachineStatus.IDLE and machine.current_batch_size < machine.max_batch_size:
                    # 二并机空闲或未满载，分配6个黄桶
                    available = min(second_load_zone.current_count, 6,
                                    machine.max_batch_size - machine.current_batch_size)
                    if available >= 6:  # 确保有完整的6个黄桶可以处理
                        loaded = machine.load_material(6)  # 一次只加载6个
                        second_load_zone.current_count -= loaded
                        machines_loaded += 1

                        if machine.status == MachineStatus.PROCESSING:
                            self.log(f"二并机 {machine.id} 开始处理批次: {loaded}个黄桶")

                        # 如果机器还有容量，且区域还有足够物料，继续装载
                        if machine.status == MachineStatus.IDLE and second_load_zone.current_count >= 6:
                            remaining_capacity = machine.max_batch_size - machine.current_batch_size
                            if remaining_capacity >= 6:
                                additional_load = min(second_load_zone.current_count, 6)
                                loaded = machine.load_material(additional_load)
                                second_load_zone.current_count -= loaded

                                if machine.status == MachineStatus.PROCESSING:
                                    self.log(
                                        f"二并机 {machine.id} 继续装载并开始处理批次: 总计{machine.current_batch_size}个黄桶")

        return machines_loaded

    def start_visualization_thread(self):
        """启动可视化线程"""

        def visualize_loop():
            while self.running:
                if self.visualizer:
                    try:
                        self.visualizer.update_plot()
                        self.visualizer.update_material_plot()
                    except Exception as e:
                        print(f"可视化更新错误: {e}")
                time.sleep(1)  # 更新间隔

        self.visualization_thread = threading.Thread(target=visualize_loop)
        self.visualization_thread.daemon = True
        self.visualization_thread.start()

    def setup_visualization(self):
        """设置可视化"""
        self.visualizer = Visualizer(self)
        self.visualizer.setup_plot()
        self.visualizer.setup_material_plot()
        self.start_visualization_thread()

    def update(self):
        """更新工厂状态"""
        # 更新全局时间
        self.global_time += 1

        # 更新机器状态
        completed_batches = self.update_machines()

        # 生成新任务
        tasks_generated = self.generate_tasks()

        # 分配任务
        tasks_assigned = self.assign_tasks()

        # 更新机器人状态
        self.update_robots()

        # 检测和解决死锁
        deadlock_info = self.detect_and_resolve_deadlocks()

        # 处理梳棉机的物料输入(空桶 -> 绿桶)
        carding_loaded = self.process_carding_machines()

        # 处理一并机的物料输入(绿桶 -> 黄桶)
        first_drawing_loaded = self.process_first_drawing_machines()

        # 处理二并机的物料输入(黄桶 -> 红桶)
        second_drawing_loaded = self.process_second_drawing_machines()

        # 更新性能监控
        self.performance_monitor.update(
            self.global_time, self.machines, self.robots, deadlock_info
        )
        self.performance_monitor.update_material_counts(
            self.material_counts["empty"],
            self.material_counts["green"],
            self.material_counts["yellow"],
            self.material_counts["red"]
        )

        # 记录每步的关键性能指标
        if self.global_time % 100 == 0:
            self.log(f"时间步 {self.global_time}: "
                     f"红桶={self.material_counts['red']}, "
                     f"黄桶={self.material_counts['yellow']}, "
                     f"绿桶={self.material_counts['green']}, "
                     f"空桶={self.material_counts['empty']}")
            self.log(f"性能指标: 任务完成={self.performance_monitor.total_tasks_completed}, "
                     f"死锁次数={self.performance_monitor.deadlock_count}, "
                     f"效率比={self.performance_monitor.get_efficiency_ratio():.2%}")

    def run_simulation(self, max_time=100000, visualize=True):
        """运行模拟"""
        # 初始化性能监控
        self.performance_monitor.start(self.global_time)

        # 设置可视化
        if visualize:
            self.setup_visualization()

        # 记录模拟开始时间
        self.log(f"开始模拟: 目标红桶数量=15, 最大时间={max_time}秒")
        start_real_time = time.time()

        # 运行模拟循环
        try:
            while self.global_time < max_time:
                self.update()

                # 输出进度
                if self.global_time % 1000 == 0:
                    red_buckets = self.material_counts["red"]
                    print(f"时间: {self.global_time}秒, 红桶数量: {red_buckets}/15")

                # 终止条件: 达到15个红桶
                if self.material_counts["red"] >= 15:
                    print(f"目标达成! 已生产15个红桶，用时{self.global_time}秒.")
                    self.log(f"模拟完成: 成功生产15个红桶，用时{self.global_time}秒")
                    break

            # 记录实际运行时间
            end_real_time = time.time()
            real_duration = end_real_time - start_real_time
            self.log(f"模拟完成: 实际运行时间={real_duration:.2f}秒")

            # 停止可视化线程
            self.running = False
            if self.visualization_thread:
                self.visualization_thread.join(timeout=1)

            # 输出最终结果
            final_report = self.performance_monitor.generate_report()
            print(final_report)
            self.log(final_report)

            # 保存可视化结果
            if visualize and self.visualizer:
                self.visualizer.save_plots()
                if hasattr(self.visualizer, 'show'):
                    self.visualizer.show()

            # 返回模拟结果
            return {
                "simulation_time": self.global_time,
                "material_counts": self.material_counts.copy(),
                "completed_tasks": self.performance_monitor.total_tasks_completed,
                "deadlock_count": self.performance_monitor.deadlock_count,
                "efficiency_ratio": self.performance_monitor.get_efficiency_ratio(),
                "real_duration": real_duration
            }

        except KeyboardInterrupt:
            print("模拟被用户中断")
            self.log("模拟被用户中断")

            # 停止可视化线程
            self.running = False
            if self.visualization_thread:
                self.visualization_thread.join(timeout=1)

            # 输出中断时的结果
            interrupted_report = self.performance_monitor.generate_report()
            print(interrupted_report)
            self.log(interrupted_report)

            # 保存可视化结果
            if visualize and self.visualizer:
                self.visualizer.save_plots()

            return None
        except Exception as e:
            print(f"模拟过程出错: {e}")
            self.log(f"模拟错误: {e}")
            import traceback
            traceback.print_exc()
            self.log(traceback.format_exc())

            # 停止可视化线程
            self.running = False
            if self.visualization_thread:
                self.visualization_thread.join(timeout=1)

            return None


# ---------------------------
# 主函数与测试
# ---------------------------
def main():
    """主函数"""
    print("启动工厂自动化生产模拟系统...")

    # 创建工厂模拟实例
    factory = FactorySimulation()

    # 运行模拟
    try:
        result = factory.run_simulation(max_time=100000, visualize=True)

        if result:
            print("\n===== 模拟完成 =====")
            print(f"总模拟时间: {result['simulation_time']}秒")
            print(f"物料状态: {result['material_counts']}")
            print(f"完成任务数: {result['completed_tasks']}")
            print(f"死锁次数: {result['deadlock_count']}")
            print(f"效率比(理论/实际): {result['efficiency_ratio']:.2%}")
            print(f"实际计算时间: {result['real_duration']:.2f}秒")
            print("\n注: 详细报告已保存到log文件和输出目录中")
    except Exception as e:
        print(f"程序运行出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()