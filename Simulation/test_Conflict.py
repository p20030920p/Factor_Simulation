"""
多机器人冲突检测测试 V1.0
Multi-Robot Conflict Detection Test V1.0

测试内容：
1. 基本冲突检测（顶点冲突和边缘冲突）
2. 多机器人路径冲突
3. 冲突可视化
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional, Set
import matplotlib.pyplot as plt
import heapq

@dataclass
class Position:
    """位置类"""
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

@dataclass
class Conflict:
    """冲突类"""
    robot1_id: int
    robot2_id: int
    pos: Position
    time_step: int
    type: str = 'vertex'  # 'vertex' 或 'edge'
    pos1_prev: Optional[Position] = None
    pos2_prev: Optional[Position] = None

    def __str__(self):
        if self.type == 'vertex':
            return f"顶点冲突: 机器人{self.robot1_id}和{self.robot2_id}在时间{self.time_step}发生在位置{self.pos}"
        return f"边缘冲突: 机器人{self.robot1_id}和{self.robot2_id}在时间{self.time_step}交叉路径"

class ConflictDetector:
    """冲突检测器"""

    @staticmethod
    def detect_vertex_conflict(pos1: Position, pos2: Position) -> bool:
        """检测顶点冲突"""
        return pos1 == pos2

    @staticmethod
    def detect_edge_conflict(pos1_curr: Position, pos1_prev: Position,
                           pos2_curr: Position, pos2_prev: Position) -> bool:
        """检测边缘冲突"""
        return pos1_curr == pos2_prev and pos2_curr == pos1_prev

    @staticmethod
    def detect_conflicts(robot_paths: Dict[int, List[Position]]) -> List[Conflict]:
        """检测所有路径中的冲突"""
        conflicts = []
        robot_ids = list(robot_paths.keys())

        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                robot1_id = robot_ids[i]
                robot2_id = robot_ids[j]
                path1 = robot_paths[robot1_id]
                path2 = robot_paths[robot2_id]

                max_time = max(len(path1), len(path2))

                for t in range(max_time):
                    # 获取当前位置
                    pos1 = path1[min(t, len(path1) - 1)]
                    pos2 = path2[min(t, len(path2) - 1)]

                    # 检测顶点冲突
                    if ConflictDetector.detect_vertex_conflict(pos1, pos2):
                        conflicts.append(Conflict(
                            robot1_id=robot1_id,
                            robot2_id=robot2_id,
                            pos=pos1,
                            time_step=t,
                            type='vertex'
                        ))
                        continue

                    # 检测边缘冲突
                    if t > 0:
                        pos1_prev = path1[min(t - 1, len(path1) - 1)]
                        pos2_prev = path2[min(t - 1, len(path2) - 1)]

                        if ConflictDetector.detect_edge_conflict(pos1, pos1_prev, pos2, pos2_prev):
                            conflicts.append(Conflict(
                                robot1_id=robot1_id,
                                robot2_id=robot2_id,
                                pos=pos1,
                                time_step=t,
                                type='edge',
                                pos1_prev=pos1_prev,
                                pos2_prev=pos2_prev
                            ))

        return conflicts

def visualize_conflicts(robot_paths: Dict[int, List[Position]],
                       conflicts: List[Conflict],
                       map_size: Tuple[int, int] = (10, 10)):
    """可视化冲突"""
    plt.figure(figsize=(10, 10))
    plt.grid(True)

    # 设置图形范围
    plt.xlim(-1, map_size[0])
    plt.ylim(-1, map_size[1])

    # 绘制每个机器人的路径
    colors = ['b', 'r', 'g', 'y', 'm', 'c']
    for robot_id, path in robot_paths.items():
        x = [p.x for p in path]
        y = [p.y for p in path]
        color = colors[robot_id % len(colors)]

        # 绘制路径
        plt.plot(x, y, f'{color}-o', label=f'Robot {robot_id} Path', linewidth=2)

        # 标记起点和终点
        plt.plot(x[0], y[0], f'{color}o', markersize=15, label=f'Robot {robot_id} Start')
        plt.plot(x[-1], y[-1], f'{color}*', markersize=15, label=f'Robot {robot_id} Goal')

    # 标记冲突点
    for conflict in conflicts:
        if conflict.type == 'vertex':
            plt.plot(conflict.pos.x, conflict.pos.y, 'kx', markersize=15,
                    label=f'Vertex Conflict t={conflict.time_step}')
        else:
            plt.plot([conflict.pos1_prev.x, conflict.pos.x],
                    [conflict.pos1_prev.y, conflict.pos.y],
                    'k--', linewidth=2)
            plt.plot(conflict.pos.x, conflict.pos.y, 'rx', markersize=15,
                    label=f'Edge Conflict t={conflict.time_step}')

    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.title('Robot Paths and Conflicts')
    plt.tight_layout()
    plt.show()

def test_basic_conflicts():
    """基础冲突测试"""
    print("\n=== 基础冲突测试 ===")

    # 测试用例1：顶点冲突
    print("\n测试用例1：顶点冲突")
    paths1 = {
        1: [Position(0,0), Position(1,1), Position(2,2)],
        2: [Position(2,2), Position(1,1), Position(0,0)]
    }
    conflicts1 = ConflictDetector.detect_conflicts(paths1)
    print("检测到的冲突:")
    for conflict in conflicts1:
        print(conflict)
    visualize_conflicts(paths1, conflicts1)

    # 测试用例2：边缘冲突
    print("\n测试用例2：边缘冲突")
    paths2 = {
        1: [Position(0,0), Position(1,0), Position(2,0)],
        2: [Position(2,0), Position(1,0), Position(0,0)]
    }
    conflicts2 = ConflictDetector.detect_conflicts(paths2)
    print("检测到的冲突:")
    for conflict in conflicts2:
        print(conflict)
    visualize_conflicts(paths2, conflicts2)

def test_complex_conflicts():
    """复杂冲突测试"""
    print("\n=== 复杂冲突测试 ===")

    # 创建多个机器人的复杂路径
    paths = {
        1: [Position(0,0), Position(1,1), Position(2,2), Position(3,3)],
        2: [Position(0,3), Position(1,2), Position(2,1), Position(3,0)],
        3: [Position(0,1), Position(1,1), Position(2,1), Position(3,1)]
    }

    print("\n测试路径:")
    for robot_id, path in paths.items():
        print(f"机器人 {robot_id}: {[(p.x, p.y) for p in path]}")

    conflicts = ConflictDetector.detect_conflicts(paths)

    print("\n检测到的冲突:")
    for conflict in conflicts:
        print(conflict)

    visualize_conflicts(paths, conflicts, map_size=(4, 4))

if __name__ == "__main__":
    # 运行基础冲突测试
    test_basic_conflicts()

    # 运行复杂冲突测试
    test_complex_conflicts()