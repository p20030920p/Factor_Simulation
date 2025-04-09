"""
任务优先级和任务链测试 V1.0
Task Priority and Task Chain Test V1.0

测试内容：
1. 基础任务生成
2. 任务优先级计算
3. 任务链依赖关系
4. 生产流程任务序列
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional, Set
import matplotlib.pyplot as plt
from datetime import datetime
import heapq


@dataclass
class Position:
    """位置类"""
    x: int
    y: int

    def __str__(self) -> str:
        return f"Pos({self.x}, {self.y})"


@dataclass
class Task:
    """任务类"""
    id: int
    type: int  # 1: 绿桶, 2: 黄桶, 3: 红桶
    state: str  # 'open', 'assigned', 'executing', 'completed'
    start: Position
    end: Position
    open_time: int
    deadline: int
    priority: float = 0.0
    assigned_to: int = 0
    next_task_id: int = 0
    completion_time: int = 0

    def __str__(self) -> str:
        type_names = {1: "绿桶", 2: "黄桶", 3: "红桶"}
        return (f"任务{self.id}({type_names[self.type]}): "
                f"{self.start}->{self.end}, 优先级={self.priority:.2f}")


class TaskPriorityManager:
    """任务优先级管理器"""

    def __init__(self):
        self.tasks: Dict[int, Task] = {}
        self.weights = {
            'deadline': 0.3,  # 截止时间权重
            'chain': 0.3,  # 任务链权重
            'type': 0.2,  # 任务类型权重
            'waiting': 0.2  # 等待时间权重
        }

    def add_task(self, task: Task) -> None:
        """添加任务"""
        self.tasks[task.id] = task
        self.update_task_priority(task)

    def update_task_priority(self, task: Task, current_time: int = 0) -> None:
        """更新任务优先级"""
        deadline_score = self._calculate_deadline_score(task, current_time)
        chain_score = self._calculate_chain_score(task)
        type_score = self._calculate_type_score(task)
        waiting_score = self._calculate_waiting_score(task, current_time)

        task.priority = (
                self.weights['deadline'] * deadline_score +
                self.weights['chain'] * chain_score +
                self.weights['type'] * type_score +
                self.weights['waiting'] * waiting_score
        )

    def _calculate_deadline_score(self, task: Task, current_time: int) -> float:
        """计算截止时间得分"""
        if task.deadline <= current_time:
            return 1.0
        time_remaining = task.deadline - current_time
        total_time = task.deadline - task.open_time
        return 1.0 - (time_remaining / total_time)

    def _calculate_chain_score(self, task: Task) -> float:
        """计算任务链得分"""
        if task.next_task_id == 0:
            return 0.0
        chain_length = self._get_chain_length(task.id)
        position = self._get_position_in_chain(task.id)
        return 1.0 - (position / chain_length) if chain_length > 0 else 0.0

    def _calculate_type_score(self, task: Task) -> float:
        """计算任务类型得分"""
        type_priorities = {
            1: 0.6,  # 绿桶任务优先级
            2: 0.8,  # 黄桶任务优先级
            3: 1.0  # 红桶任务优先级（最高）
        }
        return type_priorities.get(task.type, 0.0)

    def _calculate_waiting_score(self, task: Task, current_time: int) -> float:
        """计算等待时间得分"""
        waiting_time = current_time - task.open_time
        max_wait = task.deadline - task.open_time
        return min(1.0, waiting_time / max_wait) if max_wait > 0 else 0.0

    def _get_chain_length(self, task_id: int) -> int:
        """获取任务链长度"""
        length = 1
        current_id = task_id
        while current_id in self.tasks and self.tasks[current_id].next_task_id != 0:
            current_id = self.tasks[current_id].next_task_id
            length += 1
        return length

    def _get_position_in_chain(self, task_id: int) -> int:
        """获取任务在链中的位置"""
        position = 0
        current_id = task_id
        while current_id in self.tasks and self.tasks[current_id].next_task_id != 0:
            position += 1
            current_id = self.tasks[current_id].next_task_id
        return position


class ProductionTaskGenerator:
    """生产任务生成器"""

    def __init__(self, map_size: int = 100):
        self.map_size = map_size
        self.current_id = 0

        # 定义区域
        self.areas = {
            'green_start': [(10, 10), (20, 20)],  # 绿桶起始区域
            'green_end': [(30, 10), (40, 20)],  # 绿桶结束区域
            'yellow_start': [(30, 30), (40, 40)],  # 黄桶起始区域
            'yellow_end': [(50, 30), (60, 40)],  # 黄桶结束区域
            'red_start': [(50, 50), (60, 60)],  # 红桶起始区域
            'red_end': [(70, 50), (80, 60)]  # 红桶结束区域
        }

    def generate_task(self, task_type: int, open_time: int) -> Task:
        """生成单个任务"""
        self.current_id += 1

        # 根据任务类型选择区域
        if task_type == 1:  # 绿桶任务
            start_area = self.areas['green_start']
            end_area = self.areas['green_end']
        elif task_type == 2:  # 黄桶任务
            start_area = self.areas['yellow_start']
            end_area = self.areas['yellow_end']
        else:  # 红桶任务
            start_area = self.areas['red_start']
            end_area = self.areas['red_end']

        # 在区域内随机选择位置
        start = Position(
            np.random.randint(start_area[0][0], start_area[1][0]),
            np.random.randint(start_area[0][1], start_area[1][1])
        )
        end = Position(
            np.random.randint(end_area[0][0], end_area[1][0]),
            np.random.randint(end_area[0][1], end_area[1][1])
        )

        return Task(
            id=self.current_id,
            type=task_type,
            state='open',
            start=start,
            end=end,
            open_time=open_time,
            deadline=open_time + 1000  # 设置一个基础截止时间
        )

    def generate_production_chain(self) -> List[Task]:
        """生成完整的生产任务链"""
        tasks = []
        current_time = 0

        # 生成绿桶任务
        green_task = self.generate_task(1, current_time)
        tasks.append(green_task)

        # 生成黄桶任务
        yellow_task = self.generate_task(2, current_time)
        green_task.next_task_id = yellow_task.id
        tasks.append(yellow_task)

        # 生成红桶任务
        red_task = self.generate_task(3, current_time)
        yellow_task.next_task_id = red_task.id
        tasks.append(red_task)

        return tasks


def visualize_tasks(tasks: List[Task], map_size: int = 100):
    """可视化任务分布"""
    plt.figure(figsize=(12, 12))
    plt.grid(True)

    # 设置图形范围
    plt.xlim(0, map_size)
    plt.ylim(0, map_size)

    # 任务类型的颜色映射
    colors = {1: 'g', 2: 'y', 3: 'r'}
    type_names = {1: "绿桶", 2: "黄桶", 3: "红桶"}

    # 绘制每个任务
    for task in tasks:
        color = colors[task.type]

        # 绘制起点和终点
        plt.plot(task.start.x, task.start.y, f'{color}o', markersize=10,
                 label=f'{type_names[task.type]}起点')
        plt.plot(task.end.x, task.end.y, f'{color}*', markersize=10,
                 label=f'{type_names[task.type]}终点')

        # 绘制任务路径
        plt.plot([task.start.x, task.end.x], [task.start.y, task.end.y],
                 f'{color}--', linewidth=2)

        # 如果有下一个任务，绘制任务链连接
        if task.next_task_id != 0:
            next_task = next(t for t in tasks if t.id == task.next_task_id)
            plt.plot([task.end.x, next_task.start.x],
                     [task.end.y, next_task.start.y],
                     'k:', linewidth=1)

    plt.title('生产任务分布图')
    plt.legend()
    plt.show()


def test_task_priority():
    """测试任务优先级计算"""
    print("\n=== 测试任务优先级计算 ===")

    # 创建任务优先级管理器
    priority_manager = TaskPriorityManager()

    # 生成测试任务
    generator = ProductionTaskGenerator()
    tasks = generator.generate_production_chain()

    # 添加任务到管理器
    for task in tasks:
        priority_manager.add_task(task)
        print(f"\n添加任务: {task}")

    # 测试不同时间点的优先级
    test_times = [0, 200, 500]
    for current_time in test_times:
        print(f"\n时间 {current_time} 的任务优先级:")
        for task in tasks:
            priority_manager.update_task_priority(task, current_time)
            print(f"任务 {task.id}: {task.priority:.3f}")

    # 可视化任务分布
    visualize_tasks(tasks)


def test_production_chain():
    """测试生产任务链生成"""
    print("\n=== 测试生产任务链生成 ===")

    generator = ProductionTaskGenerator()

    # 生成多个生产任务链
    num_chains = 3
    all_tasks = []

    for i in range(num_chains):
        print(f"\n生成第 {i + 1} 条生产链:")
        chain_tasks = generator.generate_production_chain()
        all_tasks.extend(chain_tasks)

        for task in chain_tasks:
            print(f"  {task}")

    # 可视化所有任务
    visualize_tasks(all_tasks)


if __name__ == "__main__":
    # 测试任务优先级计算
    test_task_priority()

    # 测试生产任务链生成
    test_production_chain()