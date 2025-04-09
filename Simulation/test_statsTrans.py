"""
任务状态转换与执行监控测试 V1.0
Task State Transition and Execution Monitoring Test V1.0

测试内容：
1. 任务状态机
2. 执行进度监控
3. 任务完成验证
4. 异常状态处理
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional, Set
from enum import Enum
import matplotlib.pyplot as plt
from datetime import datetime

class TaskState(Enum):
    """任务状态枚举"""
    PENDING = "pending"       # 等待分配
    ASSIGNED = "assigned"     # 已分配
    EXECUTING = "executing"   # 执行中
    COMPLETED = "completed"   # 已完成
    FAILED = "failed"         # 执行失败
    CANCELLED = "cancelled"   # 已取消

class TaskEvent(Enum):
    """任务事件枚举"""
    ASSIGN = "assign"         # 分配任务
    START = "start"           # 开始执行
    COMPLETE = "complete"     # 完成任务
    FAIL = "fail"            # 任务失败
    CANCEL = "cancel"        # 取消任务
    RETRY = "retry"          # 重试任务

@dataclass
class TaskExecution:
    """任务执行信息"""
    task_id: int
    robot_id: int
    start_time: int
    estimated_duration: int
    progress: float = 0.0
    actual_duration: Optional[int] = None
    state: TaskState = TaskState.PENDING
    failure_reason: Optional[str] = None

class TaskExecutionMonitor:
    """任务执行监控器"""
    def __init__(self):
        self.executions: Dict[int, TaskExecution] = {}
        self.state_transitions = {
            TaskState.PENDING: {
                TaskEvent.ASSIGN: TaskState.ASSIGNED,
                TaskEvent.CANCEL: TaskState.CANCELLED
            },
            TaskState.ASSIGNED: {
                TaskEvent.START: TaskState.EXECUTING,
                TaskEvent.CANCEL: TaskState.CANCELLED
            },
            TaskState.EXECUTING: {
                TaskEvent.COMPLETE: TaskState.COMPLETED,
                TaskEvent.FAIL: TaskState.FAILED
            },
            TaskState.FAILED: {
                TaskEvent.RETRY: TaskState.PENDING,
                TaskEvent.CANCEL: TaskState.CANCELLED
            }
        }

    def add_execution(self, execution: TaskExecution) -> None:
        """添加任务执行记录"""
        self.executions[execution.task_id] = execution

    def update_state(self, task_id: int, event: TaskEvent) -> bool:
        """更新任务状态"""
        if task_id not in self.executions:
            return False

        execution = self.executions[task_id]
        current_state = execution.state

        if current_state in self.state_transitions and event in self.state_transitions[current_state]:
            execution.state = self.state_transitions[current_state][event]
            return True
        return False

    def update_progress(self, task_id: int, progress: float) -> None:
        """更新任务进度"""
        if task_id in self.executions:
            self.executions[task_id].progress = min(max(progress, 0.0), 1.0)

    def get_execution_stats(self) -> Dict[TaskState, int]:
        """获取执行统计信息"""
        stats = {state: 0 for state in TaskState}
        for execution in self.executions.values():
            stats[execution.state] += 1
        return stats

def test_state_transitions():
    """测试状态转换"""
    print("\n=== 测试任务状态转换 ===")

    monitor = TaskExecutionMonitor()

    # 创建测试任务
    execution = TaskExecution(
        task_id=1,
        robot_id=1,
        start_time=0,
        estimated_duration=100
    )
    monitor.add_execution(execution)

    # 测试正常流程
    transitions = [
        (TaskEvent.ASSIGN, TaskState.ASSIGNED),
        (TaskEvent.START, TaskState.EXECUTING),
        (TaskEvent.COMPLETE, TaskState.COMPLETED)
    ]

    print("\n正常执行流程测试:")
    for event, expected_state in transitions:
        success = monitor.update_state(1, event)
        actual_state = monitor.executions[1].state
        print(f"事件: {event.value}, 成功: {success}, "
              f"期望状态: {expected_state.value}, 实际状态: {actual_state.value}")

    # 测试失败流程
    execution = TaskExecution(
        task_id=2,
        robot_id=2,
        start_time=0,
        estimated_duration=100
    )
    monitor.add_execution(execution)

    print("\n失败处理流程测试:")
    fail_transitions = [
        (TaskEvent.ASSIGN, TaskState.ASSIGNED),
        (TaskEvent.START, TaskState.EXECUTING),
        (TaskEvent.FAIL, TaskState.FAILED),
        (TaskEvent.RETRY, TaskState.PENDING)
    ]

    for event, expected_state in fail_transitions:
        success = monitor.update_state(2, event)
        actual_state = monitor.executions[2].state
        print(f"事件: {event.value}, 成功: {success}, "
              f"期望状态: {expected_state.value}, 实际状态: {actual_state.value}")

def test_progress_monitoring():
    """测试进度监控"""
    print("\n=== 测试任务进度监控 ===")

    monitor = TaskExecutionMonitor()

    # 创建多个测试任务
    tasks = []
    for i in range(5):
        execution = TaskExecution(
            task_id=i,
            robot_id=i % 2,
            start_time=0,
            estimated_duration=100
        )
        monitor.add_execution(execution)
        tasks.append(execution)

    # 模拟进度更新
    progress_updates = [0.0, 0.25, 0.5, 0.75, 1.0]
    for progress in progress_updates:
        print(f"\n更新进度到 {progress*100}%")
        for task in tasks:
            monitor.update_progress(task.task_id, progress)  # 修改这里：使用task.task_id
            print(f"任务 {task.task_id}: {task.progress*100}%")

    # 可视化进度
    plt.figure(figsize=(10, 6))
    task_ids = [task.task_id for task in tasks]  # 修改这里：使用task.task_id
    progress = [task.progress for task in tasks]

    plt.bar(task_ids, progress)
    plt.xlabel('任务ID')
    plt.ylabel('完成进度')
    plt.title('任务执行进度')
    plt.ylim(0, 1)

    # 添加进度标签
    for i, v in enumerate(progress):
        plt.text(i, v, f'{v*100}%', ha='center', va='bottom')

    plt.show()

def test_execution_monitoring():
    """测试执行监控"""
    print("\n=== 测试任务执行监控 ===")

    monitor = TaskExecutionMonitor()

    # 创建不同状态的任务
    states = [
        (1, [TaskEvent.ASSIGN, TaskEvent.START, TaskEvent.COMPLETE]),
        (2, [TaskEvent.ASSIGN, TaskEvent.START]),
        (3, [TaskEvent.ASSIGN, TaskEvent.START, TaskEvent.FAIL]),
        (4, [TaskEvent.ASSIGN]),
        (5, [TaskEvent.ASSIGN, TaskEvent.CANCEL])
    ]

    for task_id, events in states:
        execution = TaskExecution(
            task_id=task_id,
            robot_id=task_id,
            start_time=0,
            estimated_duration=100
        )
        monitor.add_execution(execution)

        for event in events:
            monitor.update_state(task_id, event)

    # 显示统计信息
    stats = monitor.get_execution_stats()
    print("\n任务状态统计:")
    for state, count in stats.items():
        print(f"{state.value}: {count}个任务")

    # 可视化状态分布
    plt.figure(figsize=(10, 6))
    states = [state.value for state in stats.keys()]
    counts = list(stats.values())

    plt.bar(states, counts)
    plt.xlabel('任务状态')
    plt.ylabel('任务数量')
    plt.title('任务状态分布')
    plt.xticks(rotation=45)

    # 添加数量标签
    for i, v in enumerate(counts):
        plt.text(i, v, str(v), ha='center', va='bottom')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 测试状态转换
    test_state_transitions()

    # 测试进度监控
    test_progress_monitoring()

    # 测试执行监控
    test_execution_monitoring()