"""
死锁系统测试模块 test_deadlock.py
测试内容：
1. 等待图构建与分析
2. 死锁检测与分类
3. 死锁解决策略
4. 预防和避免机制
"""

import unittest
from dataclasses import dataclass
from typing import Dict, List, Set, Optional, Tuple
from enum import Enum
import networkx as nx

class DeadlockType(Enum):
    DIRECT = "direct"  # 直接死锁
    INDIRECT = "indirect"  # 间接死锁
    RESOURCE = "resource"  # 资源死锁
    NONE = "none"  # 无死锁

class ResourceType(Enum):
    MACHINE = "machine"  # 加工设备
    STORAGE = "storage"  # 存储区
    PATH = "path"  # 路径点
    CHARGING = "charging"  # 充电桩

@dataclass
class Resource:
    id: str
    type: ResourceType
    capacity: int = 1  # 资源容量
    occupied_by: Set[str] = None  # 被哪些机器人占用
    waiting_robots: List[str] = None  # 等待该资源的机器人队列

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
    id: str
    current_resources: Set[str] = None  # 当前占用的资源集合
    requested_resource: Optional[str] = None  # 请求的资源

    def __post_init__(self):
        if self.current_resources is None:
            self.current_resources = set()

class DeadlockDetector:
    def __init__(self):
        self.resources: Dict[str, Resource] = {}
        self.robots: Dict[str, Robot] = {}
        self.wait_graph = nx.DiGraph()
        self.deadlock_history: List[Tuple[DeadlockType, Set[str]]] = []  # 死锁历史记录

    def add_resource(self, resource: Resource) -> None:
        """添加资源"""
        self.resources[resource.id] = resource

    def add_robot(self, robot: Robot) -> None:
        """添加机器人"""
        self.robots[robot.id] = robot

    def request_resource(self, robot_id: str, resource_id: str) -> bool:
        """机器人请求资源"""
        if resource_id not in self.resources or robot_id not in self.robots:
            return False

        robot = self.robots[robot_id]
        resource = self.resources[resource_id]

        # 如果机器人已经占用该资源
        if resource_id in robot.current_resources:
            return True

        # 如果资源未满且机器人未在等待队列中
        if not resource.is_full and robot_id not in resource.waiting_robots:
            resource.occupied_by.add(robot_id)
            robot.current_resources.add(resource_id)
            return True
        else:
            # 加入等待队列
            if robot_id not in resource.waiting_robots:
                resource.waiting_robots.append(robot_id)
            robot.requested_resource = resource_id
            self._update_wait_graph()
            return False

    def release_resource(self, robot_id: str, resource_id: str) -> bool:
        """释放资源"""
        if resource_id not in self.resources or robot_id not in self.robots:
            return False

        resource = self.resources[resource_id]
        robot = self.robots[robot_id]

        if robot_id not in resource.occupied_by:
            return False

        # 释放资源
        resource.occupied_by.remove(robot_id)
        robot.current_resources.remove(resource_id)

        # 处理等待队列
        if resource.waiting_robots and not resource.is_full:
            next_robot_id = resource.waiting_robots.pop(0)
            next_robot = self.robots[next_robot_id]
            resource.occupied_by.add(next_robot_id)
            next_robot.current_resources.add(resource_id)
            next_robot.requested_resource = None

        self._update_wait_graph()
        return True

    def _update_wait_graph(self) -> None:
        """更新等待图"""
        self.wait_graph.clear()

        # 添加顶点
        for robot_id in self.robots:
            self.wait_graph.add_node(robot_id)

        # 添加边
        for robot_id, robot in self.robots.items():
            if robot.requested_resource:
                resource = self.resources[robot.requested_resource]
                for blocking_robot in resource.occupied_by:
                    self.wait_graph.add_edge(robot_id, blocking_robot)

    def detect_deadlock(self) -> Tuple[DeadlockType, Set[str]]:
        """检测死锁类型和涉及的机器人"""
        # 检查循环等待
        try:
            cycles = list(nx.simple_cycles(self.wait_graph))
        except:
            cycles = []

        if cycles:
            involved_robots = set().union(*[set(cycle) for cycle in cycles])
            if any(len(cycle) == 2 for cycle in cycles):
                return DeadlockType.DIRECT, involved_robots
            return DeadlockType.INDIRECT, involved_robots

        # 检查资源死锁
        for resource in self.resources.values():
            if len(resource.waiting_robots) > 2 * resource.capacity:
                return DeadlockType.RESOURCE, set(resource.waiting_robots)

        return DeadlockType.NONE, set()

    def resolve_deadlock(self) -> List[str]:
        """解决死锁"""
        deadlock_type, involved_robots = self.detect_deadlock()
        if deadlock_type == DeadlockType.NONE:
            return []

        self.deadlock_history.append((deadlock_type, involved_robots))
        affected_robots = []

        if deadlock_type in [DeadlockType.DIRECT, DeadlockType.INDIRECT]:
            # 选择一个机器人进行回退
            robot_to_backoff = next(iter(involved_robots))
            affected_robots.append(robot_to_backoff)
            robot = self.robots[robot_to_backoff]

            if robot.requested_resource:
                resource = self.resources[robot.requested_resource]
                if robot_to_backoff in resource.waiting_robots:
                    resource.waiting_robots.remove(robot_to_backoff)
                robot.requested_resource = None

        elif deadlock_type == DeadlockType.RESOURCE:
            # 对等待时间最长的机器人进行回退
            for resource in self.resources.values():
                if len(resource.waiting_robots) > 2 * resource.capacity:
                    # 选择前N个等待的机器人进行回退
                    robots_to_remove = resource.waiting_robots[:resource.capacity]
                    affected_robots.extend(robots_to_remove)

                    for robot_id in robots_to_remove:
                        resource.waiting_robots.remove(robot_id)
                        self.robots[robot_id].requested_resource = None

        self._update_wait_graph()
        return affected_robots

class TestDeadlockSystem(unittest.TestCase):
    def setUp(self):
        """测试初始化"""
        self.detector = DeadlockDetector()

        # 添加测试资源
        self.detector.add_resource(Resource("R1", ResourceType.MACHINE))
        self.detector.add_resource(Resource("R2", ResourceType.MACHINE))
        self.detector.add_resource(Resource("R3", ResourceType.MACHINE))
        self.detector.add_resource(Resource("S1", ResourceType.STORAGE, capacity=2))

        # 添加测试机器人
        for i in range(1, 6):
            self.detector.add_robot(Robot(f"Bot{i}"))

    def test_direct_deadlock(self):
        """测试直接死锁检测"""
        # 创建直接死锁场景
        self.detector.request_resource("Bot1", "R1")
        self.detector.request_resource("Bot2", "R2")
        self.detector.request_resource("Bot1", "R2")
        self.detector.request_resource("Bot2", "R1")

        deadlock_type, involved_robots = self.detector.detect_deadlock()
        self.assertEqual(deadlock_type, DeadlockType.DIRECT)
        self.assertEqual(len(involved_robots), 2)

        # 测试死锁解决
        affected_robots = self.detector.resolve_deadlock()
        self.assertEqual(len(affected_robots), 1)

        # 验证死锁已解决
        new_type, _ = self.detector.detect_deadlock()
        self.assertEqual(new_type, DeadlockType.NONE)

    def test_indirect_deadlock(self):
        """测试间接死锁检测"""
        # 创建间接死锁场景
        self.detector.request_resource("Bot1", "R1")
        self.detector.request_resource("Bot2", "R2")
        self.detector.request_resource("Bot3", "R3")
        self.detector.request_resource("Bot1", "R2")
        self.detector.request_resource("Bot2", "R3")
        self.detector.request_resource("Bot3", "R1")

        deadlock_type, involved_robots = self.detector.detect_deadlock()
        self.assertEqual(deadlock_type, DeadlockType.INDIRECT)
        self.assertEqual(len(involved_robots), 3)

    def test_resource_deadlock(self):
        """测试资源死锁检测"""
        # 创建资源死锁场景
        self.detector.request_resource("Bot1", "R1")  # Bot1获取R1

        # 多个机器人等待同一资源
        for i in range(2, 6):
            self.detector.request_resource(f"Bot{i}", "R1")

        deadlock_type, involved_robots = self.detector.detect_deadlock()
        self.assertEqual(deadlock_type, DeadlockType.RESOURCE)
        self.assertEqual(len(involved_robots), 4)  # 4个等待的机器人

    def test_shared_resource(self):
        """测试共享资源场景"""
        storage = self.detector.resources["S1"]  # 容量为2的存储区

        # 两个机器人可以同时访问
        self.assertTrue(self.detector.request_resource("Bot1", "S1"))
        self.assertTrue(self.detector.request_resource("Bot2", "S1"))

        # 第三个机器人需要等待
        self.assertFalse(self.detector.request_resource("Bot3", "S1"))

        # 释放一个位置
        self.detector.release_resource("Bot1", "S1")

        # 验证等待的机器人可以获取资源
        self.assertIn("Bot3", storage.occupied_by)

    def test_deadlock_history(self):
        """测试死锁历史记录"""
        # 创建并解决多个死锁
        self.detector.request_resource("Bot1", "R1")
        self.detector.request_resource("Bot2", "R2")
        self.detector.request_resource("Bot1", "R2")
        self.detector.request_resource("Bot2", "R1")
        self.detector.resolve_deadlock()

        self.detector.request_resource("Bot3", "R1")
        self.detector.request_resource("Bot4", "R1")
        self.detector.request_resource("Bot5", "R1")
        self.detector.resolve_deadlock()

        # 验证历史记录
        self.assertEqual(len(self.detector.deadlock_history), 2)
        self.assertEqual(self.detector.deadlock_history[0][0], DeadlockType.DIRECT)
        self.assertEqual(self.detector.deadlock_history[1][0], DeadlockType.RESOURCE)

if __name__ == '__main__':
    unittest.main()