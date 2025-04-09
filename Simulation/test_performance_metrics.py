import unittest
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set
from enum import Enum
from datetime import datetime, timedelta
import statistics
from collections import defaultdict


# 基础类定义
class MaterialType(Enum):
    GREEN = "green"
    YELLOW = "yellow"
    RED = "red"
    EMPTY = "empty"


class MachineType(Enum):
    CARDING = "carding"
    FIRST_DRAW = "first"
    SECOND_DRAW = "second"


class ProductionState(Enum):
    IDLE = "idle"
    PROCESSING = "processing"
    BLOCKED = "blocked"
    STARVED = "starved"


@dataclass
class Material:
    id: str
    type: MaterialType
    location: Optional[str] = None
    next_process: Optional[str] = None


@dataclass
class Machine:
    id: str
    type: MachineType
    input_buffer: List[Material] = field(default_factory=list)
    output_buffer: List[Material] = field(default_factory=list)
    state: ProductionState = ProductionState.IDLE
    processing_time: int = 0
    current_material: Optional[Material] = None
    buffer_capacity: int = 3


class ProductionSystem:
    def __init__(self):
        self.machines: Dict[str, Machine] = {}
        self.materials: Dict[str, Material] = {}
        self.time = 0

    def add_machine(self, machine: Machine) -> None:
        self.machines[machine.id] = machine

    def add_material(self, material: Material) -> None:
        self.materials[material.id] = material

    def can_process(self, machine: Machine, material: Material) -> bool:
        material_machine_map = {
            MaterialType.GREEN: MachineType.CARDING,
            MaterialType.YELLOW: MachineType.FIRST_DRAW,
            MaterialType.RED: MachineType.SECOND_DRAW
        }
        return material.type in material_machine_map and machine.type == material_machine_map[material.type]

    def get_processing_time(self, machine: Machine) -> int:
        base_time = {
            MachineType.CARDING: 5,
            MachineType.FIRST_DRAW: 3,
            MachineType.SECOND_DRAW: 4
        }
        return base_time[machine.type]

    def transform_material(self, material: Material, machine: Machine) -> None:
        material_transform = {
            MachineType.CARDING: MaterialType.YELLOW,
            MachineType.FIRST_DRAW: MaterialType.RED,
            MachineType.SECOND_DRAW: MaterialType.EMPTY
        }
        material.type = material_transform[machine.type]

    def step(self) -> None:
        self.time += 1

        # 处理正在进行的加工
        for machine in self.machines.values():
            if machine.state == ProductionState.PROCESSING:
                machine.processing_time -= 1
                if machine.processing_time <= 0:
                    if len(machine.output_buffer) < machine.buffer_capacity:
                        self.transform_material(machine.current_material, machine)
                        machine.output_buffer.append(machine.current_material)
                        machine.current_material = None
                        machine.state = ProductionState.IDLE
                    else:
                        machine.state = ProductionState.BLOCKED

        # 开始新的加工
        for machine in self.machines.values():
            if machine.state == ProductionState.IDLE:
                if machine.input_buffer:
                    material = machine.input_buffer[0]
                    if self.can_process(machine, material):
                        material = machine.input_buffer.pop(0)
                        machine.current_material = material
                        machine.processing_time = self.get_processing_time(machine)
                        machine.state = ProductionState.PROCESSING
                    else:
                        machine.state = ProductionState.BLOCKED
                else:
                    machine.state = ProductionState.STARVED

    def transfer_material(self, from_machine_id: str, to_machine_id: str) -> bool:
        if from_machine_id not in self.machines or to_machine_id not in self.machines:
            return False

        from_machine = self.machines[from_machine_id]
        to_machine = self.machines[to_machine_id]

        if not from_machine.output_buffer or len(to_machine.input_buffer) >= to_machine.buffer_capacity:
            return False

        material = from_machine.output_buffer.pop(0)
        to_machine.input_buffer.append(material)
        material.location = to_machine_id
        return True


@dataclass
class PerformanceMetrics:
    """性能指标数据类"""
    completion_rate: float = 0.0
    utilization_rate: Dict[str, float] = field(default_factory=dict)
    deadlock_count: int = 0
    throughput: float = 0.0
    machine_states: Dict[str, Dict[ProductionState, int]] = field(
        default_factory=lambda: defaultdict(lambda: defaultdict(int)))
    processing_times: Dict[str, List[int]] = field(default_factory=lambda: defaultdict(list))
    blocked_times: Dict[str, List[int]] = field(default_factory=lambda: defaultdict(list))
    starved_times: Dict[str, List[int]] = field(default_factory=lambda: defaultdict(list))


class ProductionMonitor:
    """生产监控系统"""

    def __init__(self, production_system: ProductionSystem):
        self.system = production_system
        self.metrics = PerformanceMetrics()
        self.start_time = 0
        self.current_time = 0
        self.machine_state_durations = defaultdict(lambda: defaultdict(int))
        self.deadlock_detection_window = 10
        self.state_history = defaultdict(list)
        self.previous_states = defaultdict(lambda: ProductionState.IDLE)
        self.state_start_times = defaultdict(int)

    def start_monitoring(self):
        """开始监控"""
        self.start_time = self.system.time
        for machine_id in self.system.machines:
            self.state_start_times[machine_id] = self.start_time
            self.previous_states[machine_id] = self.system.machines[machine_id].state

    def update_metrics(self) -> None:
        """更新性能指标"""
        self.current_time = self.system.time
        elapsed_time = max(1, self.current_time - self.start_time)

        # 更新完成率
        total_materials = len(self.system.materials)
        completed_materials = sum(1 for m in self.system.materials.values()
                                  if m.type == MaterialType.EMPTY)
        self.metrics.completion_rate = completed_materials / total_materials if total_materials > 0 else 0

        # 更新各机器状态时间和利用率
        for machine_id, machine in self.system.machines.items():
            current_state = machine.state
            previous_state = self.previous_states[machine_id]
            state_duration = self.current_time - self.state_start_times[machine_id]

            # 记录状态持续时间
            self.machine_state_durations[machine_id][previous_state] += state_duration

            # 如果状态发生变化，更新相关计数器
            if current_state != previous_state:
                if previous_state == ProductionState.PROCESSING:
                    self.metrics.processing_times[machine_id].append(state_duration)
                elif previous_state == ProductionState.BLOCKED:
                    self.metrics.blocked_times[machine_id].append(state_duration)
                elif previous_state == ProductionState.STARVED:
                    self.metrics.starved_times[machine_id].append(state_duration)

                self.state_start_times[machine_id] = self.current_time
                self.previous_states[machine_id] = current_state

            # 计算利用率
            total_time = sum(self.machine_state_durations[machine_id].values())
            if total_time > 0:
                processing_time = self.machine_state_durations[machine_id][ProductionState.PROCESSING]
                self.metrics.utilization_rate[machine_id] = processing_time / total_time

        # 更新吞吐量
        self.metrics.throughput = completed_materials / elapsed_time if elapsed_time > 0 else 0

    def detect_deadlock(self) -> bool:
        """检测系统死锁"""
        all_machines_blocked_or_starved = True
        for machine in self.system.machines.values():
            if machine.state not in [ProductionState.BLOCKED, ProductionState.STARVED]:
                all_machines_blocked_or_starved = False
                break

        if all_machines_blocked_or_starved:
            self.metrics.deadlock_count += 1
            return True
        return False

    def record_state(self) -> None:
        """记录当前状态"""
        for machine_id, machine in self.system.machines.items():
            self.state_history[machine_id].append(machine.state)
            self.metrics.machine_states[machine_id][machine.state] += 1


class TestSystemPerformance(unittest.TestCase):
    def setUp(self):
        """测试初始化"""
        self.system = ProductionSystem()
        self.monitor = ProductionMonitor(self.system)

        # 初始化机器
        machines = [
            ("Carding1", MachineType.CARDING),
            ("FirstDraw1", MachineType.FIRST_DRAW),
            ("SecondDraw1", MachineType.SECOND_DRAW)
        ]
        for machine_id, machine_type in machines:
            self.system.add_machine(Machine(machine_id, machine_type))

        # 初始化材料
        for i in range(10):
            material = Material(f"Material{i}", MaterialType.GREEN, "Carding1")
            self.system.add_material(material)
            if i == 0:  # 将第一个材料放入第一台机器的输入缓冲区
                self.system.machines["Carding1"].input_buffer.append(material)

        self.monitor.start_monitoring()

    def run_simulation(self, steps: int) -> None:
        """运行模拟"""
        for _ in range(steps):
            # 处理物料转移
            if self.system.machines["FirstDraw1"].output_buffer:
                self.system.transfer_material("FirstDraw1", "SecondDraw1")
            if self.system.machines["Carding1"].output_buffer:
                self.system.transfer_material("Carding1", "FirstDraw1")

            # 运行一步并更新监控数据
            self.system.step()
            self.monitor.record_state()
            self.monitor.update_metrics()

    def test_completion_rate(self):
        """测试完成率统计"""
        self.run_simulation(100)
        self.assertGreaterEqual(self.monitor.metrics.completion_rate, 0)
        self.assertLessEqual(self.monitor.metrics.completion_rate, 1)
        print(f"\nCompletion rate: {self.monitor.metrics.completion_rate:.2%}")

    def test_utilization_rate(self):
        """测试资源利用率"""
        self.run_simulation(50)
        for machine_id in self.system.machines:
            utilization = self.monitor.metrics.utilization_rate.get(machine_id, 0)
            self.assertGreaterEqual(utilization, 0)
            self.assertLessEqual(utilization, 1)
            print(f"\nMachine {machine_id} utilization: {utilization:.2%}")

    def test_deadlock_detection(self):
        """测试死锁检测"""
        self.run_simulation(100)
        deadlock_rate = self.monitor.metrics.deadlock_count / 100
        print(f"\nDeadlock count: {self.monitor.metrics.deadlock_count}")
        print(f"Deadlock rate: {deadlock_rate:.2%}")

    def test_throughput(self):
        """测试系统吞吐量"""
        self.run_simulation(100)
        self.assertGreaterEqual(self.monitor.metrics.throughput, 0)
        print(f"\nSystem throughput: {self.monitor.metrics.throughput:.2f} items/step")

    def test_performance_statistics(self):
        """测试性能统计"""
        self.run_simulation(100)

        for machine_id in self.system.machines:
            print(f"\nStatistics for {machine_id}:")

            # 处理时间统计
            proc_times = self.monitor.metrics.processing_times[machine_id]
            if proc_times:
                avg_proc = statistics.mean(proc_times)
                print(f"Average processing time: {avg_proc:.2f}")
                if len(proc_times) > 1:
                    std_proc = statistics.stdev(proc_times)
                    print(f"Processing time std dev: {std_proc:.2f}")

            # 阻塞时间统计
            block_times = self.monitor.metrics.blocked_times[machine_id]
            if block_times:
                avg_block = statistics.mean(block_times)
                print(f"Average blocked time: {avg_block:.2f}")

            # 待料时间统计
            starve_times = self.monitor.metrics.starved_times[machine_id]
            if starve_times:
                avg_starve = statistics.mean(starve_times)
                print(f"Average starved time: {avg_starve:.2f}")


if __name__ == '__main__':
    unittest.main(verbosity=2)