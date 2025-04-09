import unittest
from dataclasses import dataclass, field
from typing import Dict, List, Optional
from enum import Enum


class MaterialType(Enum):
    GREEN = "green"  # 绿桶
    YELLOW = "yellow"  # 黄桶
    RED = "red"  # 红桶
    EMPTY = "empty"  # 空桶


class MachineType(Enum):
    CARDING = "carding"  # 梳棉机
    FIRST_DRAW = "first"  # 一并机
    SECOND_DRAW = "second"  # 二并机


class ProductionState(Enum):
    IDLE = "idle"  # 空闲
    PROCESSING = "processing"  # 加工中
    BLOCKED = "blocked"  # 被阻塞
    STARVED = "starved"  # 原料不足


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

    def update_state(self) -> None:
        """更新机器状态"""
        if self.state != ProductionState.PROCESSING:
            if not self.input_buffer:
                self.state = ProductionState.STARVED
            elif len(self.output_buffer) >= self.buffer_capacity:
                self.state = ProductionState.BLOCKED
            else:
                self.state = ProductionState.IDLE


class ProductionSystem:
    def __init__(self):
        self.machines: Dict[str, Machine] = {}
        self.materials: Dict[str, Material] = {}
        self.time = 0
        self.production_history: List[Dict] = []
        self.debug = False

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

    def log_debug(self, message: str) -> None:
        if self.debug:
            print(message)

    def step(self) -> None:
        self.time += 1
        state_snapshot = {}

        # 处理所有机器
        for machine in self.machines.values():
            # 处理当前加工中的材料
            if machine.state == ProductionState.PROCESSING:
                machine.processing_time -= 1
                self.log_debug(f"{machine.id} processing, time left: {machine.processing_time}")

                if machine.processing_time <= 0:
                    if len(machine.output_buffer) < machine.buffer_capacity:
                        self.transform_material(machine.current_material, machine)
                        machine.output_buffer.append(machine.current_material)
                        machine.current_material = None
                        machine.state = ProductionState.IDLE
                        self.log_debug(f"{machine.id} finished processing")
                    else:
                        machine.state = ProductionState.BLOCKED
                        self.log_debug(f"{machine.id} blocked")

            # 开始新的加工
            elif machine.state == ProductionState.IDLE:
                if machine.input_buffer:
                    material = machine.input_buffer[0]
                    if self.can_process(machine, material):
                        material = machine.input_buffer.pop(0)
                        machine.current_material = material
                        machine.processing_time = self.get_processing_time(machine)
                        machine.state = ProductionState.PROCESSING
                        self.log_debug(f"{machine.id} started processing {material.id}")

                machine.update_state()

            # 记录状态
            state_snapshot[machine.id] = {
                'state': machine.state,
                'input_buffer': len(machine.input_buffer),
                'output_buffer': len(machine.output_buffer),
                'current_material': machine.current_material.id if machine.current_material else None
            }

        self.production_history.append({
            'time': self.time,
            'states': state_snapshot
        })

    def transfer_material(self, from_machine_id: str, to_machine_id: str) -> bool:
        """在机器之间转移材料"""
        if from_machine_id not in self.machines or to_machine_id not in self.machines:
            return False

        from_machine = self.machines[from_machine_id]
        to_machine = self.machines[to_machine_id]

        if not from_machine.output_buffer or len(to_machine.input_buffer) >= to_machine.buffer_capacity:
            return False

        material = from_machine.output_buffer.pop(0)
        to_machine.input_buffer.append(material)
        material.location = to_machine_id

        # 更新机器状态
        from_machine.update_state()
        to_machine.update_state()

        self.log_debug(f"Transferred {material.id} from {from_machine_id} to {to_machine_id}")
        return True


class TestProductionSystem(unittest.TestCase):
    def setUp(self):
        self.system = ProductionSystem()
        self.system.debug = True  # 启用调试输出

        # 创建机器
        self.system.add_machine(Machine("Carding1", MachineType.CARDING))
        self.system.add_machine(Machine("FirstDraw1", MachineType.FIRST_DRAW))
        self.system.add_machine(Machine("SecondDraw1", MachineType.SECOND_DRAW))

        # 创建材料
        for i in range(5):
            material = Material(f"Material{i}", MaterialType.GREEN, location="Carding1")
            self.system.add_material(material)

    def test_production_line(self):
        """测试完整生产线"""
        material = self.system.materials["Material1"]
        material.type = MaterialType.GREEN

        carding = self.system.machines["Carding1"]
        first_draw = self.system.machines["FirstDraw1"]
        second_draw = self.system.machines["SecondDraw1"]

        carding.input_buffer.append(material)

        max_steps = 20
        step_count = 0

        def print_status():
            print(f"\nStep {step_count}:")
            for m in [carding, first_draw, second_draw]:
                mat_id = m.current_material.id if m.current_material else 'None'
                print(f"{m.id}: State={m.state.value}, "
                      f"Input={len(m.input_buffer)}, "
                      f"Output={len(m.output_buffer)}, "
                      f"Processing={mat_id}")

        while step_count < max_steps:
            print_status()

            # 处理材料转移
            if first_draw.output_buffer:
                self.system.transfer_material("FirstDraw1", "SecondDraw1")
            if carding.output_buffer:
                self.system.transfer_material("Carding1", "FirstDraw1")

            # 运行生产步骤
            self.system.step()

            # 检查完成条件
            if second_draw.output_buffer and any(m.type == MaterialType.EMPTY for m in second_draw.output_buffer):
                print("\nProduction completed successfully!")
                break

            step_count += 1

        print_status()

        # 验证结果
        self.assertTrue(step_count < max_steps,
                        f"生产未能在{max_steps}步内完成，当前步数：{step_count}")
        self.assertTrue(len(second_draw.output_buffer) > 0,
                        "二并机输出缓冲区为空")
        self.assertEqual(second_draw.output_buffer[0].type, MaterialType.EMPTY,
                         f"最终产品类型错误：{second_draw.output_buffer[0].type}")

    def test_material_transformation(self):
        """测试材料转换"""
        material = self.system.materials["Material0"]
        carding = self.system.machines["Carding1"]
        carding.input_buffer.append(material)

        while not carding.output_buffer:
            self.system.step()

        self.assertEqual(carding.output_buffer[0].type, MaterialType.YELLOW)

    def test_buffer_capacity(self):
        """测试缓冲区容量"""
        machine = self.system.machines["Carding1"]

        # 填充到缓冲区容量
        for i in range(machine.buffer_capacity):
            material = Material(f"ExtraMaterial{i}", MaterialType.GREEN)
            machine.input_buffer.append(material)

        self.assertEqual(len(machine.input_buffer), machine.buffer_capacity)

        # 测试超出容量
        extra_material = Material("Overflow", MaterialType.GREEN)
        initial_length = len(machine.input_buffer)
        machine.input_buffer.append(extra_material)

        self.assertEqual(len(machine.input_buffer), initial_length + 1)

    def test_machine_states(self):
        """测试机器状态转换"""
        machine = self.system.machines["Carding1"]

        # 测试初始状态
        self.assertEqual(machine.state, ProductionState.IDLE)

        # 测试加工状态
        material = Material("TestMaterial", MaterialType.GREEN)
        machine.input_buffer.append(material)
        self.system.step()
        self.assertEqual(machine.state, ProductionState.PROCESSING)

        # 等待加工完成
        while machine.state == ProductionState.PROCESSING:
            self.system.step()

        self.assertEqual(machine.state, ProductionState.IDLE)

        # 测试饥饿状态
        self.system.step()
        self.assertEqual(machine.state, ProductionState.STARVED)

    def test_production_history(self):
        """测试生产历史记录"""
        material = Material("HistoryTest", MaterialType.GREEN, location="Carding1")
        machine = self.system.machines["Carding1"]
        machine.input_buffer.append(material)

        steps = 10
        for _ in range(steps):
            self.system.step()

        # 验证历史记录
        self.assertEqual(len(self.system.production_history), steps)
        self.assertTrue(all('time' in record for record in self.system.production_history))
        self.assertTrue(all('states' in record for record in self.system.production_history))

        for record in self.system.production_history:
            for machine_id in self.system.machines:
                self.assertIn(machine_id, record['states'])
                machine_state = record['states'][machine_id]
                self.assertIn('state', machine_state)
                self.assertIn('input_buffer', machine_state)
                self.assertIn('output_buffer', machine_state)
                self.assertIn('current_material', machine_state)


if __name__ == '__main__':
    test = TestProductionSystem()
    test.setUp()
    test.test_production_line()