import numpy as np
import heapq
import time
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set
from enum import Enum
import matplotlib.pyplot as plt
from collections import defaultdict


# ====================== 基础数据类型 ======================
class AreaType(Enum):
    EMPTY_BUCKET = "空桶区"
    CARDING_IN = "梳棉待装填"
    CARDING_OUT = "梳棉已完成"
    DRAWING1_IN = "一并待装填"
    DRAWING1_OUT = "一并已完成"
    DRAWING2_IN = "二并待装填"
    DRAWING2_OUT = "二并已完成"
    ROVING = "粗纱原料区"
    CHARGING = "充电区"
    OBSTACLE = "障碍区"


class MaterialType(Enum):
    EMPTY = "空桶"
    GREEN = "绿桶"
    YELLOW = "黄桶"
    RED = "红桶"


class MachineType(Enum):
    CARDING = "梳棉机"
    DRAWING1 = "一并机"
    DRAWING2 = "二并机"
    ROVING = "粗纱机"


@dataclass
class Position:
    x: int
    y: int

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def distance(self, other: 'Position') -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5


# ====================== 工厂布局系统 ======================
class FactoryLayout:
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.areas = self._define_areas()
        self.machines = self._place_machines()
        self.obstacles = self._generate_obstacles()

    def _define_areas(self) -> Dict[AreaType, Tuple[Position, Position]]:
        """定义工厂各区域坐标范围"""
        return {
            AreaType.EMPTY_BUCKET: (Position(50, 50), Position(150, 150)),
            AreaType.CARDING_IN: (Position(200, 50), Position(300, 150)),
            AreaType.CARDING_OUT: (Position(350, 50), Position(450, 150)),
            AreaType.DRAWING1_IN: (Position(500, 50), Position(600, 150)),
            AreaType.DRAWING1_OUT: (Position(650, 50), Position(750, 150)),
            AreaType.DRAWING2_IN: (Position(800, 50), Position(900, 150)),
            AreaType.DRAWING2_OUT: (Position(50, 200), Position(150, 300)),
            AreaType.ROVING: (Position(200, 200), Position(300, 300)),
            AreaType.CHARGING: (Position(900, 900), Position(950, 950)),
            AreaType.OBSTACLE: (Position(400, 400), Position(600, 600))
        }

    def _place_machines(self) -> Dict[MachineType, List[Position]]:
        """放置各类型机器"""
        return {
            MachineType.CARDING: [Position(200 + i * 50, 100) for i in range(10)],
            MachineType.DRAWING1: [Position(500 + i * 60, 100) for i in range(5)],
            MachineType.DRAWING2: [Position(800 + i * 60, 100) for i in range(5)],
            MachineType.ROVING: [Position(250 + i * 50, 250) for i in range(5)]
        }

    def _generate_obstacles(self) -> Set[Position]:
        """生成障碍物集合"""
        obstacles = set()
        # 添加机器位置作为障碍
        for positions in self.machines.values():
            obstacles.update(positions)
        # 添加障碍区域
        for x in range(400, 601):
            for y in range(400, 601):
                obstacles.add(Position(x, y))
        return obstacles

    def is_valid_position(self, pos: Position) -> bool:
        """检查位置是否有效"""
        return (0 <= pos.x < self.width and
                0 <= pos.y < self.height and
                pos not in self.obstacles)

    def get_area_center(self, area_type: AreaType) -> Position:
        """获取区域中心坐标"""
        tl, br = self.areas[area_type]
        return Position((tl.x + br.x) // 2, (tl.y + br.y) // 2)


# ====================== 物料与生产批次 ======================
@dataclass
class Material:
    id: int
    type: MaterialType
    position: Position
    state: str = "waiting"  # waiting/processing/completed


@dataclass
class ProductionBatch:
    batch_id: int
    machine_type: MachineType
    input_materials: List[Material]
    output_type: MaterialType
    start_time: float = 0.0
    end_time: float = 0.0
    state: str = "pending"  # pending/processing/completed


# ====================== 生产链管理系统 ======================
class ProductionManager:
    def __init__(self):
        self.materials = defaultdict(list)
        self.batches = []
        self.conversion_rates = {
            MachineType.CARDING: (MaterialType.EMPTY, MaterialType.GREEN, 1),
            MachineType.DRAWING1: (MaterialType.GREEN, MaterialType.YELLOW, 6),
            MachineType.DRAWING2: (MaterialType.YELLOW, MaterialType.RED, 6)
        }
        self.processing_times = {
            MachineType.CARDING: 4225.94,
            MachineType.DRAWING1: 2594.75,
            MachineType.DRAWING2: 2594.75
        }

        # 初始化540个空桶
        for i in range(540):
            self.materials[MaterialType.EMPTY].append(
                Material(i, MaterialType.EMPTY, Position(0, 0))
            )

    def create_batch(self, machine_type: MachineType) -> Optional[ProductionBatch]:
        """创建新的生产批次"""
        input_type, output_type, ratio = self.conversion_rates[machine_type]

        if len(self.materials[input_type]) >= ratio:
            inputs = self.materials[input_type][:ratio]
            del self.materials[input_type][:ratio]

            batch = ProductionBatch(
                batch_id=len(self.batches) + 1,
                machine_type=machine_type,
                input_materials=inputs,
                output_type=output_type
            )
            self.batches.append(batch)
            return batch
        return None

    def complete_batch(self, batch: ProductionBatch) -> List[Material]:
        """完成批次生产"""
        new_materials = []
        for i in range(len(batch.input_materials) // self.conversion_rates[batch.machine_type][2]):
            new_id = len(self.materials[batch.output_type]) + 1
            new_materials.append(
                Material(new_id, batch.output_type, Position(0, 0))
            )
        self.materials[batch.output_type].extend(new_materials)
        batch.state = "completed"
        return new_materials


# ====================== 机器人实体 ======================
@dataclass
class Robot:
    robot_id: int
    position: Position
    battery: float = 100.0
    state: str = "idle"  # idle/moving/charging
    current_task: Optional[dict] = None
    path: List[Position] = field(default_factory=list)
    stats: dict = field(default_factory=lambda: {
        "total_distance": 0.0,
        "completed_tasks": 0,
        "working_time": 0.0
    })

    def update_position(self, new_pos: Position):
        """更新位置并计算移动距离"""
        if self.path:
            self.stats["total_distance"] += self.position.distance(new_pos)
            self.stats["working_time"] += 1  # 假设每步1秒
        self.position = new_pos

    def needs_charging(self) -> bool:
        """检查是否需要充电"""
        return self.battery < 20.0

# （第一部分结束，请发送[继续生成]获取第二部分）
# ====================== 任务管理系统 ======================
class Task:
    def __init__(self, task_id: int, task_type: str,
                 start_pos: Position, end_pos: Position,
                 priority: float = 1.0):
        self.task_id = task_id
        self.type = task_type  # transport/charging
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.priority = priority
        self.state = "pending"  # pending/assigned/completed
        self.assigned_robot = None


class TaskManager:
    def __init__(self, layout: FactoryLayout):
        self.layout = layout
        self.tasks = []
        self.task_id_counter = 0

    def generate_transport_tasks(self, production: ProductionManager) -> List[Task]:
        """生成物料运输任务"""
        tasks = []
        # 空桶 -> 梳棉机
        if len(production.materials[MaterialType.EMPTY]) > 0:
            start = self.layout.get_area_center(AreaType.EMPTY_BUCKET)
            end = self.layout.get_area_center(AreaType.CARDING_IN)
            tasks.append(self._create_task(start, end, "transport"))

        # 梳棉完成 -> 一并机
        if len(production.materials[MaterialType.GREEN]) > 0:
            start = self.layout.get_area_center(AreaType.CARDING_OUT)
            end = self.layout.get_area_center(AreaType.DRAWING1_IN)
            tasks.append(self._create_task(start, end, "transport"))

        # ...其他运输任务生成逻辑...
        return tasks

    def _create_task(self, start: Position, end: Position, task_type: str) -> Task:
        """创建新任务"""
        self.task_id_counter += 1
        return Task(
            task_id=self.task_id_counter,
            task_type=task_type,
            start_pos=start,
            end_pos=end
        )


# ====================== 路径规划节点 ======================
@dataclass
class PathNode:
    position: Position
    g_cost: float = float('inf')
    h_cost: float = 0.0
    parent: Optional['PathNode'] = None
    time_step: int = 0

    @property
    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost


# ====================== A*路径规划器 ======================
class AStarPlanner:
    def __init__(self, layout: FactoryLayout):
        self.layout = layout
        self.directions = [
            Position(0, 1), Position(1, 0), Position(0, -1), Position(-1, 0),
            Position(1, 1), Position(1, -1), Position(-1, 1), Position(-1, -1)
        ]

    def plan_path(self, start: Position, goal: Position) -> List[Position]:
        """A*路径规划"""
        open_set = []
        start_node = PathNode(start, 0, self._heuristic(start, goal))
        heapq.heappush(open_set, start_node)

        came_from = {}
        g_score = {start: 0}

        while open_set:
            current = heapq.heappop(open_set)

            if current.position == goal:
                return self._reconstruct_path(current)

            for dx, dy in self.directions:
                neighbor_pos = Position(
                    current.position.x + dx,
                    current.position.y + dy
                )

                if not self.layout.is_valid_position(neighbor_pos):
                    continue

                tentative_g = g_score[current.position] + self._get_move_cost(current.position, neighbor_pos)

                if tentative_g < g_score.get(neighbor_pos, float('inf')):
                    came_from[neighbor_pos] = current
                    g_score[neighbor_pos] = tentative_g
                    neighbor_node = PathNode(
                        position=neighbor_pos,
                        g_cost=tentative_g,
                        h_cost=self._heuristic(neighbor_pos, goal),
                        parent=current
                    )
                    heapq.heappush(open_set, neighbor_node)

        return []  # 未找到路径

    def _heuristic(self, pos: Position, goal: Position) -> float:
        """曼哈顿距离启发函数"""
        return abs(pos.x - goal.x) + abs(pos.y - goal.y)

    def _get_move_cost(self, from_pos: Position, to_pos: Position) -> float:
        """计算移动代价（简单版本）"""
        return 1.0  # 可扩展为考虑地形代价

    def _reconstruct_path(self, end_node: PathNode) -> List[Position]:
        """重建路径"""
        path = []
        current = end_node
        while current:
            path.append(current.position)
            current = current.parent
        return path[::-1]


# ====================== CBS冲突解决 ======================
class CBSConstraint:
    def __init__(self, robot_id: int, position: Position, time_step: int):
        self.robot_id = robot_id
        self.position = position
        self.time_step = time_step


class CBSNode:
    def __init__(self, constraints: List[CBSConstraint], solution: Dict[int, List[Position]]):
        self.constraints = constraints
        self.solution = solution
        self.cost = sum(len(path) for path in solution.values())


class CBSSolver:
    def __init__(self, layout: FactoryLayout):
        self.layout = layout
        self.astar = AStarPlanner(layout)

    def plan_paths(self, robot_tasks: Dict[int, Tuple[Position, Position]]) -> Dict[int, List[Position]]:
        """基于CBS的多机器人路径规划"""
        # 初始解
        solution = {}
        for rid, (start, goal) in robot_tasks.items():
            solution[rid] = self.astar.plan_path(start, goal)

        # 检测冲突
        conflicts = self._find_conflicts(solution)
        if not conflicts:
            return solution

        # 处理冲突（简化版本）
        for conflict in conflicts:
            # 为第二个机器人添加等待
            if conflict.robot2 in solution:
                solution[conflict.robot2] = [solution[conflict.robot2][0]] + solution[conflict.robot2]

        return solution

    def _find_conflicts(self, solution: Dict[int, List[Position]]) -> List[dict]:
        """检测路径中的冲突"""
        conflicts = []
        max_len = max(len(p) for p in solution.values())

        for t in range(max_len):
            positions = defaultdict(list)
            for rid, path in solution.items():
                pos = path[min(t, len(path) - 1)]
                positions[pos].append(rid)

            for pos, robots in positions.items():
                if len(robots) > 1:
                    conflicts.append({
                        "time": t,
                        "position": pos,
                        "robots": robots
                    })

        return conflicts

# （第二部分结束，请发送[继续生成]获取第三部分）
# ====================== 仿真主系统 ======================
class FactorySimulation:
    def __init__(self):
        self.layout = FactoryLayout()
        self.production = ProductionManager()
        self.robots = {i: Robot(i, self.layout.get_area_center(AreaType.CHARGING))
                       for i in range(10)}
        self.task_manager = TaskManager(self.layout)
        self.cbs_solver = CBSSolver(self.layout)
        self.time_step = 0
        self.metrics = {
            "production": defaultdict(float),
            "robots": defaultdict(dict),
            "efficiency": 0.0
        }

    def run(self, max_steps=100000):
        """运行仿真主循环"""
        start_time = time.time()

        while self.time_step < max_steps and not self._production_completed():
            # 1. 更新生产批次
            self._update_production()

            # 2. 生成运输任务
            tasks = self.task_manager.generate_transport_tasks(self.production)

            # 3. 分配任务给机器人
            self._assign_tasks_to_robots(tasks)

            # 4. 规划路径
            robot_goals = {
                rid: (robot.position, robot.current_task.end_pos)
                for rid, robot in self.robots.items()
                if robot.current_task
            }
            paths = self.cbs_solver.plan_paths(robot_goals)

            # 5. 更新机器人路径
            for rid, path in paths.items():
                if path:
                    self.robots[rid].path = path

            # 6. 更新机器人状态
            self._update_robots()

            # 7. 收集指标
            self._collect_metrics()

            self.time_step += 1

        # 计算最终效率
        self._calculate_efficiency(start_time)

    def _update_production(self):
        """更新生产系统状态"""
        # 梳棉机处理
        if self.time_step % int(self.production.processing_times[MachineType.CARDING]) == 0:
            batch = self.production.create_batch(MachineType.CARDING)
            if batch:
                self.production.complete_batch(batch)

        # 一并机处理
        if self.time_step % int(self.production.processing_times[MachineType.DRAWING1]) == 0:
            batch = self.production.create_batch(MachineType.DRAWING1)
            if batch:
                self.production.complete_batch(batch)

        # 二并机处理
        if self.time_step % int(self.production.processing_times[MachineType.DRAWING2]) == 0:
            batch = self.production.create_batch(MachineType.DRAWING2)
            if batch:
                self.production.complete_batch(batch)

    def _assign_tasks_to_robots(self, tasks: List[Task]):
        """分配任务给空闲机器人"""
        for task in tasks:
            if task.state == "pending":
                # 找到空闲机器人
                for rid, robot in self.robots.items():
                    if robot.state == "idle":
                        robot.current_task = task
                        task.state = "assigned"
                        robot.state = "moving"
                        break

    def _update_robots(self):
        """更新所有机器人状态"""
        for robot in self.robots.values():
            if robot.path:
                next_pos = robot.path.pop(0)
                robot.update_position(next_pos)

                if not robot.path and robot.current_task:
                    robot.current_task.state = "completed"
                    robot.stats["completed_tasks"] += 1
                    robot.state = "idle"
                    robot.current_task = None

            # 充电逻辑
            if robot.needs_charging():
                self._send_to_charging(robot)

    def _send_to_charging(self, robot: Robot):
        """发送机器人去充电"""
        if robot.state != "charging":
            task = Task(
                task_id=len(self.task_manager.tasks) + 1,
                task_type="charging",
                start_pos=robot.position,
                end_pos=self.layout.get_area_center(AreaType.CHARGING)
            )
            robot.current_task = task
            robot.path = self.cbs_solver.astar.plan_path(robot.position, task.end_pos)
            robot.state = "charging"

    def _collect_metrics(self):
        """收集性能指标"""
        # 生产指标
        self.metrics["production"]["empty"] = len(self.production.materials[MaterialType.EMPTY])
        self.metrics["production"]["green"] = len(self.production.materials[MaterialType.GREEN])
        self.metrics["production"]["yellow"] = len(self.production.materials[MaterialType.YELLOW])
        self.metrics["production"]["red"] = len(self.production.materials[MaterialType.RED])

        # 机器人指标
        for rid, robot in self.robots.items():
            self.metrics["robots"][rid] = {
                "distance": robot.stats["total_distance"],
                "tasks": robot.stats["completed_tasks"],
                "utilization": robot.stats["working_time"] / (self.time_step + 1e-9)
            }

    def _calculate_efficiency(self, start_time: float):
        """计算整体效率"""
        actual_time = time.time() - start_time
        theoretical_time = (
                540 * 4225.94 / 10 +  # 梳棉
                90 * 2594.75 / 5 +  # 一并
                15 * 2594.75 / 5  # 二并
        )
        self.metrics["efficiency"] = (theoretical_time / actual_time) * 100

    def _production_completed(self) -> bool:
        """检查生产是否完成"""
        return len(self.production.materials[MaterialType.RED]) >= 15


# ====================== 可视化系统 ======================
class SimulationVisualizer:
    def __init__(self, simulation: FactorySimulation):
        self.sim = simulation
        plt.figure(figsize=(15, 15))
        self.ax = plt.gca()

    def plot_layout(self):
        """绘制工厂布局"""
        # 绘制区域
        for area_type, (tl, br) in self.sim.layout.areas.items():
            self.ax.add_patch(plt.Rectangle(
                (tl.x, tl.y), br.x - tl.x, br.y - tl.y,
                alpha=0.2, label=area_type.value
            ))

        # 绘制机器
        colors = {"CARDING": "blue", "DRAWING1": "green", "DRAWING2": "red"}
        for machine_type, positions in self.sim.layout.machines.items():
            x = [p.x for p in positions]
            y = [p.y for p in positions]
            self.ax.scatter(x, y, c=colors[machine_type.name],
                            label=machine_type.value, marker="s")

        # 绘制机器人
        for rid, robot in self.sim.robots.items():
            color = "red" if robot.state == "moving" else "green"
            self.ax.plot(robot.position.x, robot.position.y,
                         marker="o", markersize=10, color=color)
            if robot.path:
                path_x = [p.x for p in robot.path]
                path_y = [p.y for p in robot.path]
                self.ax.plot(path_x, path_y, linestyle="--", color=color)

        self.ax.legend()
        plt.title(f"Factory Simulation - Step {self.sim.time_step}")
        plt.show()

    def plot_metrics(self):
        """绘制性能指标"""
        plt.figure(figsize=(12, 6))

        # 生产进度
        plt.subplot(2, 2, 1)
        plt.plot([m["red"] for m in self.sim.metrics["production"]])
        plt.title("Red Buckets Production")
        plt.xlabel("Time Steps")
        plt.ylabel("Count")

        # 机器人利用率
        plt.subplot(2, 2, 2)
        utilizations = [np.mean([r["utilization"] for r in self.sim.metrics["robots"].values()])]
        plt.plot(utilizations)
        plt.title("Average Robot Utilization")
        plt.ylim(0, 1)

        # 效率指标
        plt.subplot(2, 2, 3)
        plt.bar(["Theoretical", "Actual"],
                [self.sim.metrics["efficiency"],
                 [self.sim.metrics["actual_time"]])
        plt.title("Production Efficiency")

        plt.tight_layout()
        plt.show()

        # ====================== 主程序 ======================
        if __name__ == "__main__":
            print("Starting factory simulation...")
        sim = FactorySimulation()
        visualizer = SimulationVisualizer(sim)

    try:
        sim.run(max_steps=10000)
        print("\n=== Simulation Results ===")
        print(f"Total Time Steps: {sim.time_step}")
        print(f"Red Buckets Produced: {sim.production.materials[MaterialType.RED]}")
        print(f"System Efficiency: {sim.metrics['efficiency']:.2f}%")

        print("\nRobot Performance:")
        for rid, metrics in sim.metrics["robots"].items():
            print(f"Robot {rid}: {metrics['tasks']} tasks, {metrics['distance']:.2f} units")

        visualizer.plot_layout()
        visualizer.plot_metrics()

    except Exception as e:
        print(f"Simulation Error: {str(e)}")
        import traceback
        traceback.print_exc()