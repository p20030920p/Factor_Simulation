import numpy as np
import matplotlib.pyplot as plt


class GridFactoryLayout:
    def __init__(self):
        self.MAP_SIZE = 1000
        self.GRID_SIZE = 1

        # 设备参数和区域值定义
        self.AREA_TYPES = {
            'boundary': 1,
            'material_warehouse': 2,
            'product_warehouse': 3,
            'charging': 4,
            'combing': 5,
            'drawing1': 6,
            'drawing2': 7,
            'roving': 8,
            'robot': 9,
            'combing_pickup': 15,  # 梳棉机取货区
            'drawing1_pickup': 16,  # 一并条取货区
            'drawing2_pickup': 17,  # 二并条取货区
            'roving_pickup': 18  # 粗纱机取货区
        }

        # 机器数量
        self.machine_counts = {
            'combing': 10,
            'drawing1': 5,
            'drawing2': 5,
            'roving': 5
        }

        # 机器尺寸（格子数）
        self.machine_size = {
            'combing': (40, 40),
            'drawing1': (40, 40),
            'drawing2': (40, 40),
            'roving': (40, 40)
        }

        # 机器人参数
        self.num_robots = 10
        self.robot_size = 10  # 机器人占用的格子大小

        # 工作区域和取放区域的位置记录
        self.work_areas = {
            'combing': [],
            'drawing1': [],
            'drawing2': [],
            'roving': []
        }
        self.pickup_areas = {
            'combing': [],
            'drawing1': [],
            'drawing2': [],
            'roving': []
        }

        # 初始化栅格地图
        self.grid = np.zeros((self.MAP_SIZE, self.MAP_SIZE))

        # 设置机器位置
        self.column_positions = {
            'combing': 150,
            'drawing1': 350,
            'drawing2': 550,
            'roving': 750
        }

        # 初始化地图和机器人位置
        self._initialize_grid()
        self.robot_positions = self._initialize_robot_positions()

    def _initialize_grid(self):
        """初始化栅格地图"""
        # 设置边界
        self.grid[0:10, :] = self.AREA_TYPES['boundary']
        self.grid[-10:, :] = self.AREA_TYPES['boundary']
        self.grid[:, 0:10] = self.AREA_TYPES['boundary']
        self.grid[:, -10:] = self.AREA_TYPES['boundary']

        self._place_warehouse()
        self._place_charging_stations()
        self._place_machines_and_areas()
        self._setup_paths()

    def _place_warehouse(self):
        """放置原料仓和成品仓"""
        # 原料仓
        self.grid[480:520, 30:70] = self.AREA_TYPES['material_warehouse']
        self.material_area = [(30, 480), (70, 520)]

        # 成品仓
        self.grid[480:520, 930:970] = self.AREA_TYPES['product_warehouse']
        self.product_area = [(930, 480), (970, 520)]

    def _place_charging_stations(self):
        """放置充电桩"""
        charging_positions = [
            (50, 50), (50, 950),
            (950, 50), (950, 950)
        ]

        self.charging_areas = []
        for x, y in charging_positions:
            self.grid[y - 20:y + 20, x - 20:x + 20] = self.AREA_TYPES['charging']
            self.charging_areas.append([(x - 20, y - 20), (x + 20, y + 20)])

    def _place_machines_and_areas(self):
        """放置所有机器和相关工作区域"""
        interval = 80

        for machine_type in ['combing', 'drawing1', 'drawing2', 'roving']:
            num_machines = self.machine_counts[machine_type]
            x_pos = self.column_positions[machine_type]
            pickup_type = f"{machine_type}_pickup"

            start_y = (self.MAP_SIZE - (num_machines - 1) * interval) // 2

            for i in range(num_machines):
                y = start_y + i * interval

                # 放置机器
                self.grid[y - 20:y + 20, x_pos - 20:x_pos + 20] = self.AREA_TYPES[machine_type]
                self.work_areas[machine_type].append([(x_pos - 20, y - 20), (x_pos + 20, y + 20)])

                # 放置取放区域（在机器前方）
                pickup_x = x_pos - 40
                self.grid[y - 15:y + 15, pickup_x - 15:pickup_x + 15] = self.AREA_TYPES[pickup_type]
                self.pickup_areas[machine_type].append([(pickup_x - 15, y - 15), (pickup_x + 15, y + 15)])

    def _setup_paths(self):
        """设置主要运输通道"""
        # 横向通道
        for y in [300, 500, 700]:
            self.grid[y - 10:y + 10, :] = 0

        # 纵向通道
        for x in [110, 310, 510, 710]:
            self.grid[:, x - 10:x + 10] = 0

    def _initialize_robot_positions(self):
        """初始化机器人位置"""
        robot_positions = []

        # 在右下角区域布置机器人（靠近空桶区）
        start_x = 880  # 起始x坐标
        start_y = 880  # 起始y坐标
        robots_per_row = 5  # 每行放置的机器人数量

        for i in range(self.num_robots):
            row = i // robots_per_row
            col = i % robots_per_row
            x = start_x + col * (self.robot_size + 5)  # 5是机器人之间的间隔
            y = start_y + row * (self.robot_size + 5)
            robot_positions.append((x, y))

            # 在栅格地图上标记机器人位置
            self.grid[y:y + self.robot_size, x:x + self.robot_size] = self.AREA_TYPES['robot']

        return robot_positions

    def is_robot_position(self, pos):
        """检查给定位置是否是机器人位置"""
        x, y = pos
        return any(abs(rx - x) < self.robot_size and abs(ry - y) < self.robot_size
                   for rx, ry in self.robot_positions)

    def get_robot_positions(self):
        """获取所有机器人的当前位置"""
        return self.robot_positions

    def update_robot_position(self, robot_id, new_pos):
        """更新机器人位置"""
        if 0 <= robot_id < self.num_robots:
            old_x, old_y = self.robot_positions[robot_id]
            new_x, new_y = new_pos

            # 清除旧位置
            self.grid[old_y:old_y + self.robot_size,
            old_x:old_x + self.robot_size] = 0

            # 设置新位置
            self.grid[new_y:new_y + self.robot_size,
            new_x:new_x + self.robot_size] = self.AREA_TYPES['robot']

            self.robot_positions[robot_id] = new_pos
            return True
        return False

    def check_layout(self):
        """检查布局配置"""
        print("\n=== 工厂布局检查 ===")

        # 检查机器数量
        print("\n机器数量检查:")
        for machine_type, count in self.machine_counts.items():
            actual_count = len(self.work_areas[machine_type])
            print(f"{machine_type}: 预期{count}台, 实际{actual_count}台")

        # 检查工作区域
        print("\n工作区域位置:")
        for machine_type in self.work_areas:
            print(f"\n{machine_type}工作区域:")
            for i, area in enumerate(self.work_areas[machine_type], 1):
                print(f"机器{i}: {area}")

        # 检查取放区域
        print("\n取放区域位置:")
        for machine_type in self.pickup_areas:
            print(f"\n{machine_type}取放区域:")
            for i, area in enumerate(self.pickup_areas[machine_type], 1):
                print(f"区域{i}: {area}")

        # 检查仓库区域
        print("\n仓库区域:")
        print(f"原料仓: {self.material_area}")
        print(f"成品仓: {self.product_area}")

        # 检查充电区域
        print("\n充电区域:")
        for i, area in enumerate(self.charging_areas, 1):
            print(f"充电桩{i}: {area}")

        # 检查机器人位置
        print("\n机器人初始位置:")
        for i, pos in enumerate(self.robot_positions, 1):
            print(f"机器人 {i}: 位置 {pos}")

    def is_in_area(self, pos, area_type, index=None):
        """检查位置是否在指定区域内"""
        x, y = pos

        if area_type in self.work_areas:
            areas = self.work_areas[area_type]
        elif area_type in self.pickup_areas:
            areas = self.pickup_areas[area_type]
        else:
            return False

        if index is not None:
            if 0 <= index < len(areas):
                area = areas[index]
                return (area[0][0] <= x <= area[1][0] and
                        area[0][1] <= y <= area[1][1])
            return False

        # 检查所有相同类型的区域
        for area in areas:
            if (area[0][0] <= x <= area[1][0] and
                    area[0][1] <= y <= area[1][1]):
                return True
        return False

    def visualize(self):
        """可视化栅格地图"""
        plt.figure(figsize=(15, 15))
        plt.imshow(self.grid, cmap='tab20')
        plt.colorbar(label='Area Type')
        plt.title('Factory Grid Map')

        # 添加图例说明
        legend_text = """
        Grid Values:
        0: Free space
        1: Boundary
        2: Material Warehouse
        3: Product Warehouse
        4: Charging Station
        5: Combing Machine
        6: Drawing Machine 1
        7: Drawing Machine 2
        8: Roving Machine
        9: Robot
        15: Combing Pickup Area
        16: Drawing1 Pickup Area
        17: Drawing2 Pickup Area
        18: Roving Pickup Area
        """
        plt.text(1050, 500, legend_text, fontsize=10)
        plt.show()

    def get_grid(self):
        """返回栅格地图"""
        return self.grid


# 创建布局并测试
factory = GridFactoryLayout()
factory.check_layout()
factory.visualize()

# 测试机器人位置更新
print("\n测试机器人位置更新:")
print("初始位置:", factory.get_robot_positions()[0])
new_pos = (890, 890)
factory.update_robot_position(0, new_pos)
print("更新后位置:", factory.get_robot_positions()[0])

# 测试位置检查
test_pos = (130, 500)
print(f"\n位置检查测试 {test_pos}:")
print(f"是否在梳棉机工作区: {factory.is_in_area(test_pos, 'combing')}")
print(f"是否在梳棉机1号工作区: {factory.is_in_area(test_pos, 'combing', 0)}")
print(f"是否是机器人位置: {factory.is_robot_position(test_pos)}")