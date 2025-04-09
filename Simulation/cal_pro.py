import math


class ProductionLine:
    def __init__(self):
        # 目标参数
        self.target_red = 15  # 最终目标15个红桶

        # 设备参数
        self.combing_machines = 10  # 梳棉机数量
        self.first_drawing_machines = 5  # 一并条机数量
        self.second_drawing_machines = 5  # 二并条机数量
        self.roving_machines = 5  # 粗纱机数量

        # 转化比例
        self.green_to_yellow_ratio = 6  # 6个绿桶到1个黄桶
        self.yellow_to_red_ratio = 6  # 6个黄桶到1个红桶

        # 处理时间（秒）
        self.combing_time = 4225.94  # 梳棉机处理时间
        self.drawing_time = 2594.75  # 并条机处理时间

    def calculate_material_requirements(self):
        """计算各阶段所需物料数量"""
        red_barrels = self.target_red  # 15个红桶
        yellow_barrels = red_barrels * self.yellow_to_red_ratio  # 90个黄桶
        green_barrels = yellow_barrels * self.green_to_yellow_ratio  # 540个绿桶
        return green_barrels, yellow_barrels, red_barrels

    def calculate_ideal_time(self):
        """计算理想状态下的完成时间"""
        green_barrels, yellow_barrels, red_barrels = self.calculate_material_requirements()

        # 1. 梳棉阶段（生产绿桶）
        # 540个绿桶分配给10台梳棉机，每台54个
        combing_batches = math.ceil(green_barrels / self.combing_machines)  # 54批次/机
        combing_time = combing_batches * self.combing_time

        # 2. 一并条阶段（绿桶→黄桶）
        # 90个黄桶分配给5台一并条机，每台18个
        first_drawing_batches = math.ceil(yellow_barrels / self.first_drawing_machines)  # 18批次/机
        first_drawing_time = first_drawing_batches * self.drawing_time

        # 3. 二并条阶段（黄桶→红桶）
        # 15个红桶分配给5台二并条机，每台3个
        second_drawing_batches = math.ceil(red_barrels / self.second_drawing_machines)  # 3批次/机
        second_drawing_time = second_drawing_batches * self.drawing_time

        # 理想流水线时间：各阶段时间叠加
        total_time = combing_time + first_drawing_time + second_drawing_time

        return {
            'total_time': total_time,
            'combing_time': combing_time,
            'first_drawing_time': first_drawing_time,
            'second_drawing_time': second_drawing_time,
            'material_count': {
                'green': green_barrels,
                'yellow': yellow_barrels,
                'red': red_barrels
            },
            'batches': {
                'combing': combing_batches,
                'first_drawing': first_drawing_batches,
                'second_drawing': second_drawing_batches
            }
        }


# 创建实例并计算
production = ProductionLine()
result = production.calculate_ideal_time()

# 输出详细结果
print("\n=== 生产任务分析 ===")
print(f"目标: {production.target_red}个红桶")

print("\n物料需求:")
print(f"绿桶: {result['material_count']['green']}个")
print(f"黄桶: {result['material_count']['yellow']}个 (6:1转化)")
print(f"红桶: {result['material_count']['red']}个 (6:1转化)")

print("\n设备批次分配:")
print(f"梳棉机: {result['batches']['combing']}批/机 × 10台")
print(f"一并条: {result['batches']['first_drawing']}批/机 × 5台")
print(f"二并条: {result['batches']['second_drawing']}批/机 × 5台")

print("\n理想时间估算:")
print(f"梳棉阶段: {result['combing_time'] / 3600:.2f}小时")
print(f"一并条阶段: {result['first_drawing_time'] / 3600:.2f}小时")
print(f"二并条阶段: {result['second_drawing_time'] / 3600:.2f}小时")
print(f"\n总计理想完成时间: {result['total_time'] / 3600:.2f}小时")

# 计算各阶段吞吐率
print("\n各阶段吞吐分析:")
print(f"梳棉阶段吞吐率: {result['material_count']['green'] / result['combing_time']:.4f}个/秒")
print(f"一并条阶段吞吐率: {result['material_count']['yellow'] / result['first_drawing_time']:.4f}个/秒")
print(f"二并条阶段吞吐率: {result['material_count']['red'] / result['second_drawing_time']:.4f}个/秒")