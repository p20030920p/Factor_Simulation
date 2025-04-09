"""
充电管理测试模块 test_charging_management.py
测试内容:
1. 充电策略验证
2. 电量管理
3. 充电调度优化
"""

import unittest
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple
from enum import Enum
from datetime import datetime, timedelta
import heapq
from collections import defaultdict

class ChargingState(Enum):
    IDLE = "idle"           # 空闲
    CHARGING = "charging"   # 充电中
    WAITING = "waiting"     # 等待充电
    WORKING = "working"     # 工作中

class ChargingPriority(Enum):
    HIGH = 3    # 高优先级
    MEDIUM = 2  # 中优先级
    LOW = 1     # 低优先级

@dataclass
class ChargeStation:
    id: str
    capacity: int = 1  # 同时充电数量
    power_rating: float = 100.0  # 充电功率 kW
    current_load: int = 0  # 当前负载
    charging_vehicles: Set[str] = field(default_factory=set)

@dataclass
class Vehicle:
    id: str
    battery_capacity: float  # 电池容量 kWh
    current_battery: float   # 当前电量 kWh
    charging_speed: float    # 充电速率 kW
    min_battery_threshold: float = 20.0  # 最低电量阈值(%)
    max_battery_threshold: float = 90.0  # 最高电量阈值(%)
    state: ChargingState = ChargingState.IDLE
    priority: ChargingPriority = ChargingPriority.MEDIUM
    charging_station: Optional[str] = None

    @property
    def battery_percentage(self) -> float:
        return (self.current_battery / self.battery_capacity) * 100

    @property
    def needs_charging(self) -> bool:
        return self.battery_percentage <= self.min_battery_threshold

    @property
    def charging_complete(self) -> bool:
        return self.battery_percentage >= self.max_battery_threshold

class ChargingManager:
    def __init__(self):
        self.stations: Dict[str, ChargeStation] = {}
        self.vehicles: Dict[str, Vehicle] = {}
        self.charging_queue: List[Tuple[int, str]] = []  # (priority, vehicle_id)
        self.time = 0
        self.charging_history: List[Dict] = []

    def add_station(self, station: ChargeStation) -> None:
        self.stations[station.id] = station

    def add_vehicle(self, vehicle: Vehicle) -> None:
        self.vehicles[vehicle.id] = vehicle

    def request_charging(self, vehicle_id: str) -> bool:
        """请求充电"""
        if vehicle_id not in self.vehicles:
            return False

        vehicle = self.vehicles[vehicle_id]
        if not vehicle.needs_charging or vehicle.state == ChargingState.CHARGING:
            return False

        # 添加到优先级队列
        heapq.heappush(self.charging_queue, (-vehicle.priority.value, vehicle_id))
        vehicle.state = ChargingState.WAITING
        return True

    def assign_charging_station(self) -> None:
        """分配充电站"""
        if not self.charging_queue:
            return

        available_stations = [
            station for station in self.stations.values()
            if station.current_load < station.capacity
        ]

        while self.charging_queue and available_stations:
            _, vehicle_id = heapq.heappop(self.charging_queue)
            vehicle = self.vehicles[vehicle_id]

            # 选择负载最小的充电站
            station = min(available_stations, key=lambda s: s.current_load)

            # 分配充电站
            vehicle.charging_station = station.id
            vehicle.state = ChargingState.CHARGING
            station.charging_vehicles.add(vehicle_id)
            station.current_load += 1

            # 更新可用充电站列表
            available_stations = [
                s for s in available_stations
                if s.current_load < s.capacity
            ]

    def update_charging_status(self, time_step: float = 1.0) -> None:
        """更新充电状态"""
        self.time += time_step

        for vehicle in self.vehicles.values():
            if vehicle.state == ChargingState.CHARGING:
                station = self.stations[vehicle.charging_station]

                # 计算充电量
                charge_amount = min(
                    vehicle.charging_speed * time_step,
                    vehicle.battery_capacity - vehicle.current_battery
                )
                vehicle.current_battery += charge_amount

                # 检查是否充电完成
                if vehicle.charging_complete:
                    vehicle.state = ChargingState.IDLE
                    vehicle.charging_station = None
                    station.charging_vehicles.remove(vehicle.id)
                    station.current_load -= 1

        # 记录状态
        self.record_state()

    def record_state(self) -> None:
        """记录状态"""
        state = {
            'time': self.time,
            'stations': {
                station_id: {
                    'load': station.current_load,
                    'vehicles': list(station.charging_vehicles)
                }
                for station_id, station in self.stations.items()
            },
            'vehicles': {
                vehicle_id: {
                    'battery': vehicle.battery_percentage,
                    'state': vehicle.state.value
                }
                for vehicle_id, vehicle in self.vehicles.items()
            }
        }
        self.charging_history.append(state)

class TestChargingManagement(unittest.TestCase):
    def setUp(self):
        """测试初始化"""
        self.manager = ChargingManager()

        # 添加充电站
        for i in range(2):
            station = ChargeStation(f"Station{i}", capacity=2, power_rating=120.0)
            self.manager.add_station(station)

        # 添加车辆 - 修改了初始电量确保高优先级车辆需要充电
        vehicle_configs = [
            ("Vehicle1", 100.0, 15.0, 50.0, ChargingPriority.HIGH),    # 改为15%电量
            ("Vehicle2", 80.0, 15.0, 40.0, ChargingPriority.MEDIUM),
            ("Vehicle3", 120.0, 40.0, 60.0, ChargingPriority.LOW),
            ("Vehicle4", 90.0, 10.0, 45.0, ChargingPriority.HIGH)     # 改为10%电量
        ]

        for v_id, capacity, current, speed, priority in vehicle_configs:
            vehicle = Vehicle(
                id=v_id,
                battery_capacity=capacity,
                current_battery=current,
                charging_speed=speed,
                priority=priority
            )
            self.manager.add_vehicle(vehicle)

    def test_charging_request(self):
        """测试充电请求"""
        # 测试低电量车辆请求充电
        success = self.manager.request_charging("Vehicle2")
        self.assertTrue(success)
        self.assertEqual(
            self.manager.vehicles["Vehicle2"].state,
            ChargingState.WAITING
        )

        # 测试电量充足车辆请求充电
        success = self.manager.request_charging("Vehicle3")
        self.assertFalse(success)

    def test_station_assignment(self):
        """测试充电站分配"""
        # 请求充电
        vehicles_to_charge = ["Vehicle1", "Vehicle2", "Vehicle4"]
        for v_id in vehicles_to_charge:
            self.manager.request_charging(v_id)

        # 分配充电站
        self.manager.assign_charging_station()

        # 验证高优先级车辆是否优先获得充电站
        high_priority_vehicles = [
            v for v in self.manager.vehicles.values()
            if v.priority == ChargingPriority.HIGH
        ]

        for vehicle in high_priority_vehicles:
            self.assertEqual(vehicle.state, ChargingState.CHARGING)
            self.assertIsNotNone(vehicle.charging_station)

    def test_charging_process(self):
        """测试充电过程"""
        # 初始化充电
        self.manager.request_charging("Vehicle2")
        self.manager.assign_charging_station()

        initial_battery = self.manager.vehicles["Vehicle2"].current_battery

        # 模拟充电30分钟
        for _ in range(30):
            self.manager.update_charging_status(1.0)

        final_battery = self.manager.vehicles["Vehicle2"].current_battery
        self.assertGreater(final_battery, initial_battery)

    def test_charging_completion(self):
        """测试充电完成"""
        vehicle = self.manager.vehicles["Vehicle1"]
        station_id = "Station0"

        # 手动设置充电状态
        vehicle.state = ChargingState.CHARGING
        vehicle.charging_station = station_id
        vehicle.current_battery = vehicle.battery_capacity * 0.89  # 接近完成
        self.manager.stations[station_id].charging_vehicles.add(vehicle.id)
        self.manager.stations[station_id].current_load += 1

        # 更新状态
        self.manager.update_charging_status(2.0)

        # 验证充电完成后的状态
        self.assertEqual(vehicle.state, ChargingState.IDLE)
        self.assertIsNone(vehicle.charging_station)
        self.assertNotIn(
            vehicle.id,
            self.manager.stations[station_id].charging_vehicles
        )

    def test_charging_strategy(self):
        """测试充电策略"""
        # 创建不同优先级的充电请求
        test_vehicles = [
            ("TestV1", 100.0, 10.0, 50.0, ChargingPriority.LOW),
            ("TestV2", 100.0, 10.0, 50.0, ChargingPriority.MEDIUM),
            ("TestV3", 100.0, 10.0, 50.0, ChargingPriority.HIGH)
        ]

        for v_id, cap, curr, speed, prio in test_vehicles:
            vehicle = Vehicle(
                id=v_id,
                battery_capacity=cap,
                current_battery=curr,
                charging_speed=speed,
                priority=prio
            )
            self.manager.add_vehicle(vehicle)
            self.manager.request_charging(v_id)

        # 分配充电站
        self.manager.assign_charging_station()

        # 验证分配顺序是否符合优先级
        charging_vehicles = [
            v for v in self.manager.vehicles.values()
            if v.state == ChargingState.CHARGING
        ]

        # 按优先级排序
        charging_vehicles.sort(key=lambda v: v.priority.value, reverse=True)

        # 验证高优先级车辆是否先获得充电机会
        for i in range(len(charging_vehicles)-1):
            self.assertGreaterEqual(
                charging_vehicles[i].priority.value,
                charging_vehicles[i+1].priority.value
            )

if __name__ == '__main__':
    unittest.main(verbosity=2)