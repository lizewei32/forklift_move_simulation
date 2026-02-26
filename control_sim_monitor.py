#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多线程仿真：控制线程 + 仿真线程
Control Thread + Simulation Thread Implementation
"""

import json
import threading
import queue
import time
import numpy as np
import copy
from typing import List, Optional
from dataclasses import dataclass
from forklift_simulator import (
    ForkliftParams, 
    ForkliftKinematics,
    ForkliftSimulator,
    ForkliftVisualizer,
    ControlInput,
    RobotState
)
from algorithm import PIDController
from config_manager import get_config
from bezier_curve_generator import BezierCurveGenerator
    

@dataclass
class TargetPoint:
    """目标点"""
    x: float
    y: float
    theta: float  # 目标航向角（可选）
    steer_angle: Optional[float] = 0.0  # 目标舵轮角（可选）
    move_mode: Optional[str] = "FORWARD"  # 运动模式（可选）
    task_mode: Optional[str] = "STOP"  # 任务模式（可选）

@dataclass
class ControlState:
    """控制状态"""
    target_index: int  # 目标点索引
    task_mode: str     # 任务模式
    rotate_stage: int  # 旋转阶段
    last_speed: float  # 上一速度
    last_angle: float  # 上一角度
    record_speed: float  # 关键记录速度
    remain_curve_distance: float  # 剩余曲线距离

class ControlThread(threading.Thread):
    """
    控制线程：根据目标和当前状态计算控制量
    输入：目标位置/角度、运动模式、机器人当前状态
    输出：控制量（舵轮速度和角度）
    """
    
    def __init__(self, targets: List[TargetPoint], 
                 command_queue: queue.Queue,
                 state_queue: queue.Queue,
                 param: ForkliftParams,
                 initial_state: RobotState,
                 dt: float = 0.1):
        """
        初始化控制线程
        
        Args:
            targets: 目标点序列（位置或角度）
            command_queue: 用于发送控制命令的队列
            state_queue: 用于接收机器人状态的队列
            dt: 控制周期 (s)
        """
        super().__init__(name="ControlThread")
        self.targets = targets
        self.forklift_param = param
        self.dt = dt

        self.command_queue = command_queue
        self.state_queue = state_queue
        self.kinematics = ForkliftKinematics(param)
        self.control_state = ControlState(target_index=0, task_mode="STOP", rotate_stage=0, last_speed=0.0, last_angle=0.0, record_speed=0.0, remain_curve_distance=0.0)
        self.robot_state = initial_state
        self.control = ControlInput(time=0.0, steering_speed=0.0, steering_angle=0.0)
        self.running = False
        
        # 控制参数
        self.max_speed = 1.5  # 最大速度 (m/s)
        self.max_acceleration = 1.0  # 最大加速度 (m/s^2)
        self.max_omega = np.deg2rad(30.0)  # 最大角速度 (rad/s)
        self.max_steering_angle = np.deg2rad(100.0)  # 最大转向角 (rad)
        self.max_steer_angle_speed = np.deg2rad(80.0)  # 最大舵轮角速度 (rad/s)
        self.position_tolerance = 0.01  # 位置容差 (m)
        self.angle_tolerance = np.deg2rad(0.02)  # 角度容差 (rad)
        
        # 控制器
        self.move_radial_controller = PIDController(kp=5.0, ki=0.0, kd=0.0)
        self.move_lateral_controller = PIDController(kp=5.0, ki=0.0, kd=5.0)
        self.rotate_controller = PIDController(kp=5.0, ki=0.0, kd=2.0)
        self.steer_angle_controller = PIDController(kp=10.0, ki=0.0, kd=0.0)

    def SetRobotState(self, state: RobotState):
        """设置机器人状态"""
        self.robot_state = state

    def SmoothTarget(self, target: float, current: float, rate: float):
        """平滑目标量"""
        if target > current + rate * self.dt:
            new_target = current + rate * self.dt
        elif target < current - rate * self.dt:
            new_target = current - rate * self.dt
        else:
            new_target = target
        return new_target

    def UpdateNewTarget(self) :
        """更新目标点"""
        if self.control_state.target_index < len(self.targets):
            self.control_state.target_index += 1
            self.control_state.rotate_stage = 0
            self.move_lateral_controller.reset()
            self.move_radial_controller.reset()
            self.rotate_controller.reset()
            self.steer_angle_controller.reset()
            if self.control_state.target_index < len(self.targets) and self.targets[self.control_state.target_index].task_mode == "FAST_ROTATE":
                self.control_state.record_speed = self.control.steering_speed
            if self.control_state.target_index < len(self.targets) and self.targets[self.control_state.target_index].task_mode != "CURVE":
                self.control_state.remain_curve_distance = 0.0
            print(f"切换到目标点 {self.control_state.target_index}, 当前位置: ({self.robot_state.x:.2f}, {self.robot_state.y:.2f}, {self.robot_state.theta:.2f}), 舵轮角: {self.robot_state.steer_angle:.2f}")
        else:
            self.StopControl()

    def UpdateControlResult(self, steer_speed: float, steer_angle: float):
        """更新控制结果"""
        self.control = ControlInput(time=self.robot_state.time, steering_speed=steer_speed, steering_angle=steer_angle)

    def StopControl(self):
        """停止控制"""
        self.control = ControlInput(time=self.robot_state.time, steering_speed=0.0, steering_angle=0.0)

    def MoveControl(self, target: TargetPoint):
        """直线运动控制"""
        sign_theta = 1.0 if target.move_mode == "FORWARD" else -1.0
        cur_x, cur_y, cur_theta = self.robot_state.x, self.robot_state.y, self.robot_state.theta
        target_x, target_y, move_theta = target.x, target.y, target.theta
        if target.move_mode == "BACKWARD":
            move_theta += np.pi
        move_theta = np.arctan2(np.sin(move_theta), np.cos(move_theta))  # 归一化

        # 计算误差
        line_vect = np.array([target_x - cur_x, target_y - cur_y])
        line_dir = np.array([np.cos(move_theta), np.sin(move_theta)])
        remaining_distance = np.dot(line_vect, line_dir)

        reach_threshold = 0.01
        if(self.control_state.target_index+1 < len(self.targets) and self.targets[self.control_state.target_index + 1].task_mode == "FAST_ROTATE"):
            reach_threshold = 2.0  # 提前停止，准备快速旋转
        if remaining_distance < reach_threshold:
            self.UpdateNewTarget()
            return  # 已到达目标

        radial_error = remaining_distance
        if self.control_state.target_index+1 < len(self.targets) and self.targets[self.control_state.target_index + 1].task_mode == "CURVE":
            next_target_theta = self.targets[self.control_state.target_index + 1].theta
            if self.targets[self.control_state.target_index + 1].move_mode == "BACKWARD":
                next_target_theta += np.pi
            diff = next_target_theta - move_theta
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            if abs(diff) < np.deg2rad(0.5):
                radial_error += 2.0  # 如果衔接弧线，则不必速度降到0
            # print(f"MoveControl: 进入曲线, 当前位置: ({cur_x:.2f}, {cur_y:.2f}, {cur_theta:.2f}), 路径角度偏差: {diff:.2f}, {next_target:.2f}, {move_theta:.2f},radial_error: {radial_error:.2f}")
        radial_output = self.move_radial_controller.compute(radial_error, self.dt) * sign_theta
        dece_speed = np.sqrt(abs(2 * self.max_acceleration * radial_error))
        radial_output = np.clip(radial_output, -dece_speed, dece_speed)
        radial_output = np.clip(radial_output, -self.max_speed, self.max_speed)
        
        lateral_dist_error = line_vect[0] * line_dir[1] - line_vect[1] * line_dir[0]
        # 计算角度偏差
        angle_error = target.theta - self.robot_state.theta
        # 归一化到[-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        lateral_error = angle_error - lateral_dist_error * 1.0
        omega_output = self.move_lateral_controller.compute(lateral_error, self.dt)
        omega_output = np.clip(omega_output, -self.max_omega, self.max_omega)
        steer_angle_output = self.kinematics.compute_steering_angle(radial_output, omega_output)
        steer_angle_output = np.clip(steer_angle_output, -self.max_steering_angle, self.max_steering_angle)
        self.UpdateControlResult(radial_output, steer_angle_output)
        # print(f"MoveControl: target=({target_x:.2f}, {target_y:.2f}), pos=({cur_x:.2f}, {cur_y:.2f}, {cur_theta:.2f}), ctrl=({radial_output:.2f}, {steer_angle_output:.2f})")

    def RotateControl(self, target: TargetPoint):
        """旋转控制"""
        cur_steer_angle, cur_theta = self.robot_state.steer_angle, self.robot_state.theta
        cur_x, cur_y = self.robot_state.x, self.robot_state.y
        target_theta = target.theta

        match self.control_state.rotate_stage:
            case 0:
                # 调整舵轮角度
                target_steer_angle = np.pi / 2
                steer_error = target_steer_angle - cur_steer_angle
                steer_omega = self.steer_angle_controller.compute(steer_error, self.dt)
                steer_omega = np.clip(steer_omega, -self.max_steer_angle_speed, self.max_steer_angle_speed)
                steer_angle = cur_steer_angle + steer_omega * self.dt
                self.UpdateControlResult(0.0, steer_angle)

                if(abs(steer_error) < np.deg2rad(0.1)):  # 舵轮旋转到位
                    self.rotate_controller.reset()
                    self.control_state.rotate_stage = 1
                    print(f"RotateControl: 进入旋转阶段 1,当前位置: ({cur_x:.2f}, {cur_y:.2f}, {cur_theta:.2f}), 舵轮角: {cur_steer_angle:.2f}")
            case 1:
                # 旋转，调整叉车角度
                angle_error = target_theta - cur_theta
                angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                omega = self.rotate_controller.compute(angle_error, self.dt)
                omega = np.clip(omega, -self.max_omega, self.max_omega)
                steering_speed = omega * self.forklift_param.steer_dist
                steering_speed = np.clip(steering_speed, -self.max_speed, self.max_speed)
                dece_speed = np.sqrt(abs(2 * self.max_acceleration * abs(angle_error) * self.forklift_param.steer_dist))
                steering_speed = np.clip(steering_speed, -dece_speed, dece_speed)
                self.UpdateControlResult(steering_speed, np.pi / 2)
                # print(f"RotateControl: 当前位置: ({cur_x:.2f}, {cur_y:.2f}, {cur_theta:.2f}), 舵轮角: {cur_steer_angle:.2f}, 控制量：{steering_speed:.2f}")

                if abs(angle_error) < self.angle_tolerance:
                    self.steer_angle_controller.reset()
                    self.control_state.rotate_stage = 2
                    print(f"RotateControl: 进入旋转阶段 2,当前位置: ({cur_x:.2f}, {cur_y:.2f}, {cur_theta:.2f}), 舵轮角: {cur_steer_angle:.2f}")
            case 2:
                # 旋转完成，舵轮回正
                target_steer_angle = 0.0
                steer_error = target_steer_angle - cur_steer_angle
                steer_omega = self.steer_angle_controller.compute(steer_error, self.dt)
                steer_omega = np.clip(steer_omega, -self.max_steer_angle_speed, self.max_steer_angle_speed)
                steer_angle = cur_steer_angle + steer_omega * self.dt
                steer_angle = np.clip(steer_angle, -self.max_steering_angle, self.max_steering_angle)
                self.UpdateControlResult(0.0, steer_angle)
                if abs(steer_error) < np.deg2rad(0.1): 
                    self.UpdateNewTarget()
    
    def FastRotateControl(self, target: TargetPoint):
        """旋转控制"""
        cur_steer_angle, cur_theta = self.robot_state.steer_angle, self.robot_state.theta
        cur_x, cur_y = self.robot_state.x, self.robot_state.y
        target_theta = target.theta

        match self.control_state.rotate_stage:
            case 0:
                # 调整舵轮角度
                angle_error = target_theta - cur_theta
                target_steer_angle = np.pi / 2 * np.sign(angle_error)
                steer_error = target_steer_angle - cur_steer_angle
                steer_omega = self.steer_angle_controller.compute(steer_error, self.dt)
                steer_omega = np.clip(steer_omega, -self.max_steer_angle_speed, self.max_steer_angle_speed)
                steer_angle = cur_steer_angle + steer_omega * self.dt
                self.UpdateControlResult(self.control_state.record_speed, steer_angle)

                if(abs(steer_error) < np.deg2rad(2.0)):  # 舵轮旋转到位
                    self.rotate_controller.reset()
                    self.control_state.rotate_stage = 1
                    print(f"RotateControl: 进入旋转阶段 1,当前位置: ({cur_x:.2f}, {cur_y:.2f}, {cur_theta:.2f}), 舵轮角: {cur_steer_angle:.2f}")
            case 1:
                # 旋转，调整叉车角度
                angle_error = target_theta - cur_theta
                angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                omega = self.rotate_controller.compute(angle_error, self.dt)
                omega = np.clip(omega, -self.max_omega, self.max_omega)
                steering_speed = omega * self.forklift_param.steer_dist
                self.UpdateControlResult(steering_speed, np.pi / 2)
                # print(f"RotateControl: 当前位置: ({cur_x:.2f}, {cur_y:.2f}, {cur_theta:.2f}), 舵轮角: {cur_steer_angle:.2f}, 控制量：{steering_speed:.2f}")

                if abs(angle_error) < np.deg2rad(20.0):
                    self.control_state.record_speed = steering_speed*0.7
                    self.steer_angle_controller.reset()
                    self.control_state.rotate_stage = 2
            case 2:
                # 边转边调整舵轮角度
                angle_error = target_theta - cur_theta
                angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                omega = self.rotate_controller.compute(angle_error, self.dt)
                omega = np.clip(omega, -self.max_omega, self.max_omega)
                steer_angle = self.kinematics.compute_steering_angle(self.control_state.record_speed, omega)
                self.UpdateControlResult(self.control_state.record_speed, steer_angle)
                if abs(angle_error) < np.deg2rad(0.5):
                    self.control_state.rotate_stage = 3
            case 3:
                target_steer_angle = 0.0
                steer_error = target_steer_angle - cur_steer_angle
                steer_omega = self.steer_angle_controller.compute(steer_error, self.dt)
                steer_omega = np.clip(steer_omega, -self.max_steer_angle_speed, self.max_steer_angle_speed)
                steer_angle = cur_steer_angle + steer_omega * self.dt
                steer_angle = np.clip(steer_angle, -self.max_steering_angle, self.max_steering_angle)
                self.UpdateControlResult(0.0, steer_angle)
                if abs(steer_error) < np.deg2rad(10.0): 
                    self.UpdateNewTarget()

    def CurveControl(self, target: TargetPoint):
        """曲线控制"""
        sign_theta = 1.0 if target.move_mode == "FORWARD" else -1.0
        cur_x, cur_y, cur_theta = self.robot_state.x, self.robot_state.y, self.robot_state.theta
        target_x, target_y, move_theta = target.x, target.y, target.theta
        if target.move_mode == "BACKWARD":
            move_theta += np.pi
        move_theta = np.arctan2(np.sin(move_theta), np.cos(move_theta))  # 归一化

        # 计算误差
        line_vect = np.array([target_x - cur_x, target_y - cur_y])
        line_dir = np.array([np.cos(move_theta), np.sin(move_theta)])
        cur_seg_remaining_distance = np.dot(line_vect, line_dir)
        total_curve_distance = cur_seg_remaining_distance
        for i in range(self.control_state.target_index + 1, len(self.targets)):
            new_task = self.targets[i]
            pre_task = self.targets[i - 1]
            if new_task.task_mode == "CURVE":
                next_dist = np.linalg.norm(np.array([new_task.x - pre_task.x, new_task.y - pre_task.y]))
                total_curve_distance += next_dist
            else:
                diff_theta = new_task.theta - pre_task.theta
                diff_theta = np.arctan2(np.sin(diff_theta), np.cos(diff_theta))
                if abs(diff_theta) < self.angle_tolerance and \
                   new_task.task_mode == "LINE" and new_task.move_mode == pre_task.move_mode:
                    total_curve_distance += np.linalg.norm(np.array([new_task.x - pre_task.x, new_task.y - pre_task.y]))
                # print(f"CurveControl: 停止累积曲线距离 at target {i}, {new_task.theta:.2f}, {pre_task.theta:.2f}, diff:{diff_theta:.2f}, 当前曲线距离: {total_curve_distance:.2f}")
                break
        self.control_state.remain_curve_distance = total_curve_distance

        reach_threshold = 0.5
        if cur_seg_remaining_distance < reach_threshold:
            self.UpdateNewTarget()
            return  # 已到达目标

        radial_error = total_curve_distance
        radial_output = self.move_radial_controller.compute(radial_error, self.dt) * sign_theta
        dece_speed = np.sqrt(abs(2 * self.max_acceleration * total_curve_distance))
        radial_output = np.clip(radial_output, -dece_speed, dece_speed)
        radial_output = np.clip(radial_output, -self.max_speed, self.max_speed)

        lateral_dist_error = line_vect[0] * line_dir[1] - line_vect[1] * line_dir[0]
        # 计算角度偏差
        angle_error = target.theta - self.robot_state.theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        lateral_error = angle_error - lateral_dist_error * 3.0
        omega_output = self.move_lateral_controller.compute(lateral_error, self.dt)
        # omega_output = np.clip(omega_output, -self.max_omega, self.max_omega)
        steer_angle_output = self.kinematics.compute_steering_angle(radial_output, omega_output)
        steer_angle_output = np.clip(steer_angle_output, -self.max_steering_angle, self.max_steering_angle)
        self.UpdateControlResult(radial_output, steer_angle_output)

    def TaskStateMachine(self):
        """任务状态机"""
        current_task = TargetPoint(0.0, 0.0, 0.0)
        if not self.targets:
            self.control_state.task_mode = "STOP"
        elif self.control_state.target_index >= len(self.targets):
            self.targets.clear()
            print("所有目标点已完成")
            self.control_state.task_mode = "STOP"
        else:
            self.control_state.task_mode = self.targets[self.control_state.target_index].task_mode
            current_task = self.targets[self.control_state.target_index]

        match self.control_state.task_mode:
            case "LINE":
                self.MoveControl(current_task)
            case "ROTATE":
                self.RotateControl(current_task)
            case "FAST_ROTATE":
                self.FastRotateControl(current_task)
            case "CURVE":
                self.CurveControl(current_task)
            case _:
                self.StopControl()

        self.control.steering_speed = self.SmoothTarget(self.control.steering_speed, self.control_state.last_speed, self.max_speed)
        self.control.steering_angle = self.SmoothTarget(self.control.steering_angle, self.control_state.last_angle, self.max_steer_angle_speed)
        self.control_state.last_speed = self.control.steering_speed
        self.control_state.last_angle = self.control.steering_angle

    def run(self):
        """线程主循环"""
        print(f"[{self.name}] 启动")
        self.running = True

        while self.running and self.targets is not None and self.control_state.target_index < len(self.targets):
            # 从状态队列获取最新状态（非阻塞）
            try:
                while not self.state_queue.empty():
                    self.robot_state = self.state_queue.get_nowait()
            except queue.Empty:
                pass

            self.TaskStateMachine()

            # 发送到队列
            command = copy.deepcopy(self.control)
            try:
                self.command_queue.put(command, timeout=0.1)
            except queue.Full:
                print(f"[{self.name}] 警告: 命令队列已满")
            
            # time.sleep(0.001)  # 短暂延迟

        # 发送结束信号
        self.command_queue.put(None)
        self.running = False
        print(f"[{self.name}] 已完成，总时间 {self.robot_state.time:.2f}s")

    def stop(self):
        """停止线程"""
        self.running = False


class SimulationThread(threading.Thread):
    """
    仿真线程：接收控制命令并更新机器人状态
    输入：控制量（舵轮速度和角度）
    输出：机器人更新后的状态，并保存到trajectory
    """
    
    def __init__(self, simulator: ForkliftSimulator, 
                 command_queue: queue.Queue,
                 state_queue: queue.Queue):
        """
        初始化仿真线程
        
        Args:
            simulator: 叉车仿真器
            command_queue: 用于接收控制命令的队列
            state_queue: 用于发送状态更新的队列
        """
        super().__init__(name="SimulationThread")
        self.simulator = simulator
        self.command_queue = command_queue
        self.state_queue = state_queue
        self.running = False
        
        # 重置仿真器状态
        # self.simulator.reset()
        
        # 扩展轨迹记录，包含舵轮角度
        self.trajectory_with_steering = []  # [(x, y, theta, steering_angle), ...]
        
    def run(self):
        """线程主循环"""
        print(f"[{self.name}] 启动")
        self.running = True
        
        # 发送初始状态
        initial_state = self.simulator.state  # [x, y, theta, steering_angle_rad]
        self.state_queue.put(initial_state)
        
        while self.running:
            try:
                # 从队列获取控制命令（阻塞，超时1秒）
                command = self.command_queue.get(timeout=0.1)
                
                # 检查是否是结束信号
                if command is None:
                    print(f"[{self.name}] 收到结束信号")
                    break
                
                # 立即执行仿真步骤
                self.simulator.simulate_single_step(command)

                # 获取更新后的状态
                new_state = self.simulator.get_robot_state()  # [x, y, theta, steering_angle_rad]

                # 发送状态更新（非阻塞）
                try:
                    self.state_queue.put_nowait(new_state)
                except queue.Full:
                    # 队列满了，丢弃旧状态
                    try:
                        self.state_queue.get_nowait()
                        self.state_queue.put_nowait(new_state)
                    except:
                        pass
                
                # 标记任务完成
                self.command_queue.task_done()
                
            except queue.Empty:
                # 超时，继续等待
                continue
            except Exception as e:
                print(f"[{self.name}] 错误: {e}")
                import traceback
                traceback.print_exc()
                break
        
        self.running = False
        print(f"[{self.name}] 已完成，生成 {len(self.simulator.trajectory)} 个轨迹点")
    
    def stop(self):
        """停止线程"""
        self.running = False


def load_control_inputs_from_file(filename: str) -> List[ControlInput]:
    """从JSON文件加载控制输入"""
    with open(filename, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    return [ControlInput(**item) for item in data]


def main():
    """主函数"""
    
    # 加载配置
    config = get_config('config.yaml')
    
    print("=" * 60)
    print("无人叉车路径仿真 - 多线程版本")
    print("=" * 60)
    
    # 创建叉车参数
    forklift_params = ForkliftParams(
        length=config.forklift.length,
        width=config.forklift.width,
        steer_dist=config.forklift.steer_dist,
        track_width=config.forklift.track_width,
    )
    
    print("\n叉车参数:")
    print(f"  车长: {forklift_params.length} m")
    print(f"  车宽: {forklift_params.width} m")
    print(f"  轮间距: {forklift_params.steer_dist} m")
    print(f"  左右轮距: {forklift_params.track_width} m")

    task_type = 4 # 1:标准原地转弯； 2:走过弧线转弯； 3:提前弧线转向；4: 弧线拐弯后直线对接
    bezier_offset = 0.0
    if task_type == 2:
        print("\n任务类型: 标准原地转弯")
        # bezier_offset = -2
        bezier_generator = BezierCurveGenerator([(13, bezier_offset), (11, bezier_offset), (10, -1), (10, 1)])
        bezier_curve_points, bezier_curve_angle = bezier_generator.get_tangent_angles(num_points=50)
        bezier_generator.visualize_bezier_curve(save_path="bezier_visualization.png")
    elif task_type == 4:
        print("\n任务类型: 弧线拐弯后直线对接")
        bezier_offset = 0
        bezier_generator = BezierCurveGenerator([(7, bezier_offset), (9, bezier_offset), (10, -1+bezier_offset), (10, -3+bezier_offset)])
        bezier_curve_points, bezier_curve_angle = bezier_generator.get_tangent_angles(num_points=50)
        bezier_generator.visualize_bezier_curve(save_path="bezier_visualization.png")

    targets = []
    initial_state = RobotState(time=0.0, x=0.0, y=bezier_offset, theta=0.0, steer_angle=0.0)
    
    match task_type:
        case 1:
            # initial_state.theta = -np.deg2rad(30.0) # 模拟验证贴路径行走方案
            targets.append(TargetPoint(x=10.0, y=0.0, theta=0.0, move_mode="FORWARD", task_mode="LINE"))
            targets.append(TargetPoint(x=10.0, y=0.0, theta=-np.pi/2, move_mode="FORWARD", task_mode="ROTATE"))
            targets.append(TargetPoint(x=10.0, y=3.0, theta=-np.pi/2, move_mode="BACKWARD", task_mode="LINE"))
        case 2:
            targets.append(TargetPoint(x=13.0, y=bezier_offset, theta=0.0, move_mode="FORWARD", task_mode="LINE"))
            for i in range(len(bezier_curve_points)):
                targets.append(TargetPoint(x=bezier_curve_points[i][0], y=bezier_curve_points[i][1], theta=bezier_curve_angle[i]+np.pi, move_mode="BACKWARD", task_mode="CURVE"))
            targets.append(TargetPoint(x=10.0, y=3.0, theta=-np.pi/2, move_mode="BACKWARD", task_mode="LINE"))
        case 3:
            targets.append(TargetPoint(x=10.0, y=0.0, theta=0.0, move_mode="FORWARD", task_mode="LINE"))
            targets.append(TargetPoint(x=10.0, y=0.0, theta=-np.pi/2, move_mode="FORWARD", task_mode="FAST_ROTATE"))
            targets.append(TargetPoint(x=10.0, y=3.0, theta=-np.pi/2, move_mode="BACKWARD", task_mode="LINE"))
        case 4:
            targets.append(TargetPoint(x=7.0, y=bezier_offset, theta=0.0, move_mode="FORWARD", task_mode="LINE"))
            for i in range(len(bezier_curve_points)):
                targets.append(TargetPoint(x=bezier_curve_points[i][0], y=bezier_curve_points[i][1], theta=bezier_curve_angle[i], move_mode="FORWARD", task_mode="CURVE"))
            targets.append(TargetPoint(x=10.0, y=3.0, theta=-np.pi/2, move_mode="BACKWARD", task_mode="LINE"))

    # 创建命令队列和状态队列
    command_queue = queue.Queue(maxsize=1)
    state_queue = queue.Queue(maxsize=1)

    # 创建仿真器
    print("\n初始化仿真器...")
    simulator = ForkliftSimulator(forklift_params, init_state=initial_state, dt=config.simulation.dt)

    # 创建控制线程和仿真线程
    print("\n创建线程...")
    control_thread = ControlThread(
        targets=targets,
        command_queue=command_queue,
        state_queue=state_queue,
        param=forklift_params,
        initial_state=initial_state,
        dt=config.simulation.dt
    )
    
    simulation_thread = SimulationThread(
        simulator=simulator,
        command_queue=command_queue,
        state_queue=state_queue
    )
    
    # 启动线程
    print("\n启动仿真线程...")
    simulation_thread.start()
    
    print("启动控制线程...")
    control_thread.start()
    
    # 等待线程完成
    print("\n等待线程完成...\n")
    control_thread.join()
    simulation_thread.join()
    
    print("\n" + "=" * 60)
    print("多线程仿真完成!")
    print("=" * 60)
    
    # 计算轨迹统计信息
    trajectory = np.array(simulator.get_trajectory())  # [[x, y, theta, steer_angle], ...]
    total_distance = 0.0
    for i in range(1, len(trajectory)):
        dx = trajectory[i].x - trajectory[i-1].x
        dy = trajectory[i].y - trajectory[i-1].y
        total_distance += np.sqrt(dx**2 + dy**2)
    
    print("\n轨迹统计:")
    print(f"  总行驶距离: {total_distance:.2f} m")
    print(f"  起点坐标: ({trajectory[0].x:.2f}, {trajectory[0].y:.2f}) m")
    print(f"  终点坐标: ({trajectory[-1].x:.2f}, {trajectory[-1].y:.2f}) m")

    # 创建可视化器
    print("\n创建可视化器...")
    visualizer = ForkliftVisualizer(simulator, config=config)
    
    # 生成可视化
    print("\n生成可视化...")
    print("  [1/2] 生成静态轨迹图...")
    visualizer.visualize_static(save_path='trajectory_static.png')
    
    print("  [2/2] 生成动态轨迹动画...")
    # 🚀 优化参数：sample_step越大越快，dpi越低越快
    # - sample_step=4: 高质量，慢
    # - sample_step=8: 中等质量，较快（推荐）
    # - sample_step=16: 低质量，很快
    # - dpi=80: 快速预览
    # - dpi=100: 平衡（推荐）
    # - dpi=150: 高质量
    visualizer.visualize_animated(
        save_path='trajectory_animated.gif', 
        interval=50,
        sample_step=4,  # 🚀 采样步长，越大越快
        dpi=100         # 🚀 分辨率，越低越快
    )
    
    print("\n" + "=" * 60)
    print("所有任务完成!")
    print("生成的文件:")
    print("  - trajectory_static.png")
    print("  - trajectory_animated.gif")
    print("=" * 60)


if __name__ == "__main__":
    main()
