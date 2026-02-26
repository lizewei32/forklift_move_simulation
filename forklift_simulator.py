#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人叉车路径仿真软件 - 重构版本
Autonomous Forklift Path Simulation Software - Refactored Version

模块结构:
- ForkliftParams: 叉车参数数据类
- ControlInput: 控制输入数据类
- ForkliftKinematics: 叉车运动学模型
- ForkliftSimulator: 叉车仿真器（纯仿真逻辑）
- ForkliftVisualizer: 叉车可视化器（纯可视化逻辑）
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，避免显示窗口
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
from dataclasses import dataclass
from typing import List, Tuple, Optional
import warnings

# 忽略字体警告
warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')

# 配置matplotlib以支持中文（如果可能）
try:
    plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
except:
    pass


@dataclass
class ForkliftParams:
    """叉车参数 / Forklift Parameters"""
    length: float  # 车体长度 (m)
    width: float  # 车体宽度 (m)
    steer_dist: float  # 轮间距 - 舵轮距离叉车中心的距离 (m)
    track_width: float  # 轮距 - 左右轮距离 (m)


@dataclass
class ControlInput:
    """控制输入 / Control Input"""
    time: float  # 时间 (s)
    steering_speed: float  # 舵轮速度 (m/s)
    steering_angle: float  # 舵轮角度 (degrees)


@dataclass
class RobotState:
    """机器人状态"""
    time: float
    x: float
    y: float
    theta: float  # 航向角（弧度）
    steer_angle: float  # 舵轮角度（弧度）


class ForkliftKinematics:
    """
    叉车运动学模型
    负责计算叉车的运动学更新和几何形状
    """
    
    def __init__(self, params: ForkliftParams):
        """
        初始化运动学模型
        
        Args:
            params: 叉车参数
        """
        self.params = params

    def compute_velocity(self, state: RobotState, steering_speed: float, 
                        steering_angle: float) -> Tuple[float, float, float]:
        """
        计算车体的速度和角速度
        
        Args:
            state: 当前状态 [x, y, theta, steer_angle]
            steering_speed: 舵轮速度 (m/s)
            steering_angle: 舵轮角度 (rad)
            
        Returns:
            (v_x, v_y, omega): 车体中心的x速度、y速度和角速度
        """
        theta = state.theta

        # 使用简化的运动学模型
        if abs(steering_angle) < 0.001:
            # 直线运动
            v_x = steering_speed * np.cos(theta)
            v_y = steering_speed * np.sin(theta)
            omega = 0.0
        else:
            # 计算转弯半径和角速度
            R_steer = abs(self.params.steer_dist / np.sin(steering_angle))
            omega = steering_speed / R_steer * np.sign(steering_angle)
            v_center = steering_speed * np.cos(steering_angle)
            # 车体中心速度
            v_x = v_center * np.cos(theta)
            v_y = v_center * np.sin(theta)
        
        return v_x, v_y, omega

    def update_state(self, state: RobotState, steering_speed: float,
                     steering_angle: float, dt: float) -> RobotState:
        """
        更新叉车状态
        
        Args:
            state: 当前状态 [x, y, theta, steer_angle]
            steering_speed: 舵轮速度 (m/s)
            steering_angle: 舵轮角度 (rad)
            dt: 时间步长 (s)
            
        Returns:
            新状态 [x_new, y_new, theta_new, steer_angle_new]
        """
        
        # 计算速度
        v_x, v_y, omega = self.compute_velocity(state, steering_speed, steering_angle)
        
        # 更新状态
        x_new = state.x + v_x * dt
        y_new = state.y + v_y * dt
        theta_new = state.theta + omega * dt

        # 归一化角度到 [-pi, pi]
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

        return RobotState(
            time=state.time + dt,
            x=x_new,
            y=y_new,
            theta=theta_new,
            steer_angle=steering_angle
        )

    def compute_steering_angle(self, robot_speed: float, robot_omega: float) -> float:
        """
        计算叉车舵轮角度

        Args:
            robot_speed: 叉车速度 (m/s)
            robot_omega: 叉车角速度 (rad/s)

        Returns:
            叉车的舵轮角度 (rad)
        """
        if abs(robot_speed) < 1e-3 or abs(robot_omega) < 1e-3:
            return 0.0
        else:
            R_robot = abs(robot_speed / robot_omega)
            steer_angle = np.arctan2(self.params.steer_dist, R_robot) * np.sign(robot_omega) * np.sign(robot_speed)
            return steer_angle


class ForkliftSimulator:
    """
    叉车仿真器
    负责执行仿真逻辑，不包含可视化
    """

    def __init__(self, params: ForkliftParams, init_state: RobotState, dt: float = 0.1):
        """
        初始化仿真器
        
        Args:
            params: 叉车参数
            dt: 仿真时间步长 (s)
        """
        self.params = params
        self.dt = dt
        self.kinematics = ForkliftKinematics(params)
        
        # 初始状态：[x, y, theta, steer_angle]
        self.state = init_state

        # 轨迹记录
        self.trajectory = [self.state]

    def reset(self, initial_state: Optional[RobotState] = None):
        """
        重置仿真器
        
        Args:
            initial_state: 初始状态，如果为None则使用默认值
        """
        if initial_state is None:
            self.state = RobotState(time=0.0, x=0.0, y=0.0, theta=0.0, steer_angle=0.0)
        else:
            self.state = initial_state
        self.trajectory = [self.state]

    def step(self, steering_speed: float, steering_angle: float):
        """
        执行一步仿真
        
        Args:
            steering_speed: 舵轮速度 (m/s)
            steering_angle: 舵轮角度 (rad)
        """
        # 使用运动学模型更新状态
        self.state = self.kinematics.update_state(
            self.state, steering_speed, steering_angle, self.dt
        )
        
        # 记录轨迹
        self.trajectory.append(self.state)
    
    def interpolate_control(self, control_inputs: List[ControlInput], 
                          current_time: float) -> Tuple[float, float]:
        """
        对控制输入进行线性插值
        
        Args:
            control_inputs: 控制输入列表
            current_time: 当前时间
            
        Returns:
            (speed, angle): 插值后的速度和角度
        """
        if not control_inputs:
            return 0.0, 0.0
        
        # 找到当前时刻对应的控制输入索引
        input_idx = 0
        for i in range(len(control_inputs) - 1):
            if control_inputs[i + 1].time > current_time:
                input_idx = i
                break
        else:
            input_idx = len(control_inputs) - 1
        
        # 线性插值
        if input_idx < len(control_inputs) - 1:
            t1 = control_inputs[input_idx].time
            t2 = control_inputs[input_idx + 1].time
            alpha = (current_time - t1) / (t2 - t1) if t2 > t1 else 0.0
            
            speed = (1 - alpha) * control_inputs[input_idx].steering_speed + \
                    alpha * control_inputs[input_idx + 1].steering_speed
            angle = (1 - alpha) * control_inputs[input_idx].steering_angle + \
                    alpha * control_inputs[input_idx + 1].steering_angle
        else:
            speed = control_inputs[input_idx].steering_speed
            angle = control_inputs[input_idx].steering_angle
        
        return speed, angle
    
    def simulate(self, control_inputs: List[ControlInput]):
        """
        运行完整仿真
        
        Args:
            control_inputs: 控制输入列表，按时间排序
        """
        self.reset()
        
        if not control_inputs:
            return

        current_time = control_inputs[0].time
        final_time = control_inputs[-1].time
        
        while current_time <= final_time:
            # 获取当前时刻的控制输入（插值）
            speed, angle = self.interpolate_control(control_inputs, current_time)
            
            # 执行一步仿真
            self.step(speed, angle)
            current_time += self.dt

    def simulate_single_step(self, control_input: ControlInput) -> RobotState:
        """
        执行单步仿真

        Args:
            control_input: 单个控制输入
        """
        self.step(control_input.steering_speed, control_input.steering_angle)
        return self.state

    def get_trajectory(self) -> List[RobotState]:
        """获取轨迹数组"""
        return self.trajectory

    def get_robot_state(self) -> RobotState:
        """获取当前机器人状态"""
        return self.state

    def get_trajectory_sample(self, step: int) -> List[RobotState]:
        """采样时保证保留首尾点"""
        if len(self.trajectory) <= 2:
            return self.trajectory

        # 采样中间部分
        middle = self.trajectory[1:-1:step]
        
        # 合并首尾
        return [self.trajectory[0]] + middle + [self.trajectory[-1]]


class ForkliftVisualizer:
    """
    叉车可视化器
    负责绘制静态图和动画
    """
    
    def __init__(self, simulator: ForkliftSimulator, config=None):
        """
        初始化可视化器
        
        Args:
            simulator: 叉车仿真器
            config: 配置对象 (可选，用于可视化参数)
        """
        self.simulator = simulator
        self.config = config
        self.kinematics = simulator.kinematics
        self.params = simulator.params

    def _get_config_value(self, *keys, default=None):
        """
        从配置中获取嵌套的值
        
        Args:
            *keys: 嵌套的键
            default: 默认值
            
        Returns:
            配置值或默认值
        """
        if not self.config:
            return default
        
        value = self.config
        for key in keys:
            if hasattr(value, key):
                value = getattr(value, key)
            elif isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value

    def get_body_shape(self, state: RobotState) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        获取叉车的详细形状：车身 + 左后轮 + 右后轮
        
        Args:
            state: 叉车状态 [x, y, theta, steer_angle]
            
        Returns:
            body_corners: 车身矩形的四个角 (4x2)
            left_wheel_corners: 左后轮矩形的四个角 (4x2)
            right_wheel_corners: 右后轮矩形的四个角 (4x2)
        """
        x, y, theta = state.x, state.y, state.theta

        # 车身尺寸：前面的主体矩形
        body_length = self._get_config_value('body', 'length', default=self.params.length * 0.3) or (self.params.length * 0.3)
        body_width = self._get_config_value('body', 'width', default=self.params.width) or self.params.width
        body_front = self._get_config_value('body', 'front', default=self.params.length * 0.9) or (self.params.length * 0.9)
        body_back = body_front - body_length

        # 车身四个角（相对于车体中心）
        body_corners_local = np.array([
            [body_front, body_width/2],      # 右前
            [body_front, -body_width/2],     # 左前
            [body_back, -body_width/2],      # 左后
            [body_back, body_width/2]        # 右后
        ])
        
        # 后轮尺寸：两个长条矩形
        wheel_length = self._get_config_value('wheel', 'length', default=self.params.length * 0.7) or (self.params.length * 0.7)
        wheel_width = self._get_config_value('wheel', 'width', default=self.params.width * 0.2) or (self.params.width * 0.2)
        wheel_back = self._get_config_value('wheel', 'back', default=-self.params.length * 0.1) or (-self.params.length * 0.1)
        wheel_front = wheel_back + wheel_length
        wheel_gap = self._get_config_value('wheel', 'gap', default=self.params.width * 0.3) or (self.params.width * 0.3)

        # 左后轮四个角
        left_wheel_corners_local = np.array([
            [wheel_front, -wheel_gap/2 - wheel_width],   # 右前
            [wheel_front, -wheel_gap/2],                 # 左前
            [wheel_back, -wheel_gap/2],                  # 左后
            [wheel_back, -wheel_gap/2 - wheel_width]     # 右后
        ])
        
        # 右后轮四个角
        right_wheel_corners_local = np.array([
            [wheel_front, wheel_gap/2 + wheel_width],    # 右前
            [wheel_front, wheel_gap/2],                  # 左前
            [wheel_back, wheel_gap/2],                   # 左后
            [wheel_back, wheel_gap/2 + wheel_width]      # 右后
        ])
        
        # 旋转矩阵
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        
        # 转换到全局坐标系
        body_corners_global = body_corners_local @ R.T + np.array([x, y])
        left_wheel_corners_global = left_wheel_corners_local @ R.T + np.array([x, y])
        right_wheel_corners_global = right_wheel_corners_local @ R.T + np.array([x, y])
        
        return body_corners_global, left_wheel_corners_global, right_wheel_corners_global

    def get_steering_wheel_position(self, state: RobotState) -> Tuple[float, float, float]:
        """
        获取舵轮位置和方向
        
        Args:
            state: [x, y, theta, steer_angle] 车体状态
            
        Returns:
            steer_x, steer_y: 舵轮中心位置
            steer_direction: 舵轮方向角度（全局坐标系）
        """
        x, y, theta, steer_angle = state.x, state.y, state.theta, state.steer_angle

        # 舵轮在车体中心线上，距离车体中心前方steer_dist
        steer_x = x + self.params.steer_dist * np.cos(theta)
        steer_y = y + self.params.steer_dist * np.sin(theta)
        
        # 舵轮方向 = 车体方向 + 转向角
        steer_direction = theta + steer_angle
        
        return steer_x, steer_y, steer_direction
    
    def _draw_multiple_pallets(self, ax, reference_state: RobotState, 
                              colors: Optional[dict] = None, 
                              linewidth: Optional[dict] = None,
                              add_label: bool = True) -> List[np.ndarray]:
        """
        绘制多个横向排列的托盘
        
        Args:
            ax: matplotlib axes
            reference_state: 参考状态（通常是终点状态）
            colors: 颜色配置
            linewidth: 线宽配置
            add_label: 是否添加图例标签
            
        Returns:
            所有托盘的角点列表
        """
        if colors is None:
            colors = {}
        if linewidth is None:
            linewidth = {}
            
        # 获取配置参数
        pallet_count = self._get_config_value('visualization', 'pallet_count', default=5) or 5
        pallet_spacing = self._get_config_value('visualization', 'pallet_spacing', default=2.5) or 2.5
        pallet_color = colors.get('pallet_border', 'gray')
        pallet_fill_color = colors.get('pallet_fill', 'lightgray')
        
        all_pallet_corners = []
        
        # 计算第一个托盘位置（参考点处）
        first_pallet_corners = self._calculate_pallet_corners(reference_state)
        all_pallet_corners.append(first_pallet_corners)
        
        # 绘制第一个托盘
        pallet_corners_closed = np.vstack([first_pallet_corners, first_pallet_corners[0]])
        label = '托盘' if add_label else None
        ax.plot(pallet_corners_closed[:, 0], pallet_corners_closed[:, 1], 
               color='m', linewidth=linewidth.get('pallet', 3), 
               linestyle='--', label=label)
        ax.fill(pallet_corners_closed[:, 0], pallet_corners_closed[:, 1], 
               color=pallet_fill_color, alpha=0.3)
        
        # 计算托盘的横向方向（垂直于参考朝向）
        ref_theta = -np.pi/2  # reference_state.theta
        perpendicular_direction = np.array([-np.sin(ref_theta), np.cos(ref_theta)])
        
        # 绘制其他托盘（横向排列）
        for i in range(1, pallet_count):
            # 计算偏移（向左或向右）
            offset_sign = -1  # 统一向一侧排列
            offset_index = i
            offset = perpendicular_direction * offset_sign * offset_index * pallet_spacing
            
            # 平移托盘角点
            shifted_corners = first_pallet_corners + offset
            all_pallet_corners.append(shifted_corners)
            
            # 绘制托盘
            pallet_corners_closed = np.vstack([shifted_corners, shifted_corners[0]])
            ax.plot(pallet_corners_closed[:, 0], pallet_corners_closed[:, 1], 
                   color=pallet_color, linewidth=linewidth.get('pallet', 2), 
                   linestyle='--')
            ax.fill(pallet_corners_closed[:, 0], pallet_corners_closed[:, 1], 
                   color=pallet_fill_color, alpha=0.3)
        
        # 绘制连接所有托盘底部的直线
        bottom_points = []
        for corners in all_pallet_corners:
            # 找到托盘底部中点（索引0和1之间）
            bottom_center = (corners[0] + corners[1]) / 2
            bottom_points.append(bottom_center)
        
        # 按横向位置排序（用于连线）
        bottom_points = sorted(bottom_points, 
                              key=lambda p: p[0] * perpendicular_direction[0] + 
                                          p[1] * perpendicular_direction[1])
        bottom_points = np.array(bottom_points)
        
        # 绘制连接线
        align_label = '托盘对齐线' if add_label else None
        line_points = np.array([bottom_points[0], bottom_points[-1]])
        line_points[0][0] -= self._get_config_value('pallet', 'width', default=1.8) 
        line_points[1][0] += self._get_config_value('pallet', 'width', default=1.8)
        line_points[:, 1] -= 0.1 # 可调整高度偏移
        
        ax.plot(line_points[:, 0], line_points[:, 1], 
               color='darkgray', linewidth=3, linestyle='-', 
               label=align_label, zorder=1)

        path_width = self._get_config_value('forklift', 'path_width', default=3.0) or 3.0

        line_points[:, 1] -= 0.1 + path_width # 可调整高度偏移
        ax.plot(line_points[:, 0], line_points[:, 1],
               color='darkgray', linewidth=3, linestyle='-',
               label=align_label, zorder=1)
        
        return all_pallet_corners
    
    def _calculate_pallet_corners(self, end_state: RobotState) -> np.ndarray:
        """
        计算托盘的四个角点
        
        Args:
            end_state: 终点状态
            
        Returns:
            托盘角点坐标 (4x2)
        """
        pallet_length = self._get_config_value('pallet', 'length', 
                                               default=0.7 * self.params.length) or (0.7 * self.params.length)
        pallet_width = self._get_config_value('pallet', 'width', default=1.8) or 1.8
        pallet_offset = self._get_config_value('pallet', 'offset', 
                                               default=-0.1 * self.params.length) or (-0.1 * self.params.length)

        # 托盘中心位置
        pallet_center_x = end_state.x + (pallet_length / 2 + pallet_offset) * np.cos(end_state.theta)
        pallet_center_y = end_state.y + (pallet_length / 2 + pallet_offset) * np.sin(end_state.theta)

        # 托盘四个角点（相对于托盘中心）
        half_length = pallet_length / 2
        half_width = pallet_width / 2
        pallet_corners_local = np.array([
            [half_length, half_width],
            [half_length, -half_width],
            [-half_length, -half_width],
            [-half_length, half_width]
        ])
        
        # 旋转矩阵
        theta = -np.pi/2
        R_pallet = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        
        # 转换到全局坐标系
        pallet_corners_global = pallet_corners_local @ R_pallet.T + \
                               np.array([pallet_center_x, pallet_center_y])
        
        return pallet_corners_global
    
    def _calculate_axis_limits(self, trajectory: List[RobotState], 
                              pallet_corners_list) -> Tuple[float, float, float, float]:
        """
        计算坐标轴范围
        
        Args:
            trajectory: 轨迹数组
            pallet_corners_list: 托盘角点数组或列表
            
        Returns:
            (x_min, x_max, y_min, y_max)
        """
        all_x = []
        all_y = []
        
        # 收集所有轨迹点的叉车角点
        sample_step = self._get_config_value('visualization', 'drawing', 'sample_step', default=50)
        step = max(1, len(trajectory) // sample_step) if sample_step else 50
        
        for i in range(0, len(trajectory), step):
            body_corners, left_wheel_corners, right_wheel_corners = self.get_body_shape(trajectory[i])
            all_x.extend(body_corners[:, 0])
            all_y.extend(body_corners[:, 1])
            all_x.extend(left_wheel_corners[:, 0])
            all_y.extend(left_wheel_corners[:, 1])
            all_x.extend(right_wheel_corners[:, 0])
            all_y.extend(right_wheel_corners[:, 1])
        
        # 加入托盘角点（支持单个数组或列表）
        if isinstance(pallet_corners_list, list):
            for pallet_corners in pallet_corners_list:
                all_x.extend(pallet_corners[:, 0])
                all_y.extend(pallet_corners[:, 1])
        else:
            all_x.extend(pallet_corners_list[:, 0])
            all_y.extend(pallet_corners_list[:, 1])
        
        # 计算边界
        margin_ratio = self._get_config_value('visualization', 'drawing', 'margin_ratio', default=1.0)
        margin = max(self.simulator.params.length, self.simulator.params.width) * (margin_ratio if margin_ratio else 1.0)
        
        x_min = min(all_x) - margin
        x_max = max(all_x) + margin
        y_min = min(all_y) - margin
        y_max = max(all_y) + margin

        return x_min, x_max, y_min, y_max
    
    def visualize_static(self, save_path: Optional[str] = None):
        """
        静态可视化完整轨迹
        
        Args:
            save_path: 保存路径，如果为None则不保存
        """
        trajectory = self.simulator.get_trajectory()
        
        # 获取配置参数
        figsize = (
            self._get_config_value('visualization', 'figure', 'width_static', default=12) or 12,
            self._get_config_value('visualization', 'figure', 'height_static', default=10) or 10
        )
        colors = self._get_config_value('visualization', 'colors', 
                                       default={'trajectory': 'b', 'body': 'steelblue', 
                                               'wheel': 'black', 'steering_wheel': 'orange'}) or {'trajectory': 'b', 'body': 'steelblue', 'wheel': 'black', 'steering_wheel': 'orange'}
        linewidth = self._get_config_value('visualization', 'linewidth',
                                          default={'trajectory': 2, 'body': 2, 'wheel': 2, 'pallet': 3}) or {'trajectory': 2, 'body': 2, 'wheel': 2, 'pallet': 3}
        markers = self._get_config_value('visualization', 'markers',
                                        default={'start': 10, 'end': 10}) or {'start': 10, 'end': 10}
        
        fig, ax = plt.subplots(figsize=figsize)
        
        # 绘制轨迹
        ax.plot([state.x for state in trajectory], [state.y for state in trajectory], 
               color=colors.get('trajectory', 'b'), 
               linewidth=linewidth.get('trajectory', 2), 
               label='轨迹')
        
        # 绘制起点和终点
        ax.plot(trajectory[0].x, trajectory[0].y, 'go', 
               markersize=markers.get('start', 10), label='起点')
        ax.plot(trajectory[-1].x, trajectory[-1].y, 'ro', 
               markersize=markers.get('end', 10), label='终点')
        
        # 绘制多个托盘（使用封装的函数）
        all_pallet_corners = self._draw_multiple_pallets(
            ax, trajectory[-1], colors, linewidth, add_label=True
        )
        
        # 绘制叉车姿态
        sample_step = self._get_config_value('visualization', 'drawing', 'sample_step', default=50)
        step = max(1, len(trajectory) // sample_step) if sample_step else 50
        
        for i in range(0, len(trajectory), step):
            self._draw_forklift_shape(ax, trajectory[i], 
                                     alpha=0.3 if i < len(trajectory) - 1 else 1.0,
                                     colors=colors, linewidth=linewidth)
            self._draw_direction_arrow(ax, trajectory[i], 
                                      alpha=0.3 if i < len(trajectory) - 1 else 1.0,
                                      static=True)
        
        # 设置坐标轴（使用所有托盘的角点）
        x_min, x_max, y_min, y_max = self._calculate_axis_limits(trajectory, all_pallet_corners)
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        
        ax.set_xlabel('X 位置 (m)', fontsize=12)
        ax.set_ylabel('Y 位置 (m)', fontsize=12)
        ax.set_title('无人叉车轨迹仿真', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='box')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"静态轨迹图已保存到: {save_path}")
        else:
            print("警告: 未指定保存路径，跳过保存")
        
        plt.close(fig)
    
    def _draw_forklift_shape(self, ax, state: RobotState, alpha: float = 1.0,
                            colors: Optional[dict] = None, linewidth: Optional[dict] = None):
        """
        在给定的axes上绘制叉车形状
        
        Args:
            ax: matplotlib axes
            state: 叉车状态
            alpha: 透明度
            colors: 颜色配置
            linewidth: 线宽配置
        """
        if colors is None:
            colors = {}
        if linewidth is None:
            linewidth = {}
        
        body_corners, left_wheel_corners, right_wheel_corners = self.get_body_shape(state)
        
        # 绘制车身
        body_color = colors.get('body', 'steelblue')
        body_closed = np.vstack([body_corners, body_corners[0]])
        ax.plot(body_closed[:, 0], body_closed[:, 1], 'k-', alpha=alpha, 
               linewidth=linewidth.get('body', 1.5))
        ax.fill(body_closed[:, 0], body_closed[:, 1], color=body_color, alpha=alpha*0.4)
        
        # 绘制左后轮
        wheel_color = colors.get('wheel', 'black')
        left_wheel_closed = np.vstack([left_wheel_corners, left_wheel_corners[0]])
        ax.plot(left_wheel_closed[:, 0], left_wheel_closed[:, 1], 'k-', alpha=alpha, 
               linewidth=linewidth.get('wheel', 1.5))
        ax.fill(left_wheel_closed[:, 0], left_wheel_closed[:, 1], color=wheel_color, alpha=alpha*0.8)
        
        # 绘制右后轮
        right_wheel_closed = np.vstack([right_wheel_corners, right_wheel_corners[0]])
        ax.plot(right_wheel_closed[:, 0], right_wheel_closed[:, 1], 'k-', alpha=alpha, 
               linewidth=linewidth.get('wheel', 1.5))
        ax.fill(right_wheel_closed[:, 0], right_wheel_closed[:, 1], color=wheel_color, alpha=alpha*0.8)
    
    def _draw_direction_arrow(self, ax, state: RobotState, alpha: float = 1.0, static: bool = False):
        """
        绘制方向箭头
        
        Args:
            ax: matplotlib axes
            state: 叉车状态
            alpha: 透明度
            static: 是否为静态图（影响箭头长度）
        """
        x, y, theta = state.x, state.y, state.theta
        
        if static:
            arrow_length = self._get_config_value('arrow', 'length_static', 
                                                 default=self.simulator.params.length * 0.3)
        else:
            arrow_length = self.simulator.params.length * 0.5
        
        arrow_head_width = self._get_config_value('arrow', 'head_width', default=0.1)
        arrow_head_length = self._get_config_value('arrow', 'head_length', default=0.1)
        
        dx = arrow_length * np.cos(theta)
        dy = arrow_length * np.sin(theta)
        ax.arrow(x, y, dx, dy, head_width=arrow_head_width, head_length=arrow_head_length, 
                fc='red', ec='red', alpha=alpha)
    
    def visualize_animated(self, save_path: Optional[str] = None, interval: int = 50, 
                          sample_step: int = 8, dpi: int = 100):
        """
        动态可视化叉车运动
        
        Args:
            save_path: 保存路径，如果为None则不保存
            interval: 帧间隔 (ms)
            sample_step: 轨迹采样步长，越大越快（默认8，推荐4-16）
            dpi: 输出分辨率，越低越快（默认100，推荐80-150）
        """
        trajectory = self.simulator.get_trajectory_sample(step=sample_step)

        # 🚀 优化1: 减小图形尺寸
        figsize_config = self._get_config_value('visualization', 'figure', 'animated_width', default=12), \
                        self._get_config_value('visualization', 'figure', 'animated_height', default=10)
        fig, ax = plt.subplots(figsize=figsize_config)
        
        # 获取配置参数
        colors = self._get_config_value('visualization', 'colors', 
                                       default={'trajectory': 'b', 'body': 'steelblue', 
                                               'wheel': 'black', 'steering_wheel': 'orange'}) or {'trajectory': 'b', 'body': 'steelblue', 'wheel': 'black', 'steering_wheel': 'orange'}
        linewidth = self._get_config_value('visualization', 'linewidth',
                                          default={'trajectory': 2, 'body': 2, 'wheel': 2, 'pallet': 3}) or {'trajectory': 2, 'body': 2, 'wheel': 2, 'pallet': 3}
        
        # 计算托盘和坐标轴范围（使用封装的函数获取所有托盘）
        # 先在临时figure上绘制以获取所有托盘角点
        temp_fig, temp_ax = plt.subplots()
        all_pallet_corners = self._draw_multiple_pallets(temp_ax, trajectory[-1], colors, linewidth, add_label=False)
        plt.close(temp_fig)
        
        # 计算动画的坐标轴范围（考虑舵轮）
        all_x, all_y = self._calculate_animated_bounds(trajectory)
        for corners in all_pallet_corners:
            all_x.extend(corners[:, 0])
            all_y.extend(corners[:, 1])
        
        vehicle_size = max(self.simulator.params.length, self.simulator.params.width)
        margin = vehicle_size * 1.0
        x_min = min(all_x) - margin
        x_max = max(all_x) + margin
        y_min = min(all_y) - margin
        y_max = max(all_y) + margin
        
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        
        # 初始化绘图元素
        trail_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.6, label='轨迹')
        forklift_body, = ax.plot([], [], 'k-', linewidth=1.5, label='叉车')
        forklift_body_fill = ax.fill([], [], color='steelblue', alpha=0.4)[0]
        forklift_left_wheel, = ax.plot([], [], 'k-', linewidth=1.5)
        forklift_left_wheel_fill = ax.fill([], [], color='black', alpha=0.8)[0]
        forklift_right_wheel, = ax.plot([], [], 'k-', linewidth=1.5)
        forklift_right_wheel_fill = ax.fill([], [], color='black', alpha=0.8)[0]
        forklift_direction, = ax.plot([], [], 'r-', linewidth=2, label='方向')
        current_pos, = ax.plot([], [], 'ro', markersize=8)
        
        # 舵轮绘图元素
        wheel_width = self.simulator.params.width * 0.1
        wheel_length = self.simulator.params.width * 0.2
        steering_wheel_ellipse = Ellipse((0, 0), wheel_length, wheel_width, 
                                        angle=0, color='purple', fill=False, 
                                        linewidth=2, visible=False)
        ax.add_patch(steering_wheel_ellipse)
        steering_wheel_line, = ax.plot([], [], 'c-', linewidth=2, label='舵轮')
        
        # 时间文本
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                           verticalalignment='top', fontsize=12,
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # 绘制起点、终点和多个托盘（使用封装的函数）
        ax.plot(trajectory[0].x, trajectory[0].y, 'go', markersize=10, label='起点')
        ax.plot(trajectory[-1].x, trajectory[-1].y, 'rs', markersize=10, label='终点')
        self._draw_multiple_pallets(ax, trajectory[-1], colors, linewidth, add_label=True)
        
        ax.set_xlabel('X 位置 (m)', fontsize=12)
        ax.set_ylabel('Y 位置 (m)', fontsize=12)
        ax.set_title('无人叉车动态仿真', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='box')
        
        def init():
            trail_line.set_data([], [])
            forklift_body.set_data([], [])
            forklift_body_fill.set_xy(np.empty((0, 2)))
            forklift_left_wheel.set_data([], [])
            forklift_left_wheel_fill.set_xy(np.empty((0, 2)))
            forklift_right_wheel.set_data([], [])
            forklift_right_wheel_fill.set_xy(np.empty((0, 2)))
            forklift_direction.set_data([], [])
            current_pos.set_data([], [])
            steering_wheel_ellipse.set_visible(False)
            steering_wheel_line.set_data([], [])
            time_text.set_text('')
            return (trail_line, forklift_body, forklift_body_fill, 
                   forklift_left_wheel, forklift_left_wheel_fill,
                   forklift_right_wheel, forklift_right_wheel_fill,
                   forklift_direction, current_pos, 
                   steering_wheel_ellipse, steering_wheel_line, time_text)
        
        def animate(frame):
            # 更新轨迹
            trail_line.set_data([traj.x for traj in trajectory[:frame+1]], [traj.y for traj in trajectory[:frame+1]])

            # 更新叉车形状
            body_corners, left_wheel_corners, right_wheel_corners = self.get_body_shape(trajectory[frame])
            
            body_closed = np.vstack([body_corners, body_corners[0]])
            forklift_body.set_data(body_closed[:, 0], body_closed[:, 1])
            forklift_body_fill.set_xy(body_corners)
            
            left_wheel_closed = np.vstack([left_wheel_corners, left_wheel_corners[0]])
            forklift_left_wheel.set_data(left_wheel_closed[:, 0], left_wheel_closed[:, 1])
            forklift_left_wheel_fill.set_xy(left_wheel_corners)
            
            right_wheel_closed = np.vstack([right_wheel_corners, right_wheel_corners[0]])
            forklift_right_wheel.set_data(right_wheel_closed[:, 0], right_wheel_closed[:, 1])
            forklift_right_wheel_fill.set_xy(right_wheel_corners)
            
            # 更新方向箭头
            x, y, theta = trajectory[frame].x, trajectory[frame].y, trajectory[frame].theta
            arrow_length = self.simulator.params.length * 0.5
            direction_x = [x, x + arrow_length * np.cos(theta)]
            direction_y = [y, y + arrow_length * np.sin(theta)]
            forklift_direction.set_data(direction_x, direction_y)
            current_pos.set_data([x], [y])
            
            # 更新舵轮
            steer_x, steer_y, steer_direction = self.get_steering_wheel_position(trajectory[frame])
            
            steering_wheel_ellipse.center = (steer_x, steer_y)
            steering_wheel_ellipse.angle = np.rad2deg(steer_direction)
            steering_wheel_ellipse.set_visible(True)
            
            wheel_line_length = wheel_length * 0.6
            steer_dx = wheel_line_length * np.cos(steer_direction)
            steer_dy = wheel_line_length * np.sin(steer_direction)
            steering_wheel_line.set_data([steer_x - steer_dx, steer_x + steer_dx],
                                        [steer_y - steer_dy, steer_y + steer_dy])
            
            # 更新时间文本
            time_text.set_text(f'时间: {trajectory[frame].time:.2f} s\n'
                             f'位置: ({x:.2f}, {y:.2f}) m\n'
                             f'角度: {np.rad2deg(theta):.1f}°')
            
            return (trail_line, forklift_body, forklift_body_fill,
                   forklift_left_wheel, forklift_left_wheel_fill,
                   forklift_right_wheel, forklift_right_wheel_fill,
                   forklift_direction, current_pos, 
                   steering_wheel_ellipse, steering_wheel_line, time_text)
        
        anim = FuncAnimation(fig, animate, init_func=init, 
                            frames=len(trajectory), interval=interval,
                            blit=True, repeat=True)
        
        if save_path:
            print(f"正在保存动画到: {save_path}")
            print(f"  帧数: {len(trajectory)}, DPI: {dpi}")
            
            try:
                # 🚀 优化2: 根据文件扩展名选择更快的writer
                if save_path.endswith('.gif'):
                    # GIF: 使用 pillow，但降低质量加速
                    anim.save(save_path, writer='pillow', fps=20, dpi=dpi)
                elif save_path.endswith('.mp4'):
                    # MP4: 更快且文件更小（需要 ffmpeg）
                    try:
                        anim.save(save_path, writer='ffmpeg', fps=30, dpi=dpi, 
                                 bitrate=1800, codec='h264')
                    except Exception:
                        print("  警告: ffmpeg 不可用，回退到 pillow")
                        anim.save(save_path, writer='pillow', fps=20, dpi=dpi)
                else:
                    # 默认使用 pillow
                    anim.save(save_path, writer='pillow', fps=20, dpi=dpi)
                
                print("动画保存完成!")
            except Exception as e:
                print(f"动画保存失败: {e}")
                import traceback
                traceback.print_exc()
        else:
            print("警告: 未指定保存路径，跳过动画保存")
        
        plt.close(fig)
        return anim
    
    def visualize_animated_fast(self, save_path: Optional[str] = None):
        """
        快速生成动画（低质量但速度快）
        
        适合快速预览和调试
        """
        return self.visualize_animated(
            save_path=save_path,
            interval=50,
            sample_step=16,  # 大步长采样
            dpi=80           # 低分辨率
        )
    
    def visualize_animated_high_quality(self, save_path: Optional[str] = None):
        """
        生成高质量动画（慢但质量好）
        
        适合最终展示
        """
        return self.visualize_animated(
            save_path=save_path,
            interval=30,
            sample_step=2,   # 小步长采样
            dpi=150          # 高分辨率
        )

    def _calculate_animated_bounds(self, trajectory: List[RobotState]) -> Tuple[List[float], List[float]]:
        """
        计算动画所需的边界（包括舵轮）
        
        Args:
            trajectory: 轨迹数组
            
        Returns:
            (all_x, all_y): x和y坐标列表
        """
        all_x = []
        all_y = []
        arrow_length = self.simulator.params.length * 0.5
        wheel_width = self.simulator.params.width * 0.1
        wheel_length = self.simulator.params.width * 0.2
        
        for state in trajectory:
            x, y, theta = state.x, state.y, state.theta

            # 叉车角点
            body_corners, left_wheel_corners, right_wheel_corners = self.get_body_shape(state)
            all_x.extend(body_corners[:, 0])
            all_y.extend(body_corners[:, 1])
            all_x.extend(left_wheel_corners[:, 0])
            all_y.extend(left_wheel_corners[:, 1])
            all_x.extend(right_wheel_corners[:, 0])
            all_y.extend(right_wheel_corners[:, 1])
            
            # 方向箭头终点
            arrow_end_x = x + arrow_length * np.cos(theta)
            arrow_end_y = y + arrow_length * np.sin(theta)
            all_x.append(arrow_end_x)
            all_y.append(arrow_end_y)
            
            # 舵轮位置
            steer_x, steer_y, _ = self.get_steering_wheel_position(state)
            all_x.extend([steer_x - wheel_length/2, steer_x + wheel_length/2])
            all_y.extend([steer_y - wheel_width/2, steer_y + wheel_width/2])
        
        return all_x, all_y


