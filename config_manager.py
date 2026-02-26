#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置管理模块 / Configuration Management Module
加载和管理YAML配置文件，提供分类的配置访问接口
"""

import yaml
import os
from dataclasses import dataclass
from typing import Dict, Any, Tuple


# ============================================================================
# 配置数据类 / Configuration Data Classes
# ============================================================================

@dataclass
class ForkliftConfig:
    """叉车物理参数配置"""
    length: float          # 车体长度 (m)
    width: float           # 车体宽度 (m)
    steer_dist: float      # 舵轮距离车体中心 (m)
    track_width: float     # 左右轮距 (m)
    path_width: float      # 叉车路径宽度 (m)


@dataclass
class SimulationConfig:
    """仿真参数配置"""
    dt: float              # 仿真时间步长 (s)


@dataclass
class PalletConfig:
    """托盘配置（计算后的实际值）"""
    length: float          # 托盘长度 (m)
    width: float           # 托盘宽度 (m)
    offset: float          # 托盘位置偏移 (m)
    
    def get_center_position(self, end_x: float, end_y: float, end_theta: float) -> Tuple[float, float]:
        """
        计算托盘中心位置
        
        Args:
            end_x: 终点X坐标
            end_y: 终点Y坐标
            end_theta: 终点朝向角度
        
        Returns:
            (center_x, center_y) 托盘中心坐标
        """
        import numpy as np
        center_x = end_x + (self.length / 2 + self.offset) * np.cos(end_theta)
        center_y = end_y + (self.length / 2 + self.offset) * np.sin(end_theta)
        return center_x, center_y


@dataclass
class SteeringWheelConfig:
    """舵轮配置（计算后的实际值）"""
    width: float           # 椭圆短轴 (m)
    length: float          # 椭圆长轴 (m)
    
    def get_line_length(self) -> float:
        """获取舵轮中心线长度"""
        return self.length * 0.6


@dataclass
class BodyConfig:
    """车身配置（计算后的实际值）"""
    length: float          # 车身长度 (m)
    width: float           # 车身宽度 (m)
    front_position: float  # 车身前端位置 (m)
    back_position: float   # 车身后端位置 (m)


@dataclass
class WheelConfig:
    """后轮配置（计算后的实际值）"""
    length: float          # 后轮长度 (m)
    width: float           # 后轮宽度 (m)
    gap: float             # 两个后轮之间的间距 (m)
    back_position: float   # 后轮后端位置 (m)
    front_position: float  # 后轮前端位置 (m)


@dataclass
class ArrowConfig:
    """箭头配置（计算后的实际值）"""
    length_static: float   # 静态图箭头长度 (m)
    length_animated: float # 动画箭头长度 (m)
    head_width: float      # 箭头头部宽度 (m)
    head_length: float     # 箭头头部长度 (m)


@dataclass
class VisualizationConfig:
    """可视化配置"""
    # 计算后的配置
    pallet: PalletConfig
    steering_wheel: SteeringWheelConfig
    body: BodyConfig
    wheel: WheelConfig
    arrow: ArrowConfig
    
    # 原始配置
    colors: Dict[str, Any]
    linewidth: Dict[str, float]
    figure: Dict[str, Any]
    drawing: Dict[str, Any]
    animation: Dict[str, Any]
    markers: Dict[str, Any]
    text: Dict[str, Any]
    labels: Dict[str, Any]


# ============================================================================
# 主配置管理类 / Main Configuration Manager
# ============================================================================

class Config:
    """配置管理类 - 提供分类的配置访问接口"""
    
    def __init__(self, config_path: str = 'config.yaml'):
        """
        初始化配置管理器
        
        Args:
            config_path: YAML配置文件路径
        """
        self.config_path = config_path
        self._raw_config = self._load_config()
        
        # 解析并分类配置
        self._forklift_config = self._parse_forklift_config()
        self._simulation_config = self._parse_simulation_config()
        self._visualization_config = self._parse_visualization_config()
        self._output_config = self._raw_config.get('output', {})
        self._path_generation_config = self._raw_config.get('path_generation', {})

    def _load_config(self) -> Dict[str, Any]:
        """加载YAML配置文件"""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"配置文件不存在: {self.config_path}")
        
        with open(self.config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        return config
    
    def _parse_forklift_config(self) -> ForkliftConfig:
        """解析叉车配置"""
        forklift = self._raw_config.get('forklift', {})
        return ForkliftConfig(
            length=forklift.get('length', 2.6),
            width=forklift.get('width', 1.6),
            steer_dist=forklift.get('steer_dist', 2.1),
            track_width=forklift.get('track_width', 0.7),
            path_width=forklift.get('path_width', 3.0)
        )
    
    def _parse_simulation_config(self) -> SimulationConfig:
        """解析仿真配置"""
        simulation = self._raw_config.get('simulation', {})
        return SimulationConfig(
            dt=simulation.get('dt', 0.05)
        )
    
    def _parse_visualization_config(self) -> VisualizationConfig:
        """解析可视化配置并计算所有参数"""
        viz = self._raw_config.get('visualization', {})
        
        # 叉车物理参数
        forklift_length = self._forklift_config.length
        forklift_width = self._forklift_config.width
        
        # 托盘配置
        pallet_cfg = viz.get('pallet', {})
        pallet = PalletConfig(
            length=forklift_length * pallet_cfg.get('length_ratio', 0.7),
            width=pallet_cfg.get('width', 1.8),
            offset=forklift_length * pallet_cfg.get('offset_ratio', -0.1)
        )
        
        # 舵轮配置
        steering_cfg = viz.get('steering_wheel', {})
        steering_wheel = SteeringWheelConfig(
            width=forklift_width * steering_cfg.get('width_ratio', 0.1),
            length=forklift_width * steering_cfg.get('length_ratio', 0.2)
        )
        
        # 车身配置
        body_cfg = viz.get('body', {})
        body_length = forklift_length * body_cfg.get('length_ratio', 0.3)
        body_front_pos = forklift_length * body_cfg.get('front_position', 0.9)
        body = BodyConfig(
            length=body_length,
            width=forklift_width,
            front_position=body_front_pos,
            back_position=body_front_pos - body_length
        )
        
        # 后轮配置
        wheel_cfg = viz.get('wheel', {})
        wheel_length = forklift_length * wheel_cfg.get('length_ratio', 0.7)
        wheel_back_pos = forklift_length * wheel_cfg.get('back_position', -0.1)
        wheel = WheelConfig(
            length=wheel_length,
            width=forklift_width * wheel_cfg.get('width_ratio', 0.2),
            gap=forklift_width * wheel_cfg.get('gap_ratio', 0.3),
            back_position=wheel_back_pos,
            front_position=wheel_back_pos + wheel_length
        )
        
        # 箭头配置
        arrow_cfg = viz.get('arrow', {})
        arrow = ArrowConfig(
            length_static=forklift_length * arrow_cfg.get('length_ratio_static', 0.4),
            length_animated=forklift_length * arrow_cfg.get('length_ratio_animated', 0.3),
            head_width=arrow_cfg.get('head_width', 0.3),
            head_length=arrow_cfg.get('head_length', 0.2)
        )
        
        return VisualizationConfig(
            pallet=pallet,
            steering_wheel=steering_wheel,
            body=body,
            wheel=wheel,
            arrow=arrow,
            colors=viz.get('colors', {}),
            linewidth=viz.get('linewidth', {}),
            figure=viz.get('figure', {}),
            drawing=viz.get('drawing', {}),
            animation=viz.get('animation', {}),
            markers=viz.get('markers', {}),
            text=viz.get('text', {}),
            labels=viz.get('labels', {})
        )
    
    def get(self, *keys, default=None):
        """
        获取配置值（支持嵌套键）- 用于访问原始YAML值
        
        Args:
            *keys: 配置键路径，例如 get('visualization', 'colors', 'body')
            default: 默认值
        
        Returns:
            配置值
        
        Example:
            >>> config = Config()
            >>> config.get('forklift', 'length')
            2.6
        """
        value = self._raw_config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value
    
    def set(self, *keys, value):
        """
        设置配置值（支持嵌套键）
        
        Args:
            *keys: 配置键路径
            value: 要设置的值
        
        Example:
            >>> config = Config()
            >>> config.set('forklift', 'length', value=3.0)
        """
        current = self._raw_config
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        current[keys[-1]] = value
        
        # 重新解析配置
        self._forklift_config = self._parse_forklift_config()
        self._simulation_config = self._parse_simulation_config()
        self._visualization_config = self._parse_visualization_config()
    
    def save(self, save_path: str | None = None):
        """
        保存配置到文件
        
        Args:
            save_path: 保存路径，默认覆盖原文件
        """
        path = save_path or self.config_path
        with open(path, 'w', encoding='utf-8') as f:
            yaml.dump(self._raw_config, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    
    # ========================================================================
    # 便捷访问属性 - 返回分类的配置对象
    # ========================================================================
    
    @property
    def forklift(self) -> ForkliftConfig:
        """叉车参数配置"""
        return self._forklift_config
    
    @property
    def simulation(self) -> SimulationConfig:
        """仿真参数配置"""
        return self._simulation_config
    
    @property
    def visualization(self) -> VisualizationConfig:
        """可视化参数配置（包含所有计算后的子配置）"""
        return self._visualization_config
    
    @property
    def pallet(self) -> PalletConfig:
        """托盘配置（计算后的实际值）"""
        return self._visualization_config.pallet
    
    @property
    def steering_wheel(self) -> SteeringWheelConfig:
        """舵轮配置（计算后的实际值）"""
        return self._visualization_config.steering_wheel
    
    @property
    def body(self) -> BodyConfig:
        """车身配置（计算后的实际值）"""
        return self._visualization_config.body
    
    @property
    def wheel(self) -> WheelConfig:
        """后轮配置（计算后的实际值）"""
        return self._visualization_config.wheel
    
    @property
    def arrow(self) -> ArrowConfig:
        """箭头配置（计算后的实际值）"""
        return self._visualization_config.arrow
    
    @property
    def output(self):
        """输出参数"""
        return self._output_config
    
    @property
    def path_generation(self):
        """路径生成参数"""
        return self._path_generation_config
    
    # ========================================================================
    # 向后兼容的计算方法（推荐使用上面的属性访问）
    # ========================================================================
    
    def get_pallet_length(self) -> float:
        """计算托盘长度（推荐使用 config.pallet.length）"""
        return self.pallet.length
    
    def get_pallet_width(self) -> float:
        """获取托盘宽度（推荐使用 config.pallet.width）"""
        return self.pallet.width
    
    def get_pallet_offset(self) -> float:
        """计算托盘位置偏移（推荐使用 config.pallet.offset）"""
        return self.pallet.offset
    
    def get_body_length(self) -> float:
        """计算车身长度（推荐使用 config.body.length）"""
        return self.body.length
    
    def get_body_front_position(self) -> float:
        """计算车身前端位置（推荐使用 config.body.front_position）"""
        return self.body.front_position
    
    def get_wheel_length(self) -> float:
        """计算后轮长度（推荐使用 config.wheel.length）"""
        return self.wheel.length
    
    def get_wheel_width(self) -> float:
        """计算后轮宽度（推荐使用 config.wheel.width）"""
        return self.wheel.width
    
    def get_wheel_gap(self) -> float:
        """计算后轮间距（推荐使用 config.wheel.gap）"""
        return self.wheel.gap
    
    def get_wheel_back_position(self) -> float:
        """计算后轮后端位置（推荐使用 config.wheel.back_position）"""
        return self.wheel.back_position
    
    def get_steering_wheel_width(self) -> float:
        """计算舵轮椭圆短轴（推荐使用 config.steering_wheel.width）"""
        return self.steering_wheel.width
    
    def get_steering_wheel_length(self) -> float:
        """计算舵轮椭圆长轴（推荐使用 config.steering_wheel.length）"""
        return self.steering_wheel.length
    
    def get_arrow_length_static(self) -> float:
        """计算静态图箭头长度（推荐使用 config.arrow.length_static）"""
        return self.arrow.length_static
    
    def get_arrow_length_animated(self) -> float:
        """计算动画箭头长度（推荐使用 config.arrow.length_animated）"""
        return self.arrow.length_animated
    
    def __repr__(self):
        """字符串表示"""
        return f"Config(config_path='{self.config_path}')"


# 全局配置实例
_global_config = None


def get_config(config_path: str = 'config.yaml') -> Config:
    """
    获取全局配置实例（单例模式）
    
    Args:
        config_path: 配置文件路径
    
    Returns:
        Config实例
    """
    global _global_config
    if _global_config is None:
        _global_config = Config(config_path)
    return _global_config


def reload_config(config_path: str = 'config.yaml') -> Config:
    """
    重新加载配置
    
    Args:
        config_path: 配置文件路径
    
    Returns:
        新的Config实例
    """
    global _global_config
    _global_config = Config(config_path)
    return _global_config


if __name__ == '__main__':
    # 测试配置加载
    config = Config()
    
    print("=" * 60)
    print("配置文件测试")
    print("=" * 60)
    
    print("\n叉车参数:")
    print(f"  车长: {config.get('forklift', 'length')} m")
    print(f"  车宽: {config.get('forklift', 'width')} m")
    print(f"  舵轮距离: {config.get('forklift', 'steer_dist')} m")
    
    print("\n计算属性:")
    print(f"  托盘长度: {config.get_pallet_length():.2f} m")
    print(f"  托盘宽度: {config.get_pallet_width():.2f} m")
    print(f"  车身长度: {config.get_body_length():.2f} m")
    print(f"  后轮长度: {config.get_wheel_length():.2f} m")
    
    print("\n颜色配置:")
    print(f"  轨迹颜色: {config.get('visualization', 'colors', 'trajectory')}")
    print(f"  车身颜色: {config.get('visualization', 'colors', 'body')}")
    
    print("\n" + "=" * 60)
