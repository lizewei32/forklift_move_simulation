#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
贝塞尔曲线生成器
Bezier Curve Generator

根据给定的控制点坐标生成贝塞尔曲线路径点
"""

import numpy as np
from typing import List, Tuple, Sequence, Union
import json


class BezierCurveGenerator:
    """贝塞尔曲线生成器"""
    
    def __init__(self, control_points: Sequence[Tuple[float, float]]):
        """
        初始化贝塞尔曲线生成器
        
        Args:
            control_points: 控制点列表 [(x0, y0), (x1, y1), ..., (xn, yn)]
        """
        if len(control_points) < 2:
            raise ValueError("至少需要2个控制点")
        
        self.control_points = np.array(control_points)
        self.n = len(control_points) - 1  # 贝塞尔曲线的阶数
        self.bezier_points = []

    def _bernstein_poly(self, i: int, n: int, t: float) -> float:
        """
        计算伯恩斯坦基多项式
        
        Args:
            i: 控制点索引
            n: 贝塞尔曲线阶数
            t: 参数 (0 <= t <= 1)
        
        Returns:
            伯恩斯坦多项式值
        """
        from math import comb
        return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))
    
    def bezier_point(self, t: float) -> np.ndarray:
        """
        计算贝塞尔曲线上参数为t的点
        
        Args:
            t: 参数 (0 <= t <= 1)
        
        Returns:
            曲线上的点坐标 [x, y]
        """
        point = np.zeros(2)
        for i in range(self.n + 1):
            point += self._bernstein_poly(i, self.n, t) * self.control_points[i]
        return point
    
    def generate_curve(self, num_points: int = 100) -> np.ndarray:
        """
        生成贝塞尔曲线的离散点
        
        Args:
            num_points: 生成的点数量
        
        Returns:
            曲线点数组，形状为 (num_points, 2)
        """
        t_values = np.linspace(0, 1, num_points)
        curve_points = np.array([self.bezier_point(t) for t in t_values])
        return curve_points
    
    def generate_curve_with_derivatives(self, num_points: int = 100) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        生成贝塞尔曲线的离散点及其一阶、二阶导数（用于计算切线方向和曲率）
        
        Args:
            num_points: 生成的点数量
        
        Returns:
            (curve_points, first_derivatives, second_derivatives)
            - curve_points: 曲线点数组，形状为 (num_points, 2)
            - first_derivatives: 一阶导数数组，形状为 (num_points, 2)
            - second_derivatives: 二阶导数数组，形状为 (num_points, 2)
        """
        t_values = np.linspace(0, 1, num_points)
        curve_points = []
        first_derivatives = []
        second_derivatives = []
        
        for t in t_values:
            # 曲线点
            point = self.bezier_point(t)
            curve_points.append(point)
            
            # 一阶导数（切线方向）
            first_deriv = np.zeros(2)
            for i in range(self.n):
                first_deriv += self.n * (self.control_points[i + 1] - self.control_points[i]) * \
                              self._bernstein_poly(i, self.n - 1, t)
            first_derivatives.append(first_deriv)
            
            # 二阶导数（曲率相关）
            if self.n >= 2:
                second_deriv = np.zeros(2)
                for i in range(self.n - 1):
                    second_deriv += self.n * (self.n - 1) * \
                                   (self.control_points[i + 2] - 2 * self.control_points[i + 1] + self.control_points[i]) * \
                                   self._bernstein_poly(i, self.n - 2, t)
                second_derivatives.append(second_deriv)
            else:
                second_derivatives.append(np.zeros(2))
        
        return (np.array(curve_points), 
                np.array(first_derivatives), 
                np.array(second_derivatives))
    
    def get_tangent_angles(self, num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取贝塞尔曲线上各点的切线角度
        
        Args:
            num_points: 生成的点数量
        
        Returns:
            (curve_points, tangent_angles)
            - curve_points: 曲线点数组，形状为 (num_points, 2)
            - tangent_angles: 切线角度数组（弧度），形状为 (num_points,)
        """
        curve_points, first_derivatives, _ = self.generate_curve_with_derivatives(num_points)
        tangent_angles = np.arctan2(first_derivatives[:, 1], first_derivatives[:, 0])
        return curve_points, tangent_angles
    
    def get_curvatures(self, num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取贝塞尔曲线上各点的曲率
        
        Args:
            num_points: 生成的点数量
        
        Returns:
            (curve_points, curvatures)
            - curve_points: 曲线点数组，形状为 (num_points, 2)
            - curvatures: 曲率数组，形状为 (num_points,)
        """
        curve_points, first_deriv, second_deriv = self.generate_curve_with_derivatives(num_points)
        
        # 曲率公式: κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
        numerator = np.abs(first_deriv[:, 0] * second_deriv[:, 1] - 
                          first_deriv[:, 1] * second_deriv[:, 0])
        denominator = (first_deriv[:, 0]**2 + first_deriv[:, 1]**2)**(3/2)
        
        # 避免除零
        curvatures = np.where(denominator > 1e-10, numerator / denominator, 0)
        
        return curve_points, curvatures
    
    def save_to_file(self, filename: str, num_points: int = 100, include_derivatives: bool = False):
        """
        将贝塞尔曲线保存到JSON文件
        
        Args:
            filename: 输出文件名
            num_points: 生成的点数量
            include_derivatives: 是否包含导数和角度信息
        """
        if include_derivatives:
            curve_points, tangent_angles = self.get_tangent_angles(num_points)
            _, curvatures = self.get_curvatures(num_points)
            
            data = {
                "control_points": self.control_points.tolist(),
                "num_points": num_points,
                "curve_points": curve_points.tolist(),
                "tangent_angles": tangent_angles.tolist(),
                "curvatures": curvatures.tolist()
            }
        else:
            curve_points = self.generate_curve(num_points)
            data = {
                "control_points": self.control_points.tolist(),
                "num_points": num_points,
                "curve_points": curve_points.tolist()
            }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"贝塞尔曲线已保存到: {filename}")
        print(f"  控制点数量: {len(self.control_points)}")
        print(f"  曲线点数量: {num_points}")

    def visualize_bezier_curve(self, save_path: str = "bezier_curve.png"):
        """
        可视化贝塞尔曲线

        Args:
            save_path: 保存图片的路径
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("需要安装 matplotlib 才能可视化: pip install matplotlib")
            return
        num_points = 100
        curve_points, tangent_angles = self.get_tangent_angles(num_points)
        _, curvatures = self.get_curvatures(num_points)

        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        
        # 图1: 贝塞尔曲线和控制点
        ax1 = axes[0, 0]
        ax1.plot(curve_points[:, 0], curve_points[:, 1], 'b-', linewidth=2, label='贝塞尔曲线')
        ax1.plot(self.control_points[:, 0], self.control_points[:, 1], 
                'ro--', markersize=10, linewidth=1, label='控制点')
        for i, (x, y) in enumerate(self.control_points):
            ax1.annotate(f'P{i}', (x, y), xytext=(5, 5), textcoords='offset points', fontsize=12)
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title('贝塞尔曲线与控制点')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal', adjustable='box')
        
        # 图2: 曲线上的切线方向
        ax2 = axes[0, 1]
        ax2.plot(curve_points[:, 0], curve_points[:, 1], 'b-', linewidth=2, label='曲线')
        # 每隔几个点绘制一个箭头
        step = max(1, num_points // 20)
        for i in range(0, num_points, step):
            dx = 0.5 * np.cos(tangent_angles[i])
            dy = 0.5 * np.sin(tangent_angles[i])
            ax2.arrow(curve_points[i, 0], curve_points[i, 1], dx, dy,
                    head_width=0.2, head_length=0.15, fc='red', ec='red', alpha=0.6)
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_title('曲线切线方向')
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal', adjustable='box')
        
        # 图3: 切线角度变化
        ax3 = axes[1, 0]
        t_values = np.linspace(0, 1, num_points)
        ax3.plot(t_values, np.rad2deg(tangent_angles), 'g-', linewidth=2)
        ax3.set_xlabel('参数 t')
        ax3.set_ylabel('切线角度 (度)')
        ax3.set_title('切线角度沿曲线的变化')
        ax3.grid(True, alpha=0.3)
        
        # 图4: 曲率变化
        ax4 = axes[1, 1]
        ax4.plot(t_values, curvatures, 'm-', linewidth=2)
        ax4.set_xlabel('参数 t')
        ax4.set_ylabel('曲率')
        ax4.set_title('曲率沿曲线的变化')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n可视化图已保存到: {save_path}")
        plt.close()


def demo_bezier_curve():
    """演示贝塞尔曲线生成"""
    
    # 示例1: 二次贝塞尔曲线（3个控制点）
    print("=" * 60)
    print("示例1: 二次贝塞尔曲线")
    print("=" * 60)
    control_points_quadratic = [
        (0, 0),
        (5, 10),
        (10, 0)
    ]
    
    bezier_quad = BezierCurveGenerator(control_points_quadratic)
    bezier_quad.save_to_file("bezier_quadratic.json", num_points=50, include_derivatives=True)
    
    # 示例2: 三次贝塞尔曲线（4个控制点）
    print("\n" + "=" * 60)
    print("示例2: 三次贝塞尔曲线")
    print("=" * 60)
    control_points_cubic = [
        (0, 0),
        (3, 8),
        (7, 8),
        (10, 0)
    ]
    
    bezier_cubic = BezierCurveGenerator(control_points_cubic)
    bezier_cubic.save_to_file("bezier_cubic.json", num_points=100, include_derivatives=True)
    
    # 示例3: 高阶贝塞尔曲线（5个控制点）
    print("\n" + "=" * 60)
    print("示例3: 五个控制点的高阶贝塞尔曲线")
    print("=" * 60)
    control_points_high_order = [
        (0, 0),
        (2, 5),
        (5, 6),
        (8, 4),
        (10, 0)
    ]
    
    bezier_high = BezierCurveGenerator(control_points_high_order)
    bezier_high.save_to_file("bezier_high_order.json", num_points=100, include_derivatives=True)
    
    # 示例4: 叉车路径示例（S型曲线）
    print("\n" + "=" * 60)
    print("示例4: 叉车S型路径")
    print("=" * 60)
    control_points_forklift = [
        (0, 0),
        (2, 1),
        (4, 3),
        (6, 5),
        (8, 6)
    ]
    
    bezier_forklift = BezierCurveGenerator(control_points_forklift)
    bezier_forklift.save_to_file("bezier_forklift_path.json", num_points=200, include_derivatives=True)
    
    print("\n" + "=" * 60)
    print("所有示例曲线已生成！")
    print("=" * 60)

