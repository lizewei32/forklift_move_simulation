import numpy as np

class PIDController:
    """PID控制器"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        """
        初始化PID控制器
        
        Args:
            kp: 比例增益
            ki: 积分增益
            kd: 微分增益
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
    
    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.last_error = 0.0
    
    def compute(self, error: float, dt: float) -> float:
        """
        计算PID输出
        
        Args:
            error: 当前误差
            dt: 时间步长
            
        Returns:
            控制输出
        """
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output
