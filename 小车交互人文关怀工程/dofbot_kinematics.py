#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from collections import namedtuple

# 定义常量
RA2DE = 180.0 / math.pi  # 弧度转角度
DE2RA = math.pi / 180.0  # 角度转弧度

class DofbotKinematics:
    """DOFbot 5自由度机械臂正反解计算类"""
    
    def __init__(self):
        # DOFbot 连杆参数 (单位: 米)
        self.a1 = 0.0      # Joint1 的 X 偏移
        self.a2 = -0.08285 # Joint3 的 X 偏移
        self.a3 = -0.08285 # Joint4 的 X 偏移
        self.a4 = -0.07385 # Joint5 的 X 偏移
        
        self.d1 = 0.064    # Joint1 的 Z 偏移
        self.d2 = 0.0435   # Joint2 的 Z 偏移
        self.d5 = -0.00215 # Joint5 的 Y 偏移
        
        # 特殊关节的旋转修正
        self.joint2_rpy = (0, math.pi/2, 0)    # Joint2: (0, 90°, 0)
        self.joint5_rpy = (0, -math.pi/2, 0)   # Joint5: (0, -90°, 0)
        
    def rotation_matrix_to_euler(self, R):
        """将旋转矩阵转换为欧拉角 (RPY)"""
        # 提取欧拉角 (ZYX顺序)
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2,1], R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
            
        return np.array([x, y, z])
    
    def dh_transform(self, a, d, alpha, theta):
        """DH参数变换矩阵"""
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_alpha = math.cos(alpha)
        sin_alpha = math.sin(alpha)
        
        T = np.array([
            [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
            [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])
        return T
    
    def dofbot_getFK(self, joint_angles):
        """
        正运动学计算
        joint_angles: 5个关节角度的弧度值列表
        返回: [x, y, z, roll, pitch, yaw]
        """
        # DH参数 (a, d, alpha, theta)
        dh_params = [
            (0, self.d1, 0, joint_angles[0]),           # Joint1
            (0, self.d2, math.pi/2, joint_angles[1]),   # Joint2 (alpha=90°)
            (self.a2, 0, 0, joint_angles[2]),           # Joint3
            (self.a3, 0, 0, joint_angles[3]),           # Joint4
            (self.a4, 0, -math.pi/2, joint_angles[4])   # Joint5 (alpha=-90°)
        ]
        
        # 计算变换矩阵
        T = np.eye(4)
        for params in dh_params:
            T_i = self.dh_transform(*params)
            T = np.dot(T, T_i)
        
        # 提取位置
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        
        # 提取姿态 (欧拉角)
        R = T[:3, :3]
        rpy = self.rotation_matrix_to_euler(R)
        
        return [x, y, z, rpy[0], rpy[1], rpy[2]]
    
    def dofbot_getIK(self, target_xyz, target_rpy):
        """
        逆运动学计算 (简化版本)
        target_xyz: [x, y, z] 目标位置
        target_rpy: [roll, pitch, yaw] 目标姿态
        返回: [joint1, joint2, joint3, joint4, joint5] 弧度值
        """
        x, y, z = target_xyz
        roll, pitch, yaw = target_rpy
        
        # 计算 Joint1 (基座旋转)
        joint1 = math.atan2(y, x)
        
        # 计算到目标点的水平距离
        r = math.sqrt(x*x + y*y)
        
        # 考虑 Joint1 的高度偏移
        z_eff = z - self.d1
        
        # 计算 Joint2 和 Joint3 (平面2R机械臂)
        # 这里是简化的几何解法
        L2 = abs(self.a2)  # 0.08285
        L3 = abs(self.a3)  # 0.08285
        L4 = abs(self.a4)  # 0.07385
        
        # 从末端到Joint3的距离 (考虑Joint5的偏移)
        r_eff = r - L4 * math.cos(joint1)
        z_eff2 = z_eff - L4 * math.sin(joint1) - self.d2
        
        # 2R机械臂逆解
        D = (r_eff*r_eff + z_eff2*z_eff2 - L2*L2 - L3*L3) / (2*L2*L3)
        
        if D > 1:
            D = 1
        elif D < -1:
            D = -1
            
        # 计算 Joint3
        joint3 = math.atan2(-math.sqrt(1 - D*D), D)
        
        # 计算 Joint2
        joint2 = math.atan2(z_eff2, r_eff) - math.atan2(L3*math.sin(joint3), L2 + L3*math.cos(joint3))
        
        # 简化处理 Joint4 和 Joint5
        joint4 = 0.0
        joint5 = yaw - joint1 - joint2 - joint3
        
        return [joint1, joint2, joint3, joint4, joint5]

# # 测试代码
# if __name__ == "__main__":
#     kinematics = DofbotKinematics()
    
#     # 测试正运动学
#     print("=== 正运动学测试 ===")
#     test_joints = [0, 0, 0, 0, 0]  # 弧度
#     result = kinematics.dofbot_getFK(test_joints)
#     print(f"关节角度: {[j*180/3.14159 for j in test_joints]}")
#     print(f"末端位置: X={result[0]:.4f}, Y={result[1]:.4f}, Z={result[2]:.4f}")
#     print(f"末端姿态: Roll={result[3]*180/3.14159:.2f}°, Pitch={result[4]*180/3.14159:.2f}°, Yaw={result[5]*180/3.14159:.2f}°")
    
#     # 测试逆运动学
#     print("\n=== 逆运动学测试 ===")
#     target_xyz = [0.1, 0.1, 0.15]
#     target_rpy = [0, 0, 0]
#     result = kinematics.dofbot_getIK(target_xyz, target_rpy)
#     print(f"目标位置: X={target_xyz[0]}, Y={target_xyz[1]}, Z={target_xyz[2]}")
#     print(f"计算关节角度: {[j*180/3.14159 for j in result]}")