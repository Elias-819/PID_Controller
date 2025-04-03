import numpy as np
import time
from src.PID_controller import PIDController

# Global variables for state estimation using fixed dt and filtering
last_pos = None  # 上一次位置（用于计算速度和加速度）
last_filtered_vel = None  # 上一次过滤后的速度（用于加速度滤波）
last_filtered_acc = None  # 上一次过滤后的加速度（用于下一次加速度计算）
stable_counter = 0  # 用于计数稳定步骤的变量

# Filtering coefficients (滤波系数，用于平滑速度和加速度)
alpha_vel = 0.3  # 速度滤波系数
alpha_acc = 0.3  # 加速度滤波系数

# Tuned PID gains for smoother convergence and less overshoot
pid_pos = PIDController(1.0, 0.05, 0.01, [0.5, 0.5, 0.5])  # 位置控制PID参数
pid_vel = PIDController(0.4, 0.01, 0.02, [0.3, 0.3, 0.3])  # 速度控制PID参数
pid_acc = PIDController(0.3, 0.0, 0.005, [0.1, 0.1, 0.1])  # 加速度控制PID参数
pid_yaw = PIDController(1.2, 0.0, 0.05, [0.5, 0.5, 0.5])  # 航向控制PID参数

# Thresholds for stability detection (稳定性检测的阈值)
pos_threshold = 0.05  # 位置误差阈值（单位：米）
yaw_threshold = 0.05  # 航向误差阈值（单位：弧度）
stabilize_zone = 0.08  # 稳定区域半径（米）
freeze_zone = 0.025  # 冻结区域半径（米）
STABLE_REQUIRED_STEPS = 15  # 需要达到稳定状态的步骤数

def controller(state, target, dt):
    """
    控制器函数，用于根据当前状态和目标计算控制命令。

    :param state: 当前状态 [x, y, z, roll, pitch, yaw]
    :param target: 目标位置和航向 [x_d, y_d, z_d, yaw_d]
    :param dt: 控制周期（时间间隔）
    :return: 返回速度控制命令 (vx, vy, vz, yaw_rate)
    """
    global last_pos, last_filtered_vel, last_filtered_acc, stable_counter

    # 解析当前状态和目标
    pos = np.array(state[0:3])  # 当前无人机位置 (x, y, z)
    yaw = state[5]  # 当前无人机航向
    target_pos = np.array(target[0:3])  # 目标位置 (x_d, y_d, z_d)
    target_yaw = target[3]  # 目标航向 yaw_d

    # 如果这是第一次调用控制器，初始化速度和加速度
    if last_pos is None:
        raw_vel = np.zeros(3)
        filtered_vel = np.zeros(3)
        filtered_acc = np.zeros(3)
    else:
        # 计算当前速度和加速度
        raw_vel = (pos - last_pos) / dt  # 原始速度（位置差/时间差）
        filtered_vel = alpha_vel * raw_vel + (1 - alpha_vel) * last_filtered_vel  # 过滤后的速度
        raw_acc = (filtered_vel - last_filtered_vel) / dt  # 原始加速度（速度差/时间差）
        filtered_acc = alpha_acc * raw_acc + (1 - alpha_acc) * last_filtered_acc  # 过滤后的加速度

    # 计算位置误差和航向误差
    pos_error = target_pos - pos  # 位置误差
    yaw_error = np.arctan2(np.sin(target_yaw - yaw), np.cos(target_yaw - yaw))  # 航向误差（归一化到[-π, π]）

    # 检查是否已达到稳定状态（位置和航向误差小于设定阈值）
    if (np.linalg.norm(pos_error) < freeze_zone and abs(yaw_error) < yaw_threshold and np.linalg.norm(filtered_vel) < 0.05 and np.linalg.norm(filtered_acc) < 0.05):
        stable_counter += 1  # 稳定计数器增加
    else:
        stable_counter = 0  # 如果位置误差较大，重置稳定计数器

    # 如果连续多次稳定，认为无人机已经到达目标，返回零控制命令
    if stable_counter >= STABLE_REQUIRED_STEPS:
        return (0.0, 0.0, 0.0, 0.0)  # 停止控制命令

    # 使用PID控制器计算位置、速度和加速度的控制命令
    u_pos = pid_pos.control_update(pos_error, dt)  # 位置控制输出
    vel_error = u_pos - filtered_vel  # 计算速度误差
    u_vel = pid_vel.control_update(vel_error, dt)  # 速度控制输出
    acc_error = u_vel - filtered_acc  # 计算加速度误差
    u_acc = pid_acc.control_update(acc_error, dt)  # 加速度控制输出

    # 计算最终的速度命令，将位置控制和加速度控制结合
    v_command = u_pos + 0.3 * u_acc
    v_command = np.tanh(v_command)  # 非线性压缩，减少震荡

    # 如果位置误差小于稳定区域，则减少控制命令的幅度，以避免过度调节
    if np.linalg.norm(pos_error) < stabilize_zone:
        v_command *= 0.2

    # 将命令从世界坐标系转换到无人机的机体坐标系
    R = np.array([[np.cos(yaw), np.sin(yaw)],
                  [-np.sin(yaw), np.cos(yaw)]])  # 旋转矩阵，用于将速度命令从世界坐标系转换到机体坐标系
    v_xy_body = R @ v_command[0:2]  # 在机体坐标系下的速度（x, y）
    v_command_transformed = np.array([v_xy_body[0], v_xy_body[1], v_command[2]])  # 完整的速度命令

    # 计算航向控制命令
    yaw_rate_output = pid_yaw.control_update(np.array([yaw_error, 0, 0]), dt)  # 计算航向误差的PID控制
    yaw_rate_command = np.clip(yaw_rate_output[0], -1.0, 1.0)  # 限制航向控制命令的范围在 [-1.0, 1.0] 之间

    # 更新历史状态，以便在下一次控制时使用
    last_pos = pos.copy()
    last_filtered_vel = filtered_vel.copy()
    last_filtered_acc = filtered_acc.copy()

    # 将控制命令限制在 [-1.0, 1.0] 范围内
    vx = np.clip(v_command_transformed[0], -1.0, 1.0)  # 限制x轴速度命令
    vy = np.clip(v_command_transformed[1], -1.0, 1.0)  # 限制y轴速度命令
    vz = np.clip(v_command_transformed[2], -1.0, 1.0)  # 限制z轴速度命令

    # 解析当前状态
    pos = np.array(state[0:3])  # 当前无人机位置 (x, y, z)
    
    # 打印当前无人机坐标
    print(f"Current position: x={pos[0]}, y={pos[1]}, z={pos[2]}")

    # 返回速度和航向命令
    return (vx, vy, vz, yaw_rate_command)
