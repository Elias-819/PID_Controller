import numpy as np
from src.PID_controller import PIDController

last_pos = None
last_filtered_vel = None
stable_counter = 0

alpha_vel = 0.5  # 更高的响应性

# 控制器：仅使用位置 + 速度控制
pid_pos = PIDController(1.0, 0.02, 0.01, [0.5, 0.5, 0.5])
pid_vel = PIDController(0.4, 0.01, 0.005, [0.3, 0.3, 0.3])
pid_yaw = PIDController(0.8, 0.0, 0.05, [0.5, 0.5, 0.5])

# 阈值与区域
freeze_zone = 0.04  # 更小的目标半径
yaw_threshold = 0.03
STABLE_REQUIRED_STEPS = 10

def controller(state, target, dt):
    global last_pos, last_filtered_vel, stable_counter

    pos = np.array(state[0:3])
    yaw = state[5]
    target_pos = np.array(target[0:3])
    target_yaw = target[3]

    if last_pos is None:
        raw_vel = np.zeros(3)
        filtered_vel = np.zeros(3)
    else:
        raw_vel = (pos - last_pos) / dt
        filtered_vel = alpha_vel * raw_vel + (1 - alpha_vel) * last_filtered_vel

    pos_error = target_pos - pos
    yaw_error = np.arctan2(np.sin(target_yaw - yaw), np.cos(target_yaw - yaw))

    # ========= ✅ 更严格的稳定检测 =========
    if (
        np.linalg.norm(pos_error) < freeze_zone and
        np.linalg.norm(filtered_vel) < 0.05 and
        abs(yaw_error) < yaw_threshold
    ):
        stable_counter += 1
    else:
        stable_counter = 0

    if stable_counter >= STABLE_REQUIRED_STEPS:
        return (0.0, 0.0, 0.0, 0.0)

    # ========= ✅ 仅使用位置 + 速度控制 =========
    u_pos = pid_pos.control_update(pos_error, dt)
    vel_error = u_pos - filtered_vel
    u_vel = pid_vel.control_update(vel_error, dt)

    # ========= ✅ 非线性缩放靠近目标的输出 =========
    distance = np.linalg.norm(pos_error)
    if distance < 0.2:
        scale = distance / 0.2  # [0,1]
        u_vel *= scale  # 缩小靠近目标时的速度命令

    # ========= ✅ 坐标转换到机体坐标系 =========
    R = np.array([[np.cos(yaw), np.sin(yaw)],
                  [-np.sin(yaw), np.cos(yaw)]])
    v_xy_body = R @ u_vel[0:2]
    v_command = np.array([v_xy_body[0], v_xy_body[1], u_vel[2]])

    # ========= ✅ 航向控制带死区 =========
    if abs(yaw_error) < 0.02:
        yaw_rate_command = 0.0
    else:
        yaw_rate_output = pid_yaw.control_update(np.array([yaw_error, 0, 0]), dt)
        yaw_rate_command = np.clip(yaw_rate_output[0], -1.0, 1.0)

    # ========= ✅ 状态更新 =========
    last_pos = pos.copy()
    last_filtered_vel = filtered_vel.copy()

    # ========= ✅ 限幅 =========
    vx = np.clip(v_command[0], -1.0, 1.0)
    vy = np.clip(v_command[1], -1.0, 1.0)
    vz = np.clip(v_command[2], -1.0, 1.0)

    print(f"Current position: x={pos[0]}, y={pos[1]}, z={pos[2]}")
    return (vx, vy, vz, yaw_rate_command)
