# PID_Controller
# 🚁 PID-Based Drone Controller with Position, Velocity, and Acceleration Feedback

This repository provides a modular **PID controller-based motion controller** for drones (or other robotic systems), using position, velocity, and acceleration feedback for smooth and stable target tracking.

The controller implements a **multi-stage PID pipeline**:
- Position PID ➜ Velocity PID ➜ Acceleration PID
- With dynamic stability checks and filtering (low-pass smoothing)

---

## 🎯 Features

- ✅ Multi-stage PID control for smoother convergence
- ✅ Real-time stability detection with configurable thresholds
- ✅ Velocity and acceleration filtering using exponential smoothing
- ✅ Yaw (heading) angle normalization and control
- ✅ Stability zones with automatic freeze once target is reached
- ✅ Body-frame transformation for control command output

---

## 📦 File Structure

your_project/
├── controller.py         # Main control loop (this file)
├── src/
│   └── PID_controller.py # Custom PID controller class
└── README.md             # This file

## ⚙️ Dependencies
This script uses standard Python scientific libraries:

pip install numpy
No additional external libraries are required unless you integrate it into a ROS system or simulator.

## 🧠 Control Logic
Calculate position error relative to target

Estimate velocity and acceleration using past position and velocity

Apply exponential smoothing filters to reduce noise

Use PID controllers in a cascade:

Position PID ➜ generates velocity setpoint

Velocity PID ➜ generates acceleration setpoint

Acceleration PID ➜ outputs final control force

Normalize the yaw error using atan2(sin(Δyaw), cos(Δyaw))

Transform velocity commands from world frame to body frame

Apply tanh() to damp high-frequency spikes

Clamp the control output within [-1.0, 1.0]

When position, velocity, and yaw are stable for N steps, output zeros

## 🔧 Parameters
PID Gains
Controller	Kp	Ki	Kd	Output Limit
Position	1.0	0.05	0.01	[0.5, 0.5, 0.5]
Velocity	0.4	0.01	0.02	[0.3, 0.3, 0.3]
Acceleration	0.3	0.0	0.005	[0.1, 0.1, 0.1]
Yaw	1.2	0.0	0.05	[0.5, 0.5, 0.5]

## Stability Thresholds
pos_threshold = 0.05        # Position error (m)
yaw_threshold = 0.05        # Yaw error (rad)
stabilize_zone = 0.08       # Radius to scale control effort (m)
freeze_zone = 0.025         # Radius to enter "stop" mode (m)
STABLE_REQUIRED_STEPS = 15  # Number of stable steps to stop


## 🚀 Usage
You can call the controller(state, target, dt) function in your control loop:

vx, vy, vz, yaw_rate = controller(
    state=[x, y, z, roll, pitch, yaw],
    target=[x_d, y_d, z_d, yaw_d],
    dt=0.05  # time interval in seconds
)
Inputs
state: Current drone state [x, y, z, roll, pitch, yaw]

target: Desired target [x_d, y_d, z_d, yaw_d]

dt: Time interval since last call

Output
(vx, vy, vz, yaw_rate): Commanded velocities and yaw rate

