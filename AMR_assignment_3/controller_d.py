import numpy as np
import time

# Define a simplified PIDController class as in PID_controller.py
class PIDController:
    def __init__(self, Kp, Ki, Kd, Ki_sat):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # Ensure Ki_sat is a numpy array for elementwise operations
        self.Ki_sat = np.array(Ki_sat, dtype=float)
        self.previous_error = np.zeros(3)
        self.int = np.zeros(3)
        
    def reset(self):
        self.int = np.zeros(3)
        self.previous_error = np.zeros(3)
        
    def control_update(self, error, dt):
        error = np.array(error, dtype=float)
        # Update the integral term
        self.int += error * dt
        # Clip the integral term to avoid windup
        self.int = np.clip(self.int, -self.Ki_sat, self.Ki_sat)
        # Compute the derivative term
        derivative = (error - self.previous_error) / dt
        self.previous_error = error.copy()
        # Calculate the PID output
        output = self.Kp * error + self.Ki * self.int + self.Kd * derivative
        return output

# Global variables for state estimation using fixed dt and filtering
last_pos = None
last_filtered_vel = None
last_filtered_acc = None

# Filtering coefficients (0 < alpha <= 1), lower values imply heavier filtering
alpha_vel = 0.2   # Smoothing for velocity
alpha_acc = 0.2   # Smoothing for acceleration

# Adjusted gains for the cascade PID
pid_pos = PIDController(0.2, 0.0, 0.005, [0.5, 0.5, 0.5])   # Outer loop: Position controller
pid_vel = PIDController(0.15, 0.0, 0.005, [0.3, 0.3, 0.3])  # Middle loop: Velocity controller
pid_acc = PIDController(0.1, 0.0, 0.005, [0.2, 0.2, 0.2])   # Inner loop: Acceleration controller
pid_yaw = PIDController(0.5, 0.0, 0.005, [0.5, 0.5, 0.5])    # Yaw controller

def controller(state, target, dt):
    """
    3-Layer Cascade PID Controller for UAV Position Stabilization with filtering and coordinate transformation.
    
    Inputs:
      - state: A numpy array representing the current state:
               [pos_x, pos_y, pos_z, roll, pitch, yaw]
      - target: A tuple (target_x, target_y, target_z, target_yaw)
      - dt: Time step for the control update (in seconds)
      
    Output:
      - A tuple (vx, vy, vz, yaw_rate) which are the desired velocity commands
        and yaw rate setpoint.
    """
    global last_pos, last_filtered_vel, last_filtered_acc

    # Extract current position and yaw from state
    pos = np.array(state[0:3])
    yaw = state[5]
    
    # Extract target position and yaw
    target_pos = np.array(target[0:3])
    target_yaw = target[3]
    
    # --- Estimate Velocity and Acceleration with Filtering ---
    if last_pos is None or last_filtered_vel is None or last_filtered_acc is None:
        # First call: initialize filtered estimates to zero
        raw_vel = np.zeros(3)
        filtered_vel = np.zeros(3)
        filtered_acc = np.zeros(3)
    else:
        # Compute raw velocity using fixed dt
        raw_vel = (pos - last_pos) / dt
        # Filter the velocity estimate (exponential moving average)
        filtered_vel = alpha_vel * raw_vel + (1 - alpha_vel) * last_filtered_vel
        # Compute raw acceleration from the filtered velocities
        raw_acc = (filtered_vel - last_filtered_vel) / dt
        # Filter the acceleration estimate
        filtered_acc = alpha_acc * raw_acc + (1 - alpha_acc) * last_filtered_acc

    # --- Outer Loop: Position Controller ---
    pos_error = target_pos - pos
    u_pos = pid_pos.control_update(pos_error, dt)

    # --- Middle Loop: Velocity Controller ---
    vel_error = u_pos - filtered_vel
    u_vel = pid_vel.control_update(vel_error, dt)

    # --- Inner Loop: Acceleration Controller ---
    acc_error = u_vel - filtered_acc
    u_acc = pid_acc.control_update(acc_error, dt)

    # Final velocity command is the sum of outer loop and inner loop corrections (in inertial frame)
    v_command = u_pos + u_acc

    # --- Coordinate Transformation ---
    # Transform the x-y components of the velocity command from the inertial frame to the body frame.
    # For a drone, the transformation from inertial to body frame (given current yaw) is:
    # [v_x_body, v_y_body]^T = R_y(-yaw) * [v_x_inertial, v_y_inertial]^T
    R = np.array([[np.cos(yaw), np.sin(yaw)],
                  [-np.sin(yaw), np.cos(yaw)]])
    v_xy_body = R @ v_command[0:2]
    # The z component is assumed to be the same.
    v_command_transformed = np.array([v_xy_body[0], v_xy_body[1], v_command[2]])

    # --- Yaw Control ---
    yaw_error = target_yaw - yaw
    yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
    yaw_error_vec = np.array([yaw_error, 0, 0])
    yaw_rate_output = pid_yaw.control_update(yaw_error_vec, dt)
    yaw_rate_command = yaw_rate_output[0]

    # Update global variables for next iteration
    last_pos = pos.copy()
    last_filtered_vel = filtered_vel.copy()
    last_filtered_acc = filtered_acc.copy()

    return (v_command_transformed[0], v_command_transformed[1], v_command_transformed[2], yaw_rate_command)


