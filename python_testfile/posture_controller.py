# This is a python test file for controller test used for posture controller.
# Controller input target posture, and the real posture, output target angle velocity.

import numpy as np
import matplotlib.pyplot as plt
import math as math

class PIDController:
    # PID Controller
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error, dt):
        # Calculate Output of PID Controller
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

class Quadrotor:
    # Quadrotor Simulator
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0 

    def update(self, dt):
        # Update drone's posture
        self.roll += self.roll_rate * dt
        self.pitch += self.pitch_rate * dt
        self.yaw += self.yaw_rate * dt

def main():
    # Initialise PID Controller
    pid_roll = PIDController(Kp=20.0, Ki=1.0, Kd=0.5)
    pid_pitch = PIDController(Kp=20.0, Ki=1.0, Kd=0.5)
    pid_yaw = PIDController(Kp=20.0, Ki=1.0, Kd=0.5)

    # Initialise Drone
    quad = Quadrotor()
    
    # Simulation parameters
    sim_time = 10.0  # Total sim time
    dt = 0.01        # Time step
    timesteps = np.arange(0, sim_time, dt)
    
    # Recorder
    time_history = []
    roll_history = []
    pitch_history = []
    yaw_history = []
    target_roll_history = []
    target_pitch_history = []
    target_yaw_history = []

    for t in timesteps:
        # Target Angle
        target_roll = math.pi / 2
        target_pitch = math.sin(math.pi * t + math.pi / 2)
        target_yaw = math.sin(math.pi * t)
        target_roll_history.append(target_roll)
        target_pitch_history.append(target_pitch)
        target_yaw_history.append(target_yaw)

        # Error calculate
        error_roll = target_roll - quad.roll
        error_pitch = target_pitch - quad.pitch
        error_yaw = target_yaw - quad.yaw

        # PID controller calculation
        quad.roll_rate = pid_roll.compute(error_roll, dt)
        quad.pitch_rate = pid_pitch.compute(error_pitch, dt)
        quad.yaw_rate = pid_yaw.compute(error_yaw, dt)

        # Update posture
        quad.update(dt)

        # Record all the data
        time_history.append(t)
        roll_history.append(quad.roll)
        pitch_history.append(quad.pitch)
        yaw_history.append(quad.yaw)

    # Plot Result
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(time_history, roll_history, label='Actual')
    plt.plot(time_history, target_roll_history, color='r', linestyle='--', label='Target')
    plt.ylabel('Roll (rad)')
    plt.title('Attitude Control Performance')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(time_history, pitch_history, label='Actual')
    plt.plot(time_history, target_pitch_history, color='r', linestyle='--', label='Target')
    plt.ylabel('Pitch (rad)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(time_history, yaw_history, label='Actual')
    plt.plot(time_history, target_yaw_history, color='r', linestyle='--', label='Target')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (rad)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()