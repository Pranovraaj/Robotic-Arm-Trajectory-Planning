# pyrefly: ignore [missing-import]
import matplotlib.pyplot as plt
import os
# pyrefly: ignore [missing-import]
import numpy as np
from kinematics import Kinematics

class GraphGenerator:
    def __init__(self, output_dir: str = "outputs/graphs"):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        # Premium dark theme
        plt.style.use('dark_background')
        
        # Customize default rcParams for better look
        plt.rcParams.update({
            'axes.facecolor': '#1e1e2e',
            'figure.facecolor': '#1e1e2e',
            'axes.edgecolor': '#313244',
            'axes.grid': True,
            'grid.color': '#313244',
            'grid.linestyle': '--',
            'text.color': '#cdd6f4',
            'axes.labelcolor': '#cdd6f4',
            'xtick.color': '#cdd6f4',
            'ytick.color': '#cdd6f4',
            'legend.facecolor': '#1e1e2e',
            'legend.edgecolor': '#313244',
        })
        
        # Color palette
        self.c_taylor = '#89b4fa' # Blue
        self.c_euler = '#f38ba8'  # Red/Pink
        self.c_error1 = '#cba6f7' # Mauve
        self.c_error2 = '#f9e2af' # Yellow
        
    def _save_and_close(self, filename: str):
        filepath = os.path.join(self.output_dir, filename)
        plt.tight_layout()
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()

    def generate_all_graphs(self, trajectory_result, kinematics: Kinematics):
        t = trajectory_result['time']
        taylor_j1 = trajectory_result['taylor']['j1']
        taylor_j2 = trajectory_result['taylor']['j2']
        euler_j1 = trajectory_result['euler']['j1']
        euler_j2 = trajectory_result['euler']['j2']
        
        # 1. Position vs Time
        plt.figure(figsize=(10, 6))
        plt.plot(t, taylor_j1['theta'], label='Joint 1 (Taylor)', color=self.c_taylor, linewidth=2.5)
        plt.plot(t, taylor_j2['theta'], label='Joint 2 (Taylor)', color='#74c7ec', linewidth=2.5)
        plt.plot(t, euler_j1['theta'], '--', label='Joint 1 (Euler)', color=self.c_euler, linewidth=2.5)
        plt.plot(t, euler_j2['theta'], '--', label='Joint 2 (Euler)', color='#eba0ac', linewidth=2.5)
        plt.title('Position vs Time (Joint Angles)', fontsize=14, fontweight='heavy', pad=15)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Angle (rad)', fontsize=12)
        plt.legend()
        self._save_and_close('1_Position_vs_Time.png')
        
        # 2. Velocity vs Time
        plt.figure(figsize=(10, 6))
        plt.plot(t, taylor_j1['velocity'], label='Joint 1 Vel (Taylor)', color=self.c_taylor, linewidth=2.5)
        plt.plot(t, taylor_j2['velocity'], label='Joint 2 Vel (Taylor)', color='#74c7ec', linewidth=2.5)
        plt.title('Velocity vs Time', fontsize=14, fontweight='heavy', pad=15)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Angular Velocity (rad/s)', fontsize=12)
        plt.legend()
        self._save_and_close('2_Velocity_vs_Time.png')
        
        # 3. Acceleration vs Time
        plt.figure(figsize=(10, 6))
        plt.plot(t, taylor_j1['acceleration'], label='Joint 1 Acc (Taylor)', color=self.c_taylor, linewidth=2.5)
        plt.plot(t, taylor_j2['acceleration'], label='Joint 2 Acc (Taylor)', color='#74c7ec', linewidth=2.5)
        plt.title('Acceleration vs Time', fontsize=14, fontweight='heavy', pad=15)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Angular Acceleration (rad/s²)', fontsize=12)
        plt.legend()
        self._save_and_close('3_Acceleration_vs_Time.png')
        
        # Calculate X, Y paths
        tx_t, ty_t = kinematics.forward_kinematics(taylor_j1['theta'], taylor_j2['theta'])
        ex_t, ey_t = kinematics.forward_kinematics(euler_j1['theta'], euler_j2['theta'])
        
        # 4. Taylor Series Trajectory (XY)
        plt.figure(figsize=(8, 8))
        plt.plot(tx_t, ty_t, label='End-Effector Path', color=self.c_taylor, linewidth=2.5)
        plt.title('Taylor Series Trajectory (X-Y Space)', fontsize=14, fontweight='heavy', pad=15)
        plt.xlabel('X Coordinate (m)', fontsize=12)
        plt.ylabel('Y Coordinate (m)', fontsize=12)
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        self._save_and_close('4_Taylor_Trajectory.png')
        
        # 5. Modified Euler Trajectory (XY)
        plt.figure(figsize=(8, 8))
        plt.plot(ex_t, ey_t, label='End-Effector Path', color=self.c_euler, linewidth=2.5)
        plt.title('Modified Euler Trajectory (X-Y Space)', fontsize=14, fontweight='heavy', pad=15)
        plt.xlabel('X Coordinate (m)', fontsize=12)
        plt.ylabel('Y Coordinate (m)', fontsize=12)
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        self._save_and_close('5_Modified_Euler_Trajectory.png')
        
        # 6. Combined Trajectory Comparison
        plt.figure(figsize=(8, 8))
        plt.plot(tx_t, ty_t, label='Taylor Series', color=self.c_taylor, linewidth=2.5)
        plt.plot(ex_t, ey_t, '--', label='Modified Euler', color=self.c_euler, linewidth=2.5)
        plt.title('Combined Trajectory Comparison', fontsize=14, fontweight='heavy', pad=15)
        plt.xlabel('X Coordinate (m)', fontsize=12)
        plt.ylabel('Y Coordinate (m)', fontsize=12)
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        self._save_and_close('6_Combined_Trajectory.png')
        
        # 7. Numerical Error Graph (Difference in joint angles)
        error_j1 = np.abs(taylor_j1['theta'] - euler_j1['theta'])
        error_j2 = np.abs(taylor_j2['theta'] - euler_j2['theta'])
        plt.figure(figsize=(10, 6))
        plt.plot(t, error_j1, label='Error Joint 1', color=self.c_error1, linewidth=2.5)
        plt.plot(t, error_j2, label='Error Joint 2', color=self.c_error2, linewidth=2.5)
        plt.title('Numerical Error Graph (Taylor vs Euler)', fontsize=14, fontweight='heavy', pad=15)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Absolute Difference (rad)', fontsize=12)
        plt.yscale('log')
        plt.legend()
        self._save_and_close('7_Numerical_Error.png')
        
        # 8. Computation Time Comparison (Bar Chart)
        plt.figure(figsize=(8, 6))
        methods = ['Taylor Series', 'Modified Euler']
        times = [trajectory_result['taylor']['total_time'], trajectory_result['euler']['total_time']]
        colors = [self.c_taylor, self.c_euler]
        plt.bar(methods, times, color=colors)
        plt.title('Computation Time Comparison', fontsize=14, fontweight='heavy', pad=15)
        plt.ylabel('Time (seconds)', fontsize=12)
        for i, v in enumerate(times):
            plt.text(i, v + (max(times)*0.01), f"{v:.6f} s", ha='center', fontweight='bold')
        self._save_and_close('8_Computation_Time.png')

