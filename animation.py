# pyrefly: ignore [missing-import]
import matplotlib.pyplot as plt
# pyrefly: ignore [missing-import]
from matplotlib.animation import FuncAnimation
# pyrefly: ignore [missing-import]
import numpy as np
from kinematics import Kinematics

class RobotAnimation:
    def __init__(self, L1: float, L2: float):
        self.kinematics = Kinematics(L1, L2)
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        
        # Premium dark theme (Catppuccin Mocha inspired)
        bg_color = '#1e1e2e'
        grid_color = '#313244'
        text_color = '#cdd6f4'
        
        self.fig.patch.set_facecolor(bg_color)
        self.ax.set_facecolor(bg_color)
        self.ax.tick_params(colors=text_color, labelsize=10)
        for spine in self.ax.spines.values():
            spine.set_color(grid_color)
            spine.set_linewidth(1.5)
            
        self.ax.set_aspect('equal')
        self.ax.grid(True, color=grid_color, linestyle='--', linewidth=0.7, alpha=0.8)
        
        # Max reach is L1 + L2
        max_reach = L1 + L2
        self.ax.set_xlim(-max_reach * 1.15, max_reach * 1.15)
        self.ax.set_ylim(-max_reach * 1.15, max_reach * 1.15)
        
        # Line objects with glow effects
        self.taylor_glow, = self.ax.plot([], [], '-', lw=10, color='#89b4fa', alpha=0.2)
        self.taylor_line, = self.ax.plot([], [], 'o-', lw=3, color='#89b4fa', markersize=10, markerfacecolor='#89b4fa', markeredgecolor='white', markeredgewidth=1.5, label='Taylor Series')
        
        self.euler_glow, = self.ax.plot([], [], '-', lw=7, color='#f38ba8', alpha=0.15)
        self.euler_line, = self.ax.plot([], [], 's--', lw=2, color='#f38ba8', markersize=7, markerfacecolor='#1e1e2e', markeredgecolor='#f38ba8', markeredgewidth=1.5, label='Modified Euler')
        
        self.target_glow, = self.ax.plot([], [], 'o', color='#f9e2af', markersize=22, alpha=0.15)
        self.target_point, = self.ax.plot([], [], '*', color='#f9e2af', markersize=16, markeredgecolor='white', markeredgewidth=1, label='Target')
        
        # End-effector trace paths
        self.taylor_path, = self.ax.plot([], [], '-', lw=2, color='#89b4fa', alpha=0.6)
        self.euler_path, = self.ax.plot([], [], ':', lw=2, color='#f38ba8', alpha=0.6)
        
        # Legend and title
        legend = self.ax.legend(loc='upper right', facecolor=bg_color, edgecolor=grid_color, labelcolor=text_color, framealpha=0.9, fancybox=True, shadow=True)
        legend.get_frame().set_linewidth(1.5)
        self.ax.set_title("Robotic Arm Real-time Animation", color=text_color, fontweight='heavy', fontsize=14, pad=15)
        
        self.animation = None

    def setup_data(self, trajectory_result, target_x, target_y):
        self.t_j1 = trajectory_result['taylor']['j1']['theta']
        self.t_j2 = trajectory_result['taylor']['j2']['theta']
        self.e_j1 = trajectory_result['euler']['j1']['theta']
        self.e_j2 = trajectory_result['euler']['j2']['theta']
        
        self.target_x = target_x
        self.target_y = target_y
        self.frames = len(self.t_j1)
        
        self.taylor_trace_x = []
        self.taylor_trace_y = []
        self.euler_trace_x = []
        self.euler_trace_y = []

    def init_anim(self):
        self.taylor_glow.set_data([], [])
        self.taylor_line.set_data([], [])
        self.euler_glow.set_data([], [])
        self.euler_line.set_data([], [])
        self.target_glow.set_data([self.target_x], [self.target_y])
        self.target_point.set_data([self.target_x], [self.target_y])
        self.taylor_path.set_data([], [])
        self.euler_path.set_data([], [])
        
        self.taylor_trace_x.clear()
        self.taylor_trace_y.clear()
        self.euler_trace_x.clear()
        self.euler_trace_y.clear()
        
        return self.taylor_glow, self.taylor_line, self.euler_glow, self.euler_line, self.target_glow, self.target_point, self.taylor_path, self.euler_path

    def update(self, frame):
        # Taylor Series positions
        tx, ty = self.kinematics.get_joint_positions(self.t_j1[frame], self.t_j2[frame])
        self.taylor_glow.set_data(tx, ty)
        self.taylor_line.set_data(tx, ty)
        self.taylor_trace_x.append(tx[2])
        self.taylor_trace_y.append(ty[2])
        self.taylor_path.set_data(self.taylor_trace_x, self.taylor_trace_y)
        
        # Euler positions
        ex, ey = self.kinematics.get_joint_positions(self.e_j1[frame], self.e_j2[frame])
        self.euler_glow.set_data(ex, ey)
        self.euler_line.set_data(ex, ey)
        self.euler_trace_x.append(ex[2])
        self.euler_trace_y.append(ey[2])
        self.euler_path.set_data(self.euler_trace_x, self.euler_trace_y)
        
        return self.taylor_glow, self.taylor_line, self.euler_glow, self.euler_line, self.target_glow, self.target_point, self.taylor_path, self.euler_path

    def get_figure(self):
        return self.fig

    def animate(self):
        self.animation = FuncAnimation(
            self.fig, self.update, frames=self.frames,
            init_func=self.init_anim, blit=True, interval=20, repeat=False
        )
