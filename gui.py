# pyrefly: ignore [missing-import]
import customtkinter as ctk
# pyrefly: ignore [missing-import]
import tkinter as tk
from tkinter import messagebox
# pyrefly: ignore [missing-import]
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from trajectory import TrajectoryPlanner
from graphs import GraphGenerator
from utils import Exporter
from comparison import ComparisonMetrics
from animation import RobotAnimation

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        self.title("Robotic Arm Trajectory Planning Dashboard")
        self.geometry("1400x900")
        
        # Configure grid layout (1 row, 2 columns)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        self._build_left_panel()
        self._build_right_panel()
        
        # State variables
        self.trajectory_result = None
        self.metrics = None
        
    def _build_left_panel(self):
        # Left Panel (Inputs)
        self.left_panel = ctk.CTkFrame(self, width=300, corner_radius=0)
        self.left_panel.grid(row=0, column=0, sticky="nsew")
        self.left_panel.grid_rowconfigure(9, weight=1)
        
        # Title
        title_label = ctk.CTkLabel(self.left_panel, text="Input Parameters", font=ctk.CTkFont(size=20, weight="bold"))
        title_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        
        # Link Lengths
        self.l1_entry = self._create_input_row("Link 1 Length (m):", "1.0", 1)
        self.l2_entry = self._create_input_row("Link 2 Length (m):", "1.0", 2)
        
        # Initial State
        self.th1_entry = self._create_input_row("Initial Joint 1 (rad):", "0.0", 3)
        self.th2_entry = self._create_input_row("Initial Joint 2 (rad):", "0.0", 4)
        
        # Target
        self.tx_entry = self._create_input_row("Target X (m):", "1.5", 5)
        self.ty_entry = self._create_input_row("Target Y (m):", "0.5", 6)
        
        # Simulation settings
        self.h_entry = self._create_input_row("Time Step (h):", "0.01", 7)
        self.t_entry = self._create_input_row("Sim Time (s):", "2.0", 8)
        
        # Buttons
        btn_frame = ctk.CTkFrame(self.left_panel, fg_color="transparent")
        btn_frame.grid(row=10, column=0, padx=20, pady=20, sticky="s")
        
        self.run_btn = ctk.CTkButton(btn_frame, text="Run Simulation", command=self.run_simulation, height=40)
        self.run_btn.pack(pady=10, fill="x")
        
        self.export_btn = ctk.CTkButton(btn_frame, text="Export Results (PDF/CSV)", command=self.export_results, height=40, state="disabled")
        self.export_btn.pack(pady=10, fill="x")
        
    def _create_input_row(self, label_text, default_val, row_idx):
        frame = ctk.CTkFrame(self.left_panel, fg_color="transparent")
        frame.grid(row=row_idx, column=0, padx=20, pady=5, sticky="ew")
        
        lbl = ctk.CTkLabel(frame, text=label_text)
        lbl.pack(side="left")
        
        entry = ctk.CTkEntry(frame, width=100)
        entry.insert(0, default_val)
        entry.pack(side="right")
        return entry
        
    def _build_right_panel(self):
        self.right_panel = ctk.CTkFrame(self, corner_radius=10)
        self.right_panel.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        self.right_panel.grid_rowconfigure(1, weight=1)
        self.right_panel.grid_columnconfigure(0, weight=1)
        
        # Dashboard Title
        dash_title = ctk.CTkLabel(self.right_panel, text="Live Dashboard & Comparison", font=ctk.CTkFont(size=24, weight="bold"))
        dash_title.grid(row=0, column=0, padx=20, pady=20, sticky="w")
        
        # Tabs for visualization
        self.tabview = ctk.CTkTabview(self.right_panel)
        self.tabview.grid(row=1, column=0, padx=20, pady=(0, 20), sticky="nsew")
        
        self.tabview.add("Animation")
        self.tabview.add("Comparison Metrics")
        
        # Setup Animation Canvas placeholder
        self.anim_frame = ctk.CTkFrame(self.tabview.tab("Animation"), fg_color="transparent")
        self.anim_frame.pack(fill="both", expand=True)
        
        # Setup Metrics Text placeholder
        self.metrics_text = ctk.CTkTextbox(self.tabview.tab("Comparison Metrics"), font=ctk.CTkFont(family="Consolas", size=16))
        self.metrics_text.pack(fill="both", expand=True, padx=20, pady=20)
        
    def run_simulation(self):
        try:
            # Gather inputs
            L1 = float(self.l1_entry.get())
            L2 = float(self.l2_entry.get())
            th1_0 = float(self.th1_entry.get())
            th2_0 = float(self.th2_entry.get())
            target_x = float(self.tx_entry.get())
            target_y = float(self.ty_entry.get())
            h = float(self.h_entry.get())
            t_total = float(self.t_entry.get())
            
            planner = TrajectoryPlanner(L1, L2)
            self.trajectory_result = planner.generate_trajectories(
                th1_0, th2_0, 0.0, 0.0, target_x, target_y, h, t_total
            )
            
            # Generate Graphs
            graph_gen = GraphGenerator()
            graph_gen.generate_all_graphs(self.trajectory_result, planner.kinematics)
            
            # Calculate Metrics
            self.metrics = ComparisonMetrics.compare(
                self.trajectory_result['taylor'],
                self.trajectory_result['euler']
            )
            
            self._update_metrics_ui()
            self._update_animation_ui(L1, L2, target_x, target_y)
            
            self.export_btn.configure(state="normal")
            messagebox.showinfo("Success", "Simulation completed and graphs generated in 'outputs/graphs'!")
            
        except ValueError as e:
            messagebox.showerror("Input Error", str(e))
        except Exception as e:
            messagebox.showerror("Simulation Error", f"An unexpected error occurred: {e}")

    def _update_metrics_ui(self):
        self.metrics_text.delete("1.0", tk.END)
        self.metrics_text.insert(tk.END, "=== PERFORMANCE COMPARISON ===\n\n")
        self.metrics_text.insert(tk.END, f"Winner: {self.metrics['Winner']}\n\n")
        for k, v in self.metrics.items():
            if k != 'Winner':
                self.metrics_text.insert(tk.END, f"{k.replace('_', ' ')}: {v:.6f}\n")

    def _update_animation_ui(self, L1, L2, target_x, target_y):
        # Clear existing canvas if any
        for widget in self.anim_frame.winfo_children():
            widget.destroy()
            
        # Create new animation
        self.robot_anim = RobotAnimation(L1, L2)
        self.robot_anim.setup_data(self.trajectory_result, target_x, target_y)
        
        canvas = FigureCanvasTkAgg(self.robot_anim.get_figure(), master=self.anim_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Start animation
        self.robot_anim.animate()
        
    def export_results(self):
        if not self.trajectory_result or not self.metrics:
            return
            
        try:
            exporter = Exporter()
            csv_traj = exporter.export_trajectory_csv(self.trajectory_result)
            csv_comp = exporter.export_comparison_csv(self.metrics)
            pdf_rep = exporter.generate_project_report(self.metrics)
            
            msg = f"Export Successful!\n\nFiles saved:\n- {csv_traj}\n- {csv_comp}\n- {pdf_rep}"
            messagebox.showinfo("Export", msg)
        except Exception as e:
            messagebox.showerror("Export Error", str(e))
