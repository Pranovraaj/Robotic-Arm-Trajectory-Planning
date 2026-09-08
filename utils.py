import csv
import os
from fpdf import FPDF
from datetime import datetime

class Exporter:
    def __init__(self):
        self.csv_dir = "outputs/csv"
        self.reports_dir = "outputs/reports"
        self.doc_dir = "documentation"
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.reports_dir, exist_ok=True)
        os.makedirs(self.doc_dir, exist_ok=True)

    def export_trajectory_csv(self, trajectory_result, filename="trajectory_data.csv"):
        filepath = os.path.join(self.csv_dir, filename)
        t = trajectory_result['time']
        t_j1 = trajectory_result['taylor']['j1']['theta']
        t_j2 = trajectory_result['taylor']['j2']['theta']
        e_j1 = trajectory_result['euler']['j1']['theta']
        e_j2 = trajectory_result['euler']['j2']['theta']
        
        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Taylor J1 (rad)', 'Taylor J2 (rad)', 'Euler J1 (rad)', 'Euler J2 (rad)'])
            for i in range(len(t)):
                writer.writerow([t[i], t_j1[i], t_j2[i], e_j1[i], e_j2[i]])
        
        return filepath

    def export_comparison_csv(self, metrics, filename="comparison_metrics.csv"):
        filepath = os.path.join(self.csv_dir, filename)
        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Metric', 'Value'])
            for key, value in metrics.items():
                writer.writerow([key, value])
        return filepath

    def generate_project_report(self, metrics, target_pdf="documentation/Project_Report.pdf"):
        pdf = FPDF()
        pdf.set_auto_page_break(auto=True, margin=15)
        
        # 1. Title Page
        pdf.add_page()
        pdf.set_font("Helvetica", style="B", size=24)
        pdf.cell(0, 60, "", ln=True)
        pdf.cell(0, 10, "B.Tech Capstone Project", ln=True, align='C')
        pdf.set_font("Helvetica", size=16)
        pdf.cell(0, 20, "Robotic Arm Trajectory Planning using", ln=True, align='C')
        pdf.cell(0, 10, "Taylor Series Method and Modified Euler Method", ln=True, align='C')
        pdf.cell(0, 40, "", ln=True)
        pdf.cell(0, 10, f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M')}", ln=True, align='C')
        
        # 2. Abstract
        pdf.add_page()
        pdf.set_font("Helvetica", style="B", size=16)
        pdf.cell(0, 10, "Abstract", ln=True)
        pdf.set_font("Helvetica", size=12)
        abstract_text = (
            "This project presents a comprehensive software implementation for trajectory planning "
            "of a 2-DOF robotic arm. It models the motion using Proportional-Derivative (PD) control, "
            "resulting in a second-order differential equation. Two numerical methods—the Taylor Series "
            "Method and the Modified Euler (Predictor-Corrector) Method—are implemented to solve the ODE "
            "and generate smooth trajectories. A performance comparison is conducted based on execution time, "
            "accuracy, and trajectory smoothness."
        )
        pdf.multi_cell(0, 10, abstract_text)
        
        # 3. Methodology
        pdf.add_page()
        pdf.set_font("Helvetica", style="B", size=16)
        pdf.cell(0, 10, "Methodology", ln=True)
        pdf.set_font("Helvetica", size=12)
        method_text = (
            "Forward and Inverse Kinematics were used to map task space coordinates to joint space angles. "
            "The ODE governing the motion is: d²θ/dt² = Kp(θ_target - θ) - Kd(dθ/dt). "
            "The Taylor Series method expands this into a 4th-order polynomial approximation, while the "
            "Modified Euler method uses a predictor step followed by an average slope correction step."
        )
        pdf.multi_cell(0, 10, method_text)
        
        # 4. Results
        pdf.add_page()
        pdf.set_font("Helvetica", style="B", size=16)
        pdf.cell(0, 10, "Performance Comparison", ln=True)
        pdf.set_font("Helvetica", size=12)
        pdf.cell(0, 10, f"Winner: {metrics['Winner']}", ln=True)
        for key, value in metrics.items():
            if key != 'Winner':
                pdf.cell(0, 10, f"{key.replace('_', ' ')}: {value:.6g}", ln=True)
                
        # 5. Viva Questions
        pdf.add_page()
        pdf.set_font("Helvetica", style="B", size=16)
        pdf.cell(0, 10, "Viva Questions (Sample)", ln=True)
        pdf.set_font("Helvetica", style="B", size=12)
        pdf.cell(0, 10, "Q1: What is Inverse Kinematics?", ln=True)
        pdf.set_font("Helvetica", size=12)
        pdf.multi_cell(0, 10, "A: It is the mathematical process of calculating the joint angles required for the end-effector to reach a specific target position.")
        
        pdf.set_font("Helvetica", style="B", size=12)
        pdf.cell(0, 10, "Q2: Why use Modified Euler over standard Euler?", ln=True)
        pdf.set_font("Helvetica", size=12)
        pdf.multi_cell(0, 10, "A: Modified Euler (Predictor-Corrector) averages the slopes at the beginning and predicted end of the step, yielding higher accuracy (2nd order) and better stability than standard 1st-order Euler.")

        pdf.output(target_pdf)
        return target_pdf
