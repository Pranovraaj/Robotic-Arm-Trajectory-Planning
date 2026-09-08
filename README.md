<div align="center">
  <h1>🤖 Robotic Arm Trajectory Planning</h1>
  <p><em>A Python-based software simulation for a B.Tech Capstone Project</em></p>

  [![Python](https://img.shields.io/badge/Python-3.11+-blue.svg)](https://www.python.org/)
  [![CustomTkinter](https://img.shields.io/badge/GUI-CustomTkinter-blueviolet.svg)](https://github.com/TomSchimansky/CustomTkinter)
  [![Matplotlib](https://img.shields.io/badge/Visualization-Matplotlib-orange.svg)](https://matplotlib.org/)
  [![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
</div>

<br />

## 📖 Overview

This repository contains a complete software simulation designed for a B.Tech Capstone Project titled **"Robotic Arm Trajectory Planning using Taylor Series Method and Modified Euler Method"**. 

It models the kinematics and dynamics of a 2-DOF (Degree of Freedom) robotic arm and uses numerical methods to simulate its trajectory toward a target coordinate under Proportional-Derivative (PD) control.

## ✨ Key Features

- **Mathematical Modeling**: Accurate modeling of 2-DOF robotic arm kinematics and dynamics using Proportional-Derivative (PD) control.
- **Advanced Numerical Solvers**: Implements and compares the **4th-order Taylor Series** method and **Modified Euler (Predictor-Corrector)** method.
- **Modern GUI Dashboard**: Built with `customtkinter` for a professional, sleek, dark-mode engineering dashboard.
- **Real-time Animation**: Live visual simulation of the robotic arm reaching its target using `matplotlib`.
- **Comprehensive Analysis**: Generates professional graphs tracking metrics like Position, Velocity, Acceleration, and Error over time.
- **Export Capabilities**: Automatically generates a PDF project report, viva questions, and exports simulation data to CSV formats.

## 🧮 Mathematical Background

1. **Forward & Inverse Kinematics**: Calculates the end-effector position based on joint angles and vice versa.
2. **PD Control System**: Computes the necessary torques for the joints to reach the target angles while minimizing overshoot and steady-state error.
3. **Numerical Integration**:
   - **Taylor Series Method (4th Order)**: Provides high-accuracy trajectory approximation by computing higher-order derivatives of the motion equations.
   - **Modified Euler Method**: A predictor-corrector approach offering a good balance between computational efficiency and numerical stability.

## 🚀 Installation & Setup

1. **Prerequisites**: Ensure you have **Python 3.11+** installed on your system.
2. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/RoboticArmTrajectoryPlanning.git
   cd RoboticArmTrajectoryPlanning
   ```
3. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```
   *Key dependencies include: `numpy`, `matplotlib`, `scipy`, `Pillow`, `customtkinter`, and `fpdf2`.*

## 💻 Usage

1. **Launch the Application**:
   ```bash
   python main.py
   ```
2. **Configure Parameters**: Use the left panel of the GUI to adjust input parameters:
   - Link lengths ($L_1$, $L_2$)
   - Initial Joint Angles ($\theta_1$, $\theta_2$)
   - Target Coordinates (X, Y)
   - Simulation time and step size
3. **Run Simulation**: Click the **Run Simulation** button to start the trajectory calculation.
4. **Visualize**: Watch the real-time robotic arm animation right within the dashboard.
5. **Analyze Metrics**: Navigate to the second tab to review detailed performance metrics.
6. **Export**: Click **Export Results** to auto-generate a comprehensive PDF report in the `documentation/` folder and CSV data in the `outputs/` folder.

## 📂 Project Structure

```text
📦 RoboticArmTrajectoryPlanning
├── 📜 main.py                # Entry point of the application
├── 📜 gui.py                 # Modern CustomTkinter interface
├── 📜 trajectory.py          # Orchestrates trajectory generation
├── 📜 taylor.py              # Taylor Series numerical method implementation
├── 📜 modified_euler.py      # Modified Euler method implementation
├── 📜 kinematics.py          # Forward and Inverse kinematics math
├── 📜 comparison.py          # Compares different numerical methods
├── 📜 animation.py           # Matplotlib arm animation logic
├── 📜 graphs.py              # Analytical plot generation
├── 📜 utils.py               # PDF report and CSV export utilities
├── 📜 requirements.txt       # Python dependencies
├── 📁 assets/                # Images and icons for GUI
├── 📁 outputs/               # Generated graphs, data (CSV), and animations
└── 📁 documentation/         # Auto-generated PDF reports and materials
```

## 📊 Outputs Generated

Upon running the simulation and exporting, the software automatically yields:
- **`outputs/graphs/`**: High-resolution analytical plots covering position, velocity, torque, and error.
- **`outputs/csv/`**: Raw dataset containing joint angles and target variables at each time step.
- **`documentation/`**: A finalized PDF Project Report summarizing inputs, outputs, methods, and generated plots.

## 🤝 Contributing
Contributions, issues, and feature requests are welcome! Feel free to check the issues page.

## 📝 License
This project is licensed under the MIT License - see the `LICENSE` file for details.
