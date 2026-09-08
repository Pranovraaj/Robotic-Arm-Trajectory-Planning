# pyrefly: ignore [missing-import]
import numpy as np
import time

class ModifiedEulerSolver:
    def __init__(self, Kp: float = 100.0, Kd: float = 20.0):
        """
        Initialize the Modified Euler (Predictor-Corrector) Solver.
        Models the movement as a Proportional-Derivative (PD) control system.
        
        Args:
            Kp (float): Proportional gain (stiffness).
            Kd (float): Derivative gain (damping).
        """
        self.Kp = Kp
        self.Kd = Kd

    def derivative(self, theta: float, v: float, target_theta: float):
        """
        Calculates the derivatives [dθ/dt, d²θ/dt²].
        
        Args:
            theta (float): Current joint angle.
            v (float): Current joint velocity.
            target_theta (float): Target joint angle.
            
        Returns:
            tuple: (dθ/dt, d²θ/dt²)
        """
        d_theta = v
        d_v = self.Kp * (target_theta - theta) - self.Kd * v
        return d_theta, d_v

    def solve(self, theta0: float, v0: float, target_theta: float, h: float, t_total: float):
        """
        Solve the differential equation using Modified Euler Method (Predictor-Corrector).
        
        Args:
            theta0 (float): Initial joint angle.
            v0 (float): Initial joint velocity.
            target_theta (float): Target joint angle.
            h (float): Time step.
            t_total (float): Total simulation time.
            
        Returns:
            dict: Trajectory data containing time, position, velocity, and acceleration arrays, and execution time.
        """
        t_steps = np.arange(0, t_total + h, h)
        n = len(t_steps)
        
        theta = np.zeros(n)
        v = np.zeros(n)
        a = np.zeros(n)
        
        theta[0] = theta0
        v[0] = v0
        
        start_time = time.perf_counter()
        
        for i in range(n - 1):
            th_n = theta[i]
            v_n = v[i]
            
            # Current derivatives
            f1_theta, f1_v = self.derivative(th_n, v_n, target_theta)
            a[i] = f1_v
            
            # Predictor step (Euler)
            th_predict = th_n + h * f1_theta
            v_predict = v_n + h * f1_v
            
            # Predictor derivatives
            f2_theta, f2_v = self.derivative(th_predict, v_predict, target_theta)
            
            # Corrector step
            theta[i+1] = th_n + (h / 2) * (f1_theta + f2_theta)
            v[i+1] = v_n + (h / 2) * (f1_v + f2_v)
            
        # Calculate acceleration for the last step
        _, f_v_last = self.derivative(theta[-1], v[-1], target_theta)
        a[-1] = f_v_last
        
        execution_time = time.perf_counter() - start_time
        
        return {
            'time': t_steps,
            'theta': theta,
            'velocity': v,
            'acceleration': a,
            'exec_time': execution_time
        }
