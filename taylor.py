# pyrefly: ignore [missing-import]
import numpy as np
import time

class TaylorSeriesSolver:
    def __init__(self, Kp: float = 100.0, Kd: float = 20.0):
        """
        Initialize the Taylor Series Solver for trajectory planning.
        Models the movement as a Proportional-Derivative (PD) control system.
        
        Args:
            Kp (float): Proportional gain (stiffness).
            Kd (float): Derivative gain (damping).
        """
        self.Kp = Kp
        self.Kd = Kd

    def solve(self, theta0: float, v0: float, target_theta: float, h: float, t_total: float):
        """
        Solve the differential equation using 3rd Order Taylor Series Method.
        Equation: d²θ/dt² = Kp(θ_target - θ) - Kd(dθ/dt)
        
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
            # Current state
            th_n = theta[i]
            v_n = v[i]
            
            # Derivatives
            # First derivative (velocity)
            th_prime = v_n
            
            # Second derivative (acceleration)
            th_double_prime = self.Kp * (target_theta - th_n) - self.Kd * v_n
            a[i] = th_double_prime
            
            # Third derivative (jerk)
            th_triple_prime = -self.Kp * v_n - self.Kd * th_double_prime
            
            # Fourth derivative
            th_quad_prime = -self.Kp * th_double_prime - self.Kd * th_triple_prime
            
            # Taylor Series Expansion for position (up to 4th term)
            theta[i+1] = th_n + h * th_prime + (h**2 / 2) * th_double_prime + \
                         (h**3 / 6) * th_triple_prime + (h**4 / 24) * th_quad_prime
                         
            # Taylor Series Expansion for velocity
            v[i+1] = v_n + h * th_double_prime + (h**2 / 2) * th_triple_prime + \
                     (h**3 / 6) * th_quad_prime
                     
        # Calculate acceleration for the last step
        a[-1] = self.Kp * (target_theta - theta[-1]) - self.Kd * v[-1]
        
        execution_time = time.perf_counter() - start_time
        
        return {
            'time': t_steps,
            'theta': theta,
            'velocity': v,
            'acceleration': a,
            'exec_time': execution_time
        }
