from kinematics import Kinematics
from taylor import TaylorSeriesSolver
from modified_euler import ModifiedEulerSolver

class TrajectoryPlanner:
    def __init__(self, L1: float, L2: float, Kp: float = 100.0, Kd: float = 20.0):
        """
        Orchestrates trajectory generation for the robotic arm.
        """
        self.kinematics = Kinematics(L1, L2)
        self.taylor_solver = TaylorSeriesSolver(Kp, Kd)
        self.euler_solver = ModifiedEulerSolver(Kp, Kd)

    def generate_trajectories(self, 
                              theta1_0: float, theta2_0: float, 
                              v1_0: float, v2_0: float,
                              target_x: float, target_y: float,
                              h: float, t_total: float):
        """
        Generates trajectories for both joints using both methods.
        
        Returns:
            dict: Contains results for Taylor and Modified Euler methods.
        """
        # 1. Inverse Kinematics to find target angles
        target_theta1, target_theta2 = self.kinematics.inverse_kinematics(target_x, target_y)
        
        # 2. Generate Taylor Series Trajectories
        taylor_j1 = self.taylor_solver.solve(theta1_0, v1_0, target_theta1, h, t_total)
        taylor_j2 = self.taylor_solver.solve(theta2_0, v2_0, target_theta2, h, t_total)
        
        # 3. Generate Modified Euler Trajectories
        euler_j1 = self.euler_solver.solve(theta1_0, v1_0, target_theta1, h, t_total)
        euler_j2 = self.euler_solver.solve(theta2_0, v2_0, target_theta2, h, t_total)
        
        return {
            'target_angles': (target_theta1, target_theta2),
            'time': taylor_j1['time'], # Time steps are the same
            'taylor': {
                'j1': taylor_j1,
                'j2': taylor_j2,
                'total_time': taylor_j1['exec_time'] + taylor_j2['exec_time']
            },
            'euler': {
                'j1': euler_j1,
                'j2': euler_j2,
                'total_time': euler_j1['exec_time'] + euler_j2['exec_time']
            }
        }
