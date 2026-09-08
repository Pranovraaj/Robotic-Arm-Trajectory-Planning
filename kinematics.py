# pyrefly: ignore [missing-import]
import numpy as np  

class Kinematics:
    def __init__(self, L1: float, L2: float):
        """
        Initialize the 2-DOF Robotic Arm Kinematics.
        
        Args:
            L1 (float): Length of the first link.
            L2 (float): Length of the second link.
        """
        self.L1 = L1
        self.L2 = L2

    def forward_kinematics(self, theta1: float, theta2: float):
        """
        Calculate the end-effector position given joint angles.
        
        Args:
            theta1 (float): Angle of joint 1 in radians.
            theta2 (float): Angle of joint 2 in radians.
            
        Returns:
            tuple: (x, y) coordinates of the end-effector.
        """
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        return x, y

    def get_joint_positions(self, theta1: float, theta2: float):
        """
        Calculate the coordinates of all joints for plotting.
        
        Args:
            theta1 (float): Angle of joint 1 in radians.
            theta2 (float): Angle of joint 2 in radians.
            
        Returns:
            tuple: (x_coords, y_coords) where each is a list of [origin, joint1, end_effector]
        """
        x0, y0 = 0.0, 0.0
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)
        return [x0, x1, x2], [y0, y1, y2]

    def inverse_kinematics(self, x: float, y: float):
        """
        Calculate joint angles required to reach a target (x, y).
        
        Args:
            x (float): Target x coordinate.
            y (float): Target y coordinate.
            
        Returns:
            tuple: (theta1, theta2) in radians.
            
        Raises:
            ValueError: If the target is unreachable.
        """
        distance_sq = x**2 + y**2
        if distance_sq > (self.L1 + self.L2)**2 or distance_sq < (self.L1 - self.L2)**2:
            raise ValueError(f"Target ({x}, {y}) is unreachable.")

        cos_theta2 = (distance_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        # Clamp to avoid floating point errors slightly exceeding domain [-1, 1]
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        
        # We choose the 'elbow down' configuration by default (positive root)
        theta2 = np.arccos(cos_theta2)
        
        k1 = self.L1 + self.L2 * np.cos(theta2)
        k2 = self.L2 * np.sin(theta2)
        
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        return theta1, theta2
