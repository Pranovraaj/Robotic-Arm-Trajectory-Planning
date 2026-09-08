# pyrefly: ignore [missing-import]
import numpy as np

class ComparisonMetrics:
    @staticmethod
    def calculate_rmse(y_true, y_pred):
        """Calculate Root Mean Square Error."""
        return np.sqrt(np.mean((y_true - y_pred)**2))
        
    @staticmethod
    def calculate_mae(y_true, y_pred):
        """Calculate Mean Absolute Error."""
        return np.mean(np.abs(y_true - y_pred))
        
    @staticmethod
    def calculate_max_error(y_true, y_pred):
        """Calculate Maximum Error."""
        return np.max(np.abs(y_true - y_pred))
        
    @staticmethod
    def evaluate_smoothness(acceleration):
        """
        Evaluate trajectory smoothness using the integral of squared jerk.
        (Smaller value means smoother).
        For simplicity, we take the variance of the acceleration.
        """
        return np.var(acceleration)

    @classmethod
    def compare(cls, taylor_data, euler_data, high_acc_reference=None):
        """
        Compares Taylor Series and Modified Euler results.
        If high_acc_reference is None, compares Euler relative to Taylor (assuming Taylor 4th order is more accurate).
        """
        if high_acc_reference is None:
            # Assuming Taylor is the more accurate reference for this capstone
            ref = taylor_data
        else:
            ref = high_acc_reference

        # Joint 1 Metrics
        j1_rmse = cls.calculate_rmse(ref['j1']['theta'], euler_data['j1']['theta'])
        j1_mae = cls.calculate_mae(ref['j1']['theta'], euler_data['j1']['theta'])
        j1_max_err = cls.calculate_max_error(ref['j1']['theta'], euler_data['j1']['theta'])
        
        # Smoothness
        taylor_smoothness = cls.evaluate_smoothness(taylor_data['j1']['acceleration']) + \
                            cls.evaluate_smoothness(taylor_data['j2']['acceleration'])
        euler_smoothness = cls.evaluate_smoothness(euler_data['j1']['acceleration']) + \
                           cls.evaluate_smoothness(euler_data['j2']['acceleration'])

        return {
            'RMSE': j1_rmse,
            'MAE': j1_mae,
            'Max_Error': j1_max_err,
            'Taylor_Time': taylor_data['total_time'],
            'Euler_Time': euler_data['total_time'],
            'Taylor_Smoothness': taylor_smoothness,
            'Euler_Smoothness': euler_smoothness,
            'Winner': 'Taylor Series' if j1_rmse < 1e-3 and taylor_data['total_time'] < euler_data['total_time'] * 1.5 else 'Modified Euler'
        }
