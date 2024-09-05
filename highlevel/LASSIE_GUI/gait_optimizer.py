import numpy as np
from scipy.optimize import minimize


class GaitOptimizer:
    def __init__(self, robot_weight=2.5, flipper_length=0.12, flipper_width=0.02, 
                 max_motor_torque=0.75, motor_acceleration_time=0.1, 
                 extrude_depth=0.01, extrude_width=0.01,
                 k_v = 0.1, 
                 bounds=[(0.5, 5), (0.5, 5), (0.5, 5), (0.5, 5), (0.001, 0.05), (np.pi/6, np.pi/6)]):
        # Initialize robot and terrain constants
        self.ROBOT_WEIGHT = robot_weight
        self.FLIPPER_LENGTH = flipper_length
        self.FLIPPER_WIDTH = flipper_width
        self.MAX_MOTOR_TORQUE = max_motor_torque
        self.MOTOR_ACCELERATION_TIME = motor_acceleration_time
        self.EXTRUDE_DEPTH = extrude_depth
        self.EXTRUDE_WIDTH = extrude_width
        self.fm = self.MAX_MOTOR_TORQUE / self.FLIPPER_LENGTH
        self.k_v = k_v
        # Set the bounds for the optimization variables
        self.bounds = bounds

    def _shear_force(self, d, k_s_average,t_s):
        return k_s_average * 0.5 * d**2 * self.FLIPPER_WIDTH + self.k_v * (d/t_s)**2

    def _resistance_force(self, k_s_average):
        return k_s_average * 0.5 * self.EXTRUDE_DEPTH * self.EXTRUDE_DEPTH * self.EXTRUDE_WIDTH

    def _acceleration_force(self, alpha, t_s):
        return self.ROBOT_WEIGHT * alpha * self.FLIPPER_LENGTH / (self.MOTOR_ACCELERATION_TIME * t_s)
    

    def _penetration_force(self, d, t_p, k_p):
        return k_p * d + self.k_v* (d/t_p)**2
    def _extraction_force(self, d, t_e, k_e):
        return k_e * d + self.k_v* (d/t_e)**2

    def _step_length(self, d, alpha, t_s, k_s_average):
        f_s = self._shear_force(d, k_s_average, t_s)
        f_r = self._resistance_force(k_s_average)
        k_alpha_t_s = self._acceleration_force(alpha, t_s)
        sqrt_term = np.sqrt(1 - ((f_r + k_alpha_t_s) / (2 * f_s))**2)
        return 2 * self.FLIPPER_LENGTH * min(alpha, sqrt_term)

    def _forward_speed(self, mu, k_s_average):
        t_p, t_s, t_e, t_b, d, alpha = mu
        S_e = self._step_length(d, alpha, t_s, k_s_average)
        return S_e / (t_p + t_s + t_e + t_b)

    def _objective(self, mu, k_s_average):
        return -self._forward_speed(mu, k_s_average)  # Negative for minimization

    def _constraint1(self, mu, k_s_average):
        return self._step_length(mu[4], mu[5], mu[1], k_s_average) - self.FLIPPER_LENGTH / 2

    def _constraint2(self, mu, k_s_average):
        return self.fm - self._shear_force(mu[4], k_s_average,mu[1])

    def _constraint3(self, mu, k_s_average):
        return self.fm - self._resistance_force(k_s_average) - self._acceleration_force(mu[6], mu[1])
    
    def _constraint4(self, mu, k_e):
        return self.fm - self._extraction_force(mu[4], mu[2], k_e) 
    
    def _constraint5(self, mu, k_p):
        return self.fm - self._penetration_force(mu[4], mu[0], k_p) 


    def optimize(self, k_p, k_s, k_e, mu0=[1, 1, 1, 1, 0.02, np.pi/12]):

        print(k_p)
        print(k_s)
        print(k_e)
        # Define constraints in a dictionary form
        constraints = [
            {'type': 'ineq', 'fun': lambda mu: self._constraint1(mu, k_s)},
            {'type': 'ineq', 'fun': lambda mu: self._constraint2(mu, k_s)},
            {'type': 'ineq', 'fun': lambda mu: self._constraint3(mu, k_s)},
            {'type': 'ineq', 'fun': lambda mu: self._constraint4(mu, k_e)},
            {'type': 'ineq', 'fun': lambda mu: self._constraint5(mu, k_p)},
        ]

        # Perform the optimization
        result = minimize(self._objective, mu0, args=(k_s), 
                          bounds=self.bounds, constraints=constraints, method="L-BFGS-B")

        # Check if the optimization was successful
        if result.success:
            optimal_mu = result.x
            control_vector_u = {
                't_p': optimal_mu[0],
                't_s': optimal_mu[1],
                't_e': optimal_mu[2],
                't_b': optimal_mu[3],
                'd': optimal_mu[4],
                'alpha': optimal_mu[5],
                'v_x': -result.fun
            }
        else:
            raise ValueError("Optimization failed.")

        return optimal_mu, -result.fun

# Example usage of the GaitOptimizer class with default bounds
if __name__ == "__main__":
    k_s_average = 1.0  # example value
    k_body = 1.0       # example value
    k_alpha = 1.0      # example value

    # Use default bounds
    optimizer = GaitOptimizer()
    control_vector = optimizer.optimize(k_s_average, k_body, k_alpha)
    print(control_vector)

    # Example with custom bounds
    custom_bounds = [(0.5, 4), (0.5, 3), (0.5, 2), (0.5, 2), (0, 0.03), (0, np.pi/8)]
    optimizer_custom = GaitOptimizer(bounds=custom_bounds)
    control_vector_custom = optimizer_custom.optimize(k_s_average, k_body, k_alpha)
    print(control_vector_custom)
