import numpy as np
from scipy.optimize import minimize


class GaitOptimizer:
    def __init__(self, robot_weight=3, flipper_length=0.12, flipper_width=0.025, 
                 max_motor_torque=1.5, motor_acceleration_time=0.024, 
                 extrude_depth=0.015, extrude_width=0.015,
                 k_v = 300, 
                 bounds=[(0.15, 7), (0.3, 14), (0.15, 7), (0.3, 0.3), (0.03, 0.03), (np.pi/6, np.pi/6)]):
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
        return k_s_average * 0.5 * d**2 * self.FLIPPER_WIDTH * 0.5

    def _resistance_force(self, k_s_average):
        return k_s_average * 0.5 * self.EXTRUDE_DEPTH * self.EXTRUDE_DEPTH * self.EXTRUDE_WIDTH

    def _acceleration_force(self, alpha, t_s):
        return self.ROBOT_WEIGHT * alpha * self.FLIPPER_LENGTH / (self.MOTOR_ACCELERATION_TIME * t_s)
    

    def _penetration_force(self, d, t_p, k_p):
        # print("static:", k_p * d * self.FLIPPER_WIDTH  * 0.005)
        # print(self.k_v* (d/t_p)**2)
        return k_p * d * self.FLIPPER_WIDTH  * 0.005 + self.k_v* (d/t_p)**2
    def _extraction_force(self, d, t_e, k_e):
        # print("static:", k_e * d * self.FLIPPER_WIDTH  * 0.005)
        return k_e * d * self.FLIPPER_WIDTH  * 0.005+ self.k_v* (d/t_e)**2

    def _step_length(self, d, alpha, t_s, k_s_average):
        f_s = self._shear_force(d, k_s_average, t_s)
        f_r = self._resistance_force(k_s_average)
        f_a = self._acceleration_force(alpha, t_s)
        if((f_r + f_a) >  (2 * f_s)):
            sqrt_term = 0
        else:
            sqrt_term = np.sqrt(1 - ((f_r + f_a) / (2 * f_s))**2)
        return 2 * self.FLIPPER_LENGTH * min(alpha, sqrt_term)

    def _effective_angle(self, d, alpha, t_s, k_s_average):
        f_s = self._shear_force(d, k_s_average, t_s)
        f_r = self._resistance_force(k_s_average)
        f_a = self._acceleration_force(alpha, t_s)
        
        effective_angle_cos = (f_r + f_a)/(2*f_s)
        return effective_angle_cos
    def _forward_speed(self, mu, k_s_average):
        t_p, t_s, t_e, t_b, d, alpha = mu
        S_e = self._step_length(d, alpha, t_s, k_s_average)
        return S_e / (t_p + t_s + t_e + t_b)

    def _objective(self, mu, k_s_average):
        return -self._forward_speed(mu, k_s_average)  # Negative for minimization

    def _constraint1(self, mu, k_s_average):
        # determine the sweeping velocity
        return -self._effective_angle(mu[4], mu[5], mu[1], k_s_average) + np.cos(mu[5])

    # def _constraint2(self, mu, k_s_average):
    #     return self.fm - self._shear_force(mu[4], k_s_average,mu[1])

    # def _constraint3(self, mu, k_s_average):
    #     return self.fm - self._resistance_force(k_s_average) - self._acceleration_force(mu[6], mu[1])
    
    def _constraint4(self, mu, k_e):
        # determin the extraction velocity
        return self.fm - self._extraction_force(mu[4], mu[2], k_e) 
    
    def _constraint5(self, mu, k_p):
        return self.fm - self._penetration_force(mu[4], mu[0], k_p) 


    def validate(self, k_p, k_s, k_e, mu0 ):
        t_p, t_s, t_e, t_b, d, alpha = mu0
        f_s = self._shear_force(d, k_s_average, t_s)
        f_r = self._resistance_force(k_s_average)
        f_a = self._acceleration_force(alpha, t_s)
        f_e = self._extraction_force(d, t_e, k_e)
        f_p = self._penetration_force(d, t_p, k_p)
        
        effective_angle = (f_r + f_a)/(2*f_s)
        speed = self._forward_speed(mu0, k_s)
        print("cons1", self._constraint1(mu0, k_s))
        print("cons4", self._constraint4(mu0, k_e))
        print("cons5", self._constraint5(mu0, k_p))
        print("f_s", f_s)
        print("f_r", f_r)
        print("f_m", self.fm)
        print("f_a", f_a)
        print("f_e", f_e)
        print("f_p", f_p)
        print("effective angle cos", effective_angle)
        print("alpha cos", np.cos(alpha))
        print("speed", speed)

    def optimize(self, k_p, k_s, k_e, mu0=[1, 1, 1, 1, 0.02, np.pi/12]):

        print(k_p)
        print(k_s)
        print(k_e)
        # Define constraints in a dictionary form
        constraints = [
            {'type': 'ineq', 'fun': lambda mu: self._constraint1(mu, k_s)},
            # {'type': 'ineq', 'fun': lambda mu: self._constraint2(mu, k_s)},
            # {'type': 'ineq', 'fun': lambda mu: self._constraint3(mu, k_s)},
            {'type': 'ineq', 'fun': lambda mu: self._constraint4(mu, k_e)},
            {'type': 'ineq', 'fun': lambda mu: self._constraint5(mu, k_p)},
        ]

        # Perform the optimization
        result = minimize(self._objective, mu0, args=(k_s), 
                          bounds=self.bounds, constraints=constraints, method="SLSQP")

        # Check if the optimization was successful
        if result.success:
            optimal_mu = result.x
            reclaim_mu = optimal_mu
            print("gait time", reclaim_mu)
            optimal_mu[0] = optimal_mu[2]
            optimal_mu[0] = optimal_mu[4]/optimal_mu[0]
            optimal_mu[1] = optimal_mu[5]/optimal_mu[1] * self.FLIPPER_LENGTH
            optimal_mu[2] = optimal_mu[4]/optimal_mu[2]
            optimal_mu[3] = optimal_mu[5]/optimal_mu[3]* self.FLIPPER_LENGTH
            print("gait speed", optimal_mu)
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
    k_s_average = 1.4 * 1e6  # example value
    k_p = 1.0 * 1e6      # example value
    k_e = 1.1 * 1e6      # example value

    # Use default bounds
    optimizer = GaitOptimizer()
    optimizer.validate(k_p, k_s_average, k_e, [7,14,7,0.3, 0.03, np.pi/6])
    # control_vector, speed = optimizer.optimize(k_p, k_s_average, k_e)
    # print(control_vector)

    # optimizer.validate(k_p, k_s_average, k_e, control_vector)

    # # Example with custom bounds
    # custom_bounds = [(0.5, 4), (0.5, 3), (0.5, 2), (0.5, 2), (0.03, 0.03), (np.pi/6, np.pi/6)]
    # optimizer_custom = GaitOptimizer(bounds=custom_bounds)
    # control_vector_custom = optimizer_custom.optimize(k_p, k_s_average, k_e)
    # print(control_vector_custom)
