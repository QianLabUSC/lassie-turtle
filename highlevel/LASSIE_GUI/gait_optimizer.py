import numpy as np
from scipy.optimize import minimize


class GaitOptimizer:
    def __init__(self, robot_weight=3, flipper_length=0.12, flipper_width=0.025, 
                 max_motor_torque=1.5, motor_acceleration_time=0.08, 
                 extrude_depth=0.015, extrude_width=0.015,
                 k_v = 200, 
                 bounds=[(0.15, 1.2), (0.15, 1.2), (0.15, 1.2), (0.3, 0.3), (0.03, 0.03), (np.pi/6, np.pi/6)]):
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
        return k_p * d * self.FLIPPER_WIDTH  * 0.005 + self.k_v* (d/t_p)**2 * k_p * d * self.FLIPPER_WIDTH  * 0.005
    def _extraction_force(self, d, t_e, k_e):
        # print("static:", k_e * d * self.FLIPPER_WIDTH  * 0.005)
        return k_e * d * self.FLIPPER_WIDTH  * 0.005+ self.k_v* (d/t_e)**2 * k_p * d * self.FLIPPER_WIDTH  * 0.005

    def _step_length(self, d, alpha, t_s, k_s_average):
        # f_s = self._shear_force(d, k_s_average, t_s)
        # f_r = self._resistance_force(k_s_average)
        # f_a = self._acceleration_force(alpha, t_s)
        # if((f_r + f_a) >  (2 * f_s)):
        #     sqrt_term = 0
        # else:
        #     sqrt_term = np.sqrt(1 - ((f_r + f_a) / (2 * f_s))**2)
        return 2 * self.FLIPPER_LENGTH * alpha

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
        return (-self._effective_angle(mu[4], mu[5], mu[1], k_s_average) + np.cos(mu[5]))/1000

    # def _constraint2(self, mu, k_s_average):
    #     return self.fm - self._shear_force(mu[4], k_s_average,mu[1])

    # def _constraint3(self, mu, k_s_average):
    #     return self.fm - self._resistance_force(k_s_average) - self._acceleration_force(mu[6], mu[1])
    
    def _constraint4(self, mu, k_e):
        # determin the extraction velocity
        # print(mu[4], mu[2], k_e)
        print((self.fm - self._extraction_force(mu[4], mu[2], k_e) ))
        return (self.fm - self._extraction_force(mu[4], mu[2], k_e) )
    
    def _constraint5(self, mu, k_p):
        # print(mu[4], mu[0], k_p)
        return (self.fm - self._penetration_force(mu[4], mu[0], k_p))


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
        return f_s, f_r, f_a, f_e, f_p, self._constraint1(mu0, k_s), self._constraint4(mu0, k_e), self._constraint5(mu0, k_p)


    def find_max_extraction_velocity(self, mu0, k_e, tolerance=1e-6):
        """Find the maximum mu[2] (extraction velocity) such that constraint 4 is > 0."""
        lower_bound = self.bounds[2][0]
        upper_bound = self.bounds[2][1]
        max_mu2 = lower_bound
        
        # Iterate over a range of possible mu[2] values, increasing step by step
        step_size = 0.001  # Adjust this step size as needed
        
        for mu2 in np.arange(lower_bound, upper_bound, step_size):
            # Check if the constraint is satisfied
            constraint_value = self._constraint4([mu0[0], mu0[1], mu2, mu0[3], mu0[4], mu0[5]], k_e)
            # print("aaaa", constraint_value, mu2)
            if constraint_value >= 0:
                # If the constraint is satisfied, update the maximum valid mu[2]
                max_mu2 = mu2
                break
                
        
        print(f"Maximum valid extraction velocity (mu[2]): {max_mu2}")
        return max_mu2
    
    def find_max_sweeping_velocity(self, mu0, k_s, tolerance=1e-6):
        """Find the maximum mu[1] (sweeping velocity) such that constraint 1 is > 0."""
        lower_bound = self.bounds[1][0]
        upper_bound = self.bounds[1][1]
        max_mu1 = lower_bound
        
        # Iterate over a range of possible mu[1] values, increasing step by step
        step_size = 0.001  # Adjust this step size as needed
        
        for mu1 in np.arange(lower_bound, upper_bound, step_size):
            # Check if the constraint is satisfied
            constraint_value = self._constraint1([mu0[0], mu1, mu0[2], mu0[3], mu0[4], mu0[5]], k_s)
            
            if constraint_value >= 0:
                # If the constraint is satisfied, update the maximum valid mu[1]
                max_mu1 = mu1
                break
    
                
        
        print(f"Maximum valid sweeping velocity (mu[1]): {max_mu1}")
        return max_mu1
    def optimize(self, k_p, k_s, k_e, mu0=[1, 1, 1, 1, 0.03, np.pi/6]):

        print(f"k_p: {k_p}")
        print(f"k_s: {k_s}")
        print(f"k_e: {k_e}")
        optimal_mu = mu0
        
        optimal_mu[2] = self.find_max_extraction_velocity(mu0, k_e)
        optimal_mu[1] = self.find_max_sweeping_velocity(mu0, k_s)
    
        optimal_mu[3] = self.bounds[3][1]
        optimal_mu[0] = optimal_mu[2]
        optimal_mu[4] = self.bounds[4][1]
        optimal_mu[5] = self.bounds[5][1]
        print("gait time", optimal_mu)



        
        optimal_mu[0] = optimal_mu[4]/optimal_mu[0]
        optimal_mu[1] = optimal_mu[5]/optimal_mu[1] * self.FLIPPER_LENGTH
        optimal_mu[2] = optimal_mu[4]/optimal_mu[2]
        optimal_mu[3] = optimal_mu[5]/optimal_mu[3] * self.FLIPPER_LENGTH
        print("gait speed", optimal_mu)
        # # Define constraints in a dictionary form
        # constraints = [
        #     {'type': 'ineq', 'fun': lambda mu: self._constraint1(mu, k_s)},
        #     # {'type': 'ineq', 'fun': lambda mu: self._constraint2(mu, k_s)},
        #     # {'type': 'ineq', 'fun': lambda mu: self._constraint3(mu, k_s)},
        #     {'type': 'ineq', 'fun': lambda mu: self._constraint4(mu, k_e)},
        #     {'type': 'ineq', 'fun': lambda mu: self._constraint5(mu, k_p)},
        # ]


        # # give a reference mu0
        
        # # Perform the optimization
        # result = minimize(self._objective, mu0, args=(k_s), 
        #           bounds=self.bounds, constraints=constraints, 
        #           method="trust-constr", options={'gtol': 1e-12, 'xtol': 1e-12, 'barrier_tol': 1e-12})


        # # Check if the optimization was successful
        # if result.success:
        #     optimal_mu = result.x
        #     reclaim_mu = optimal_mu
        #     print("gait time", reclaim_mu)
        #     optimal_mu[0] = optimal_mu[2]
        #     optimal_mu[0] = optimal_mu[4]/optimal_mu[0]
        #     optimal_mu[1] = optimal_mu[5]/optimal_mu[1] * self.FLIPPER_LENGTH
        #     optimal_mu[2] = optimal_mu[4]/optimal_mu[2]
        #     optimal_mu[3] = optimal_mu[5]/optimal_mu[3]* self.FLIPPER_LENGTH
        #     print("gait speed", optimal_mu)
        #     control_vector_u = {
        #         't_p': optimal_mu[0],
        #         't_s': optimal_mu[1],
        #         't_e': optimal_mu[2],
        #         't_b': optimal_mu[3],
        #         'd': optimal_mu[4],
        #         'alpha': optimal_mu[5],
        #         'v_x': -result.fun
        #     }
        # else:
        #     raise ValueError("Optimization failed.")
        

        return optimal_mu

# Example usage of the GaitOptimizer class with default bounds
if __name__ == "__main__":
    k_s_average = 0.3 * 1e6  # example value
    k_p = 1.0 * 1e6      # example value
    k_e = 2 * 1e6      # example value

    # Use default bounds
    optimizer = GaitOptimizer()
    # f_s, f_r, f_a, f_e, f_p, cons1, cons4, cons5 = optimizer.validate(k_p, k_s_average, k_e, [7,14,7,0.3, 0.03, np.pi/6])


    # ploting
#     import matplotlib.pyplot as plt
#     k_p_values = np.linspace(0.2, 2, 10) * 1e6

#    # Update to vary penetration velocity from 0.3 to 7 instead of keeping it fixed

#     # Define the range for penetration velocities
#     penetration_velocity_values = np.linspace(0.025, 0.2, 10)

#     # Store results for constraints for each penetration velocity
#     cons1_results = []
#     cons4_results = []
#     cons5_results = []

#     # Loop over different k_p values and penetration velocities
#     for k_p in k_p_values:
#         cons1_kp = []
#         cons4_kp = []
#         cons5_kp = []
#         for penetration_velocity in penetration_velocity_values:
#             _, _, _, _, _, cons1, cons4, cons5 = optimizer.validate(k_p, k_p, k_p, [1,1, 0.03/penetration_velocity, 0.3, 0.03, np.pi / 6])
#             cons1_kp.append(cons1)
#             cons4_kp.append(cons4)
#             cons5_kp.append(cons5)
#         cons1_results.append(cons1_kp)
#         cons4_results.append(cons4_kp)
#         cons5_results.append(cons5_kp)

#     # Plotting
#     plt.figure(figsize=(10, 6))

#     # Plot cons1 vs penetration velocity with different colors for k_p
#     # for idx, k_p in enumerate(k_p_values):
#     #     plt.plot(penetration_velocity_values, cons1_results[idx], 'o-', label=f'k_p={k_p}', color=plt.cm.viridis(idx / len(k_p_values)))

#     # # Plot cons4 vs penetration velocity
#     # for idx, k_p in enumerate(k_p_values):
#     #     plt.plot(penetration_velocity_values, cons4_results[idx], 'x-', label=f'k_p={k_p}', color=plt.cm.plasma(idx / len(k_p_values)))

#     # Plot cons5 vs penetration velocity
#     for idx, k_p in enumerate(k_p_values):
#         plt.plot(penetration_velocity_values, cons4_results[idx], 's-', label=f'k_p={k_p}', color=plt.cm.cividis(idx / len(k_p_values)))

#     plt.title('Constraints vs Penetration Velocity (Varying k_p)')
#     plt.xlabel('Penetration Velocity')
#     plt.ylabel('Constraint Values')
#     plt.legend()
#     plt.grid(True)
#     plt.show()

    control_vector = optimizer.optimize(k_p, k_s_average, k_e)
    # print(control_vector)

    optimizer.validate(k_p, k_s_average, k_e, control_vector)

    # # Example with custom bounds
    # custom_bounds = [(0.5, 4), (0.5, 3), (0.5, 2), (0.5, 2), (0.03, 0.03), (np.pi/6, np.pi/6)]
    # optimizer_custom = GaitOptimizer(bounds=custom_bounds)
    # control_vector_custom = optimizer_custom.optimize(k_p, k_s_average, k_e)
    # print(control_vector_custom)
