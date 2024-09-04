import numpy as np

class TerrainSensor:
    def __init__(self, 
                 torque_constant=0.083, 
                 flipper_length=0.111, 
                 flipper_depth=0.055, 
                 motor_center_to_surface=0.068):
        

        self.torque_constant = torque_constant
        self.flipper_length = flipper_length
        self.flipper_depth = flipper_depth
        self.motor_center_to_surface = motor_center_to_surface

    def estimate_terrain(self, 
                         right_sweeping_pos, 
                         right_adduction_pos, 
                         right_sweeping_curr, 
                         right_adduction_curr, 
                         turtle_state_list):
        # Calculate forces, insertion depths, and terrain properties
        seg_adduction_force_adjust, \
            seg_sweeping_force_adjust, \
                seg_insertion_depth = self.calculate_forces_and_depth(right_sweeping_pos, 
                                                                      right_adduction_pos, 
                                                                      right_sweeping_curr, 
                                                                      right_adduction_curr)
        penetration_slope = self.calculate_penetration_slope(seg_adduction_force_adjust, 
                                                                                           seg_insertion_depth, 
                                                                                           turtle_state_list)
        max_extraction_force = self.calculate_max_extraction_force(seg_adduction_force_adjust, 
                                                                   turtle_state_list)
        avg_shear_force = self.calculate_average_shear_force(seg_sweeping_force_adjust, 
                                                             turtle_state_list)

        return penetration_slope, max_extraction_force, avg_shear_force
    

    def calculate_forces_and_depth(self, sweeping_pos, adduction_pos, sweeping_curr, adduction_curr):
        # Calculate forces
        seg_adduction_force = (adduction_curr * self.torque_constant) / self.flipper_length
        seg_sweeping_force = (sweeping_curr * self.torque_constant) / self.flipper_length

        # Adjust forces using inverse kinematics
        seg_adduction_force_adjust = seg_adduction_force * np.cos(sweeping_pos) * np.cos(adduction_pos) - seg_sweeping_force * np.sin(sweeping_pos) * np.sin(adduction_pos)
        seg_sweeping_force_adjust = seg_sweeping_force * np.cos(sweeping_pos)

        # Calculate insertion depth
        seg_insertion_depth = self.flipper_length * np.sin(adduction_pos) * np.cos(sweeping_pos) - self.flipper_depth * np.cos(adduction_pos) + self.motor_center_to_surface

        return seg_adduction_force_adjust, seg_sweeping_force, seg_insertion_depth
    

    def calculate_penetration_slope(self, seg_insertion_depth_right, seg_right_adduction_force, turtle_state_list):
        # Penetration phase occurs when turtle_state == 2
        segments2 = self.get_segments(turtle_state_list, 2)
        start, end = segments2[-1]
        # Find penetration depth within the limits for the right flipper
        for i in range(start, end + 1):
                    if seg_insertion_depth_right[i] >=0:
                        new_start = i
                        break

        # Use penetration segment to calculate slopes
        x_right = -seg_insertion_depth_right[new_start:end + 1]
        segment_forces_right = seg_right_adduction_force[new_start:end + 1]
        slope_right = np.sum(x_right * segment_forces_right) / np.sum(x_right**2)  # Penetration slope

        return slope_right

    def calculate_max_extraction_force(self, seg_adduction_force_adjust, turtle_state_list):
       
        segments4 = self.get_segments(turtle_state_list, 4)
        start, end = segments4[-1]

        # Find the maximum extraction force
        max_extraction_force = np.max(seg_adduction_force_adjust[start:end])

        return max_extraction_force

    def calculate_average_shear_force(self, seg_sweeping_force_adjust, turtle_state_list):
        segments3 = self.get_segments(turtle_state_list, 3)
        start, end = segments3[-1]
        avg_shear_force = np.max(seg_sweeping_force_adjust[start:end])
        return avg_shear_force


    def get_segments(array, value):
        segments = []
        start = None
        for i, v in enumerate(array):
            if v == value:
                if start is None:
                    start = i
            else:
                if start is not None:
                    segments.append((start, i - 1))
                    start = None
        if start is not None:
            segments.append((start, len(array) - 1))
        return segments