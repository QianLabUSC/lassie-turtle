#!/usr/bin/env python3
import sys
import csv
import os

sys.argv = [sys.argv[0]]


import threading
import time
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import matplotlib.pyplot as plt








class ControlNodeTurtle(Node):
    def __init__(self, gait_optimizer, terrain_sensor):
        super().__init__('control_node')
        self.id = "turtle"
        self.publisher_ = self.create_publisher(Float64MultiArray, '/Gui_information', 10)
        self.previous_turtle_state = -1
        self.turtle_subscriber = self.create_subscription(
            Float64MultiArray,
            '/robot_state',
            self.turtle_status,
            10)

        self.Optitrack_State = self.create_subscription(
            Pose,
            '/optitrack_body',
            self.OptitrackState,
            10)
        self.LeftFlipper_State = self.create_subscription(
            Pose,
            '/optitrack_left_flipper',
            self.LeftFlipperState,
            10)
        self.RightFlipper_State = self.create_subscription(
            Pose,
            '/optitrack_right_flipper',
            self.RightFlipperState,
            10)
        

        ## some parameters to specify here
        self.tab_control = ["Preset_gait" , "Optimized_gait"]
        self.gait_optimizer_ = gait_optimizer
        self.terrain_sensor_ = terrain_sensor

        
        
        # motor information stream
        self.leftadduction_pos = 0.0
        self.leftsweeping_pos = 0.0
        self.rightadduction_pos = 0.0
        self.rightsweeping_pos = 0.0
        self.leftadduction_curr = 0.0
        self.leftsweeping_curr = 0.0
        self.rightadduction_curr = 0.0
        self.rightsweeping_curr = 0.0

    

        # terrain & other information stream
        '''
        something '''
        
        
        # optitrack information stream
        self.OptitrackPosition_x = 0
        self.OptitrackPosition_y = 0
        self.OptitrackPosition_z = 0
        self.OptitrackOrientation_x = 0
        self.OptitrackOrientation_y = 0
        self.OptitrackOrientation_z = 0
        self.OptitrackOrientation_w = 0

        self.LeftFlipperPosition_x = 0
        self.LeftFlipperPosition_y = 0
        self.LeftFlipperPosition_z = 0
        self.LeftFlipperOrientation_x = 0
        self.LeftFlipperOrientation_y = 0
        self.LeftFlipperOrientation_z = 0
        self.LeftFlipperOrientation_w = 0

        self.RightFlipperPosition_x = 0
        self.RightFlipperPosition_y = 0
        self.RightFlipperPosition_z = 0
        self.RightFlipperOrientation_x = 0
        self.RightFlipperOrientation_y = 0
        self.RightFlipperOrientation_z = 0
        self.RightFlipperOrientation_w = 0

        # state information
        self.turtle_state = 0


        # print('aefasdf')
        self.start_time = time.time()
        self.time_list = []
        # motor information 
        self.leftadduction_pos_array = []
        self.leftsweeping_pos_array = []
        self.rightadduction_pos_array = []
        self.rightsweeping_pos_array = []
        self.leftadduction_curr_array = []
        self.leftsweeping_curr_array = []
        self.rightadduction_curr_array = []
        self.rightsweeping_curr_array = []
    


        # terrain & other information 
        '''
        something '''
    


        # optitrack information 
        self.OptitrackPosition_x_list = []
        self.OptitrackPosition_y_list = []
        self.OptitrackPosition_z_list = []
        self.OptitrackOrientation_x_list = []
        self.OptitrackOrientation_y_list = []
        self.OptitrackOrientation_z_list = []
        self.OptitrackOrientation_w_list = []

        self.LeftFlipperPosition_x_list = []
        self.LeftFlipperPosition_y_list = []
        self.LeftFlipperPosition_z_list = []
        self.LeftFlipperOrientation_x_list = []
        self.LeftFlipperOrientation_y_list = []
        self.LeftFlipperOrientation_z_list = []
        self.LeftFlipperOrientation_w_list = []

        self.RightFlipperPosition_x_list = []
        self.RightFlipperPosition_y_list = []
        self.RightFlipperPosition_z_list = []
        self.RightFlipperOrientation_x_list = []
        self.RightFlipperOrientation_y_list = []
        self.RightFlipperOrientation_z_list = []
        self.RightFlipperOrientation_w_list = []
        self.insertion_depth_list = []
        self.insertion_depth = 0.03
        self.turtle_state_list = []

        self.fpx = None
        self.fpy = None
        self.fpz = None
        self.fp = None
        self.fig4 = None
        self.fig5 = None
        self.fig6 = None
        self.speed_px = None
        self.speed_py = None
        self.speed_p = None
        self.ppp = None
        self.pp = None

        self.fig4, self.fp = plt.subplots()
        self.fp.set_title('displacements Measurements vs Time', fontsize=10)
        self.fp.set_xlabel('Time(t)', fontsize=10)
        self.fp.set_ylabel('Distance(m)', fontsize=10)
        self.fp.grid(True)
        self.fpx, = self.fp.plot([],[], 'r', linewidth=3)
        self.fpy, = self.fp.plot([],[], 'g', linewidth=3)
        self.fpz, = self.fp.plot([],[], 'b', linewidth=3)
        self.fp.legend(['x','y','z'])
        self.fig5, self.speed_p = plt.subplots()
        self.speed_p.set_title('Speed vs time', fontsize=10)
        self.speed_p.set_xlabel('Time(t)', fontsize=10)
        self.speed_p.set_ylabel('Speed(m/s)', fontsize=10)
        self.speed_p.grid(True)
        self.speed_px, = self.speed_p.plot([],[], 'r', linewidth=3)
        self.speed_py, = self.speed_p.plot([],[], 'g', linewidth=3)
        self.speed_p.legend(['x','y'])
        self.fig6, self.pp = plt.subplots()
        self.pp.set_title('position trajectories', fontsize=10)
        self.pp.set_xlabel('X', fontsize=10)
        self.pp.set_ylabel('Y', fontsize=10)
        self.pp.grid(True)
        self.ppp, = self.pp.plot([],[], 'r', linewidth=3)

    def calibrate(self):
        self.start_time = time.time()
        self.time_list = []
        # other information & terrain
        '''
        something '''
        


        # motor information 
        self.leftadduction_pos_array = []
        self.leftsweeping_pos_array = []
        self.rightadduction_pos_array = []
        self.rightsweeping_pos_array = []
        self.leftadduction_curr_array = []
        self.leftsweeping_curr_array = []
        self.rightadduction_curr_array = []
        self.rightsweeping_curr_array = []
        
        # optitrack information
        self.OptitrackPosition_x_list = []
        self.OptitrackPosition_y_list = []
        self.OptitrackPosition_z_list = []
        self.OptitrackOrientation_x_list = []
        self.OptitrackOrientation_y_list = []
        self.OptitrackOrientation_z_list = []
        self.OptitrackOrientation_w_list = []


        self.LeftFlipperPosition_x_list = []
        self.LeftFlipperPosition_y_list = []
        self.LeftFlipperPosition_z_list = []
        self.LeftFlipperOrientation_x_list = []
        self.LeftFlipperOrientation_y_list = []
        self.LeftFlipperOrientation_z_list = []
        self.LeftFlipperOrientation_w_list = []


        self.RightFlipperPosition_x_list = []
        self.RightFlipperPosition_y_list = []
        self.RightFlipperPosition_z_list = []
        self.RightFlipperOrientation_x_list = []
        self.RightFlipperOrientation_y_list = []
        self.RightFlipperOrientation_z_list = []
        self.RightFlipperOrientation_w_list = []

        self.insertion_depth_list = []
        self.insertion_depth = 0.03


        self.turtle_state_list = []




    def get_fp(self):
        return self.fp
    
    def get_pp(self):
        return self.pp
    
    def get_speed_p(self):
        return self.speed_p


    def update_plot(self):
        self.fpx.set_xdata(self.time_list)
        self.fpx.set_ydata(self.OptitrackPosition_x_list)
        self.fpy.set_xdata(self.time_list)
        self.fpy.set_ydata(self.OptitrackPosition_y_list)
        self.fpz.set_xdata(self.time_list)
        self.fpz.set_ydata(self.OptitrackPosition_z_list)
        # print(self.time_list, self.force_list_x)
        self.fp.relim()
        self.fp.autoscale_view()
        self.fig4.canvas.draw()
        self.fig4.canvas.flush_events()


        self.speed_px.set_xdata(self.time_list)
        self.speed_px.set_ydata(self.turtle_state_list)
        self.speed_py.set_xdata(self.time_list)
        self.speed_py.set_ydata(self.turtle_state_list)
        self.speed_p.relim()
        self.speed_p.autoscale_view()
        self.fig5.canvas.draw()
        self.fig5.canvas.flush_events()


        self.ppp.set_xdata(self.turtle_state_list)
        self.ppp.set_ydata(self.turtle_state_list)
        self.pp.relim()
        self.pp.autoscale_view()
        self.fig6.canvas.draw()
        self.fig6.canvas.flush_events()
      
    def update_force_plot(self, real_time_plot_active, updateplotflag):
        if (real_time_plot_active and updateplotflag):
            self.update_plot()
    def update_force_data(self, updateplotflag):
        # self.run_time += 1
        # # print(self.run_time)
        # print('frequency: ', self.run_time/(time.time()-self.start_time) )       
        if(updateplotflag):
            current_time = time.time() - self.start_time
            self.time_list.append(current_time)
            self.insertion_depth_list.append(self.insertion_depth)
            self.leftadduction_pos_array.append(self.leftadduction_pos)
            self.leftsweeping_pos_array.append(self.leftsweeping_pos)  
            self.rightadduction_pos_array.append(self.rightadduction_pos)  
            self.rightsweeping_pos_array.append(self.rightsweeping_pos)  
            self.leftadduction_curr_array.append(self.leftadduction_curr)  
            self.leftsweeping_curr_array.append(self.leftsweeping_curr)  
            self.rightadduction_curr_array.append(self.rightadduction_curr) 
            self.rightsweeping_curr_array.append(self.rightsweeping_curr) 

            
            
           
            self.OptitrackPosition_x_list.append(self.OptitrackPosition_x)
            self.OptitrackPosition_y_list.append(self.OptitrackPosition_y)
            self.OptitrackPosition_z_list.append(self.OptitrackPosition_z)
            self.OptitrackOrientation_x_list.append(self.OptitrackOrientation_x)
            self.OptitrackOrientation_y_list.append(self.OptitrackOrientation_y)
            self.OptitrackOrientation_z_list.append(self.OptitrackOrientation_z)
            self.OptitrackOrientation_w_list.append(self.OptitrackOrientation_w)

            self.LeftFlipperPosition_x_list.append(self.LeftFlipperPosition_x)
            self.LeftFlipperPosition_y_list.append(self.LeftFlipperPosition_y)
            self.LeftFlipperPosition_z_list.append(self.LeftFlipperPosition_z)
            self.LeftFlipperOrientation_x_list.append(self.LeftFlipperOrientation_x)
            self.LeftFlipperOrientation_y_list.append(self.LeftFlipperOrientation_y)
            self.LeftFlipperOrientation_z_list.append(self.LeftFlipperOrientation_z)
            self.LeftFlipperOrientation_w_list.append(self.LeftFlipperOrientation_w)


            self.RightFlipperPosition_x_list.append(self.RightFlipperPosition_x)
            self.RightFlipperPosition_y_list.append(self.RightFlipperPosition_y)
            self.RightFlipperPosition_z_list.append(self.RightFlipperPosition_z)
            self.RightFlipperOrientation_x_list.append(self.RightFlipperOrientation_x)
            self.RightFlipperOrientation_y_list.append(self.RightFlipperOrientation_y)
            self.RightFlipperOrientation_z_list.append(self.RightFlipperOrientation_z)
            self.RightFlipperOrientation_w_list.append(self.RightFlipperOrientation_w)
            
        
            self.turtle_state_list.append(self.turtle_state)


    def download_data(self, file_name, node_id, real_time_plot, gui_message ):
        if not os.path.exists('./experiment_data/turtle'):
            os.makedirs('./experiment_data/turtle')
        path = "./experiment_data/turtle/" + file_name + ".csv"
        with open(path, 'w', newline='') as f:
            
            writer=csv.writer(f)
          
            writer.writerow(["scenario","real_time_plot", "lateral_angle_range","drag_speed", "wiggle_time", "servo_speed", "extraction_height", "wiggle frequency",
                            "insertion_angle", "wiggle_amplitude"])
            writer.writerow([node_id, real_time_plot] + list(gui_message[2:]))
            
            writer.writerow(["time", "turtle_state",
                                "leftadduction_pos", "leftsweeping_pos",
                                "rightadduction_pos", "rightsweeping_pos",
                                "leftadduction_curr", "leftsweeping_curr",
                                "rightadduction_curr", "rightsweeping_curr", "insertiondepth",
                                "OptitrackPosition_x","OptitrackPosition_y","OptitrackPosition_z",
                                "OptitrackOrientation_x", "OptitrackOrientation_y", "OptitrackOrientation_z", "OptitrackOrientation_w",
                                "LeftFlipperPosition_x","LeftFlipperPosition_y","LeftFlipperPosition_z",
                                "LeftFlipperOrientation_x", "LeftFlipperOrientation_y", "LeftFlipperOrientation_z", "LeftFlipperOrientation_w",
                                "RightFlipperPosition_x","RightFlipperPosition_y","RightFlipperPosition_z",
                                "RightFlipperOrientation_x", "RightFlipperOrientation_y", "RightFlipperOrientation_z", "RightFlipperOrientation_w"
                                ])
            
            
            for i in range(len(self.time_list)):
                writer.writerow([self.time_list[i],  self.turtle_state_list[i],
                                self.leftadduction_pos_array[i], self.leftsweeping_pos_array[i],
                                self.rightadduction_pos_array[i], self.rightsweeping_pos_array[i],
                                self.leftadduction_curr_array[i], self.leftsweeping_curr_array[i],
                                self.rightadduction_curr_array[i], self.rightsweeping_curr_array[i],self.insertion_depth_list[i],
                                self.OptitrackPosition_x_list[i],self.OptitrackPosition_y_list[i],self.OptitrackPosition_z_list[i],
                                self.OptitrackOrientation_x_list[i], self.OptitrackOrientation_y_list[i],
                                self.OptitrackOrientation_z_list[i], self.OptitrackOrientation_w_list[i], 
                                self.LeftFlipperPosition_x_list[i],self.LeftFlipperPosition_y_list[i],self.LeftFlipperPosition_z_list[i],
                                self.LeftFlipperOrientation_x_list[i], self.LeftFlipperOrientation_y_list[i],
                                self.LeftFlipperOrientation_z_list[i], self.LeftFlipperOrientation_w_list[i],
                                self.RightFlipperPosition_x_list[i],self.RightFlipperPosition_y_list[i],self.RightFlipperPosition_z_list[i],
                                self.RightFlipperOrientation_x_list[i], self.RightFlipperOrientation_y_list[i],
                                self.RightFlipperOrientation_z_list[i], self.RightFlipperOrientation_w_list[i]            
                                ])

    def turtle_status(self, msg):
        self.turtle_state = msg.data[0]
        self.leftadduction_pos = msg.data[1]
        self.leftsweeping_pos = msg.data[2]
        self.rightadduction_pos = msg.data[3]
        self.rightsweeping_pos = msg.data[4]
        self.leftadduction_curr = msg.data[5]
        self.leftsweeping_curr = msg.data[6]
        self.rightadduction_curr = msg.data[7]
        self.rightsweeping_curr = msg.data[8]


    def OptitrackState(self, msg):
        print(msg)
        self.OptitrackPosition_x = msg.position.x
        self.OptitrackPosition_y = msg.position.y
        self.OptitrackPosition_z = msg.position.z
        self.OptitrackOrientation_x = msg.orientation.x
        self.OptitrackOrientation_y = msg.orientation.y
        self.OptitrackOrientation_z = msg.orientation.z
        self.OptitrackOrientation_w = msg.orientation.w
    def LeftFlipperState(self, msg):
        self.LeftFlipperPosition_x = msg.position.x
        self.LeftFlipperPosition_y = msg.position.y
        self.LeftFlipperPosition_z = msg.position.z
        self.LeftFlipperOrientation_x = msg.orientation.x
        self.LeftFlipperOrientation_y = msg.orientation.y
        self.LeftFlipperOrientation_z = msg.orientation.z
        self.LeftFlipperOrientation_w = msg.orientation.w


    def RightFlipperState(self, msg):
        self.RightFlipperPosition_x = msg.position.x
        self.RightFlipperPosition_y = msg.position.y
        self.RightFlipperPosition_z = msg.position.z
        self.RightFlipperOrientation_x = msg.orientation.x
        self.RightFlipperOrientation_y = msg.orientation.y
        self.RightFlipperOrientation_z = msg.orientation.z
        self.RightFlipperOrientation_w = msg.orientation.w
        
        


    def start_preset_gait(self, msg):
        self.publisher_.publish(msg)


    def start_gait_optimization(self, msg):
        self.control_gait = msg
        self.publisher_.publish(msg)

        # also start or stop the gait optimizer
        if(msg.data[0] == True):
            self.timer = self.create_timer(0.1, self.optimize_gait)
        else:
            if self.timer is not None:
                self.timer.cancel()

    def optimize_gait(self):
        self.tem_message = Float64MultiArray()
        self.tem_message.data.append(True)                                                 
        self.tem_message.data.append(6) #this does matter, since in high controller, check this value, and assign different gait parameters
        # check if it is in a new back phase
        
        if self.turtle_state == 1 and self.previous_turtle_state != 1:
            
            self.k_p_, self.k_s_, self.k_e_ = self.terrain_sensor_.estimate_terrain(
                                                np.array(self.rightsweeping_pos_array),
                                                np.array(self.rightadduction_pos_array),
                                                np.array(self.rightsweeping_curr_array),
                                                -np.array(self.rightadduction_curr_array),
                                                np.array(self.turtle_state_list),
                                                self.insertion_depth)
                                            
            print("optimizing")
            print(self.control_gait.data[2:8])
            print(self.k_s_)
            self.gait_u_= self.gait_optimizer_.optimize(self.k_p_, self.k_s_, self.k_e_, self.control_gait.data[2:8])
            self.insertion_depth = self.gait_u_[4]
            print(self.gait_u_)
            self.tem_message.data.extend(self.gait_u_)
            self.publisher_.publish(self.tem_message)
            print(self.tem_message)
            self.control_gait = self.tem_message
        self.previous_turtle_state = self.turtle_state


