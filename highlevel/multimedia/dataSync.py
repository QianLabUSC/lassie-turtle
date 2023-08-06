
import rclpy
from rclpy.node import Node
import numpy as np
# from traveler_msgs.msg import TravelerMode
from traveler_msgs.msg import TravelerMode
from traveler_msgs.msg import TravelerStatus
from traveler_msgs.msg import TravelerConfig
import csv
import datetime
import os



class TravelerDataSave(Node):

    def __init__(self):
        super().__init__('TravelerProxyer')
           
        self.fieldnames = ['time','toeforce_x','toeforce_y','toe_pos_x',
                           'toe_pos_y','motor0_pos','motor1_pos',
                           'motor0_torque','motor1_torque']
        self.data_flag_subscriber = self.create_subscription(
            TravelerMode,
            '/traveler/start_flag',
            self.data_save,
            10
        )
        self.odrive_status = self.create_subscription(
            TravelerStatus,
            '/traveler/status',
            self.traveler_status_callback,
            3000
        )
        self.filename_subscription = self.create_subscription(
           TravelerConfig,
           '/traveler/config',
           self.filename_callback,
           10
        )
        self.filename = "defaultfilename"
        self.running_scenario = "defaultscenario"
        self.high_data_save_flag = False
        self.high_data_list = []
        print('initialize')

    def filename_callback(self,msg):
        self.filename = msg.filename
        self.running_scenario = msg.running_scenario


    def traveler_status_callback(self, msg):
        
        if(self.high_data_save_flag):
            # print('go to save mdoe')
            rowdict = {
                self.fieldnames[0]: msg.time,
                self.fieldnames[1]: msg.toeforce_x,
                self.fieldnames[2]: msg.toeforce_y,
                self.fieldnames[3]: msg.toe_pos_x,
                self.fieldnames[4]: msg.toe_pos_y,
                self.fieldnames[5]: msg.motor0_pos,
                self.fieldnames[6]: msg.motor1_pos,
                self.fieldnames[7]: msg.motor0_torque,
                self.fieldnames[8]: msg.motor1_torque,
            }
            self.high_data_list.append(rowdict)

    def data_save(self, msg):
        print(msg.traveler_mode)
        self.high_data_save_flag = msg.start_flag
        if(self.high_data_save_flag):
            pass
        else:
            current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.high_filename = self.filename + "_" + current_time + ".csv" # video formats and sizes also depend and vary according to the camera used
            REC_FOLDER = "experiment_records/" + self.running_scenario + "/"
            CURR_FOLDER = os.getcwd()
            PARAENT_FOLDER = os.path.dirname(CURR_FOLDER)
            COMBINED_HIGH_PATH = os.path.join(PARAENT_FOLDER, REC_FOLDER + self.high_filename)
            if not os.path.exists(os.path.dirname(COMBINED_HIGH_PATH)):
                os.makedirs(os.path.dirname(COMBINED_HIGH_PATH))
            # save high data file
            self.file = open(COMBINED_HIGH_PATH, 'w', newline='')
            self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
            self.writer.writeheader()
            for dict in self.high_data_list:
                self.writer.writerow(dict)   
            self.file.close()
            self.get_logger().warn('traveler data saved in folder: "{0}"'.format(REC_FOLDER))

    
    
def main():
    rclpy.init()

    traveler_proxyer = TravelerDataSave() 

    rclpy.spin(traveler_proxyer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traveler_proxyer.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
