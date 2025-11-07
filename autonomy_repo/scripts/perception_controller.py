#!/usr/bin/python3

import rclpy
from time import sleep

from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl

class PerceptionController(BaseController):
    def __init__(self):
        super().__init__('perception_controller')
        self.declare_parameter('active', True)
        
        self.timer = self.create_timer(5.0, self.toggle_active)
 
    @property
    def active(self) -> bool:
        return self.get_parameter('active').get_parameter_value().bool_value
    
    def toggle_active(self):
        new = not self.active
        self.set_parameters([rclpy.Parameter("active", value=new)])
    
    def compute_control(self) -> TurtleBotControl:
        control_msg = TurtleBotControl()
        
        if self.active:
            control_msg.omega = 0.5
            control_msg.v = 0.0
        else:
            control_msg.omega = 0.0
            control_msg.v = 0.0
                        
        return control_msg

def main():
    rclpy.init()
    node = PerceptionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()

