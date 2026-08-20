#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from forward_gait import HybridIKTrot

from backward_gait import HybridIKTrot_Backward

from sideward_gait import SideWalk


class teleop_control(Node):
    def __init__(self):
        super().__init__('teleop_controller')
        self.forward_gait=HybridIKTrot(steps=11111)
        self.backward_gait=HybridIKTrot_Backward(steps=11111)
        self.sideward_gait=SideWalk(steps=11111)
        
        self.vx=0.0
        self.vy=0.0
        self.wz=0.0
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        
    
        self.pub=self.create_publisher(Float64MultiArray,'/forward_position_controller/commands',10)
        self.create_subscription(Twist,'/cmd_vel',self.vel_callback,10)
        
        self.rate=50.0
        self.timer=self.create_timer(1.0/self.rate,self.update)
        
    def vel_callback(self,msg):
        msg=Twist()
        self.vx=msg.linear.x
        self.vy=msg.linear.y
        self.wz=msg.angular.z
        
    def update(self):
        now=self.get_clock().now().nanoseconds * 1e-9    
        time_elapsed=now-self.t0
        if time_elapsed >0.45:
            self.vx=0.0
            self.vy=0.0
            self.wz=0.0
            
        if self.vx>0.1:
            self.forward_gait.update()
        elif self.vx<-0.1:
            self.backward_gait.update()
        elif self.vy>0.1:
            self.sideward_gait.update(1)
        elif self.vy<-0.1:
            self.sideward_gait.update(-1)
        else:
            self.forward_gait.publish_neutral()    
                    
def main():
    rclpy.init()
    node = teleop_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()                 