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
        
    def vel_callback(self,msg:Twist):
        
        self.vx=msg.linear.x
        self.vy=msg.linear.y
        self.wz=msg.angular.z
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        
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

    


# import subprocess
# import sys
# import termios
# import tty

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray


# class teleop_control(Node):
#     def __init__(self):
#         super().__init__("teleop_control")

#         self.publisher = self.create_publisher(
#             Float64MultiArray,
#             "/forward_position_controller/commands",
#             10
#         )

#         self.active_process = None

#         print("\nPETBOT KEYBOARD TELEOP")
#         print("W: Forward | S: Backward | A: Left | D: Right")
#         print("X: Stop | Q: Quit")
#         print("Press a movement key, then enter number of steps.")

#     def publish_neutral_pose(self):
#         message = Float64MultiArray()
#         message.data = [
#             -0.4, 0.0, 0.0,
#              0.4, 0.0, 0.0,
#              0.4, 0.0, 0.0,
#             -0.4, 0.0, 0.0
#         ]
#         self.publisher.publish(message)

#     def stop_active_gait(self):
#         if self.active_process is not None:
#             if self.active_process.poll() is None:
#                 self.active_process.terminate()

#                 try:
#                     self.active_process.wait(timeout=2)
#                 except subprocess.TimeoutExpired:
#                     self.active_process.kill()

#         self.active_process = None
#         self.publish_neutral_pose()

#     def ask_steps(self):
#         while True:
#             try:
#                 steps = int(input("\nEnter number of steps: "))

#                 if steps >= 1:
#                     return steps

#                 print("Enter 1 or more.")

#             except ValueError:
#                 print("Enter a whole number, for example 3.")

#     def run_gait(self, script_name, script_input, movement_name):
#         self.stop_active_gait()
#         self.get_logger().info(f"Starting {movement_name}")

#         self.active_process = subprocess.Popen(
#             ["ros2", "run", "pet_bot_test5", script_name],
#             stdin=subprocess.PIPE,
#             text=True
#         )

#         self.active_process.stdin.write(script_input)
#         self.active_process.stdin.flush()

#     def get_key(self):
#         terminal_settings = termios.tcgetattr(sys.stdin)

#         try:
#             tty.setraw(sys.stdin.fileno())
#             key = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(
#                 sys.stdin,
#                 termios.TCSADRAIN,
#                 terminal_settings
#             )

#         return key.lower()

#     def run(self):
#         try:
#             while rclpy.ok():
#                 key = self.get_key()

#                 if key == "w":
#                     steps = self.ask_steps()
#                     self.run_gait(
#                         "forward_gait.py",
#                         f"{steps}\n",
#                         f"FORWARD for {steps} steps"
#                     )

#                 elif key == "s":
#                     steps = self.ask_steps()
#                     self.run_gait(
#                         "backward_gait.py",
#                         f"{steps}\n",
#                         f"BACKWARD for {steps} steps"
#                     )

#                 elif key == "a":
#                     steps = self.ask_steps()
#                     script_steps = max(0, steps - 1)
#                     self.run_gait(
#                         "sideward_gait.py",
#                         f"{script_steps}\n1\n",
#                         f"LEFT for {steps} steps"
#                     )

#                 elif key == "d":
#                     steps = self.ask_steps()
#                     script_steps = max(0, steps - 1)
#                     self.run_gait(
#                         "sideward_gait.py",
#                         f"{script_steps}\n-1\n",
#                         f"RIGHT for {steps} steps"
#                     )

#                 elif key == "x":
#                     self.stop_active_gait()
#                     print("\nStopped.")

#                 elif key == "q":
#                     self.stop_active_gait()
#                     print("\nTeleop closed.")
#                     break

#         except KeyboardInterrupt:
#             self.stop_active_gait()


# def main():
#     rclpy.init()
#     node = teleop_control()

#     try:
#         node.run()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
# if __name__ == "__main__":
#    main()