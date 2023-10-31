from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import sys
import math
import time
class Happy(Node):

     def __init__(self):
         # Creates a node with name 'turtlebot_controller' and make sure it is a
         # unique node (using anonymous=True).
         super().__init__('happy')
 
         # Publisher which will publish to the topic '/turtle1/cmd_vel'.
         self.velocity_publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)

         self.timer = self.create_timer(0.5, self.move)

     def move(self):
         """Moves the turtle to the goal."""
         
         vel_msg = Twist()
         
         vel_msg.linear.x = 0.0
         vel_msg.angular.z = 1.046/2
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 1.0
         vel_msg.angular.z = 0.0
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 0.0
         vel_msg.angular.z = -2*1.046
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 1.0
         vel_msg.angular.z = 0.0
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 0.0
         vel_msg.angular.z = -2*1.046
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 1.0
         vel_msg.angular.z = 0.0
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 0.0
         vel_msg.angular.z = -2*1.046
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 0.0
         vel_msg.angular.z = 0.0
         self.velocity_publisher.publish(vel_msg)
         time.sleep(1)

         
 
         
def main(args=None):
    rclpy.init(args=args)
    x = Happy()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()