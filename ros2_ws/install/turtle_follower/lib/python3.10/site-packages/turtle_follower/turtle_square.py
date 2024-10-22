
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleSquare(Node):

    def __init__(self):
        super().__init__('turtle_square')
        
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0
        self.state = 'forward'

    def timer_callback(self):
        twist= Twist()
        
        if self.state == 'forward':

            twist.linear.x = 0.5
            twist.angular.z = 0.0
            self.i += 1
            if self.i >= 10: 
                self.state = 'turn'
                self.i = 0
        elif self.state == 'turn':
            twist.linear.x = 0.0
            twist.angular.z = math.pi / 2 
            self.i += 1
            if self.i >= 2:
                self.state = 'forward'
                self.i = 0
        
        self.publisher_.publish(twist)
        
        self.get_logger().info(f'Turtle is moving: {self.state}')

def main(args=None):
    rclpy.init(args=args)
    
    turtle_square = TurtleSquare()
    
    rclpy.spin(turtle_square)
    turtle_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
