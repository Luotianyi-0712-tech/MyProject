import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleCircle(Node):
    def __init__(self):
        super().__init__('turtle_circle')
        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.move_in_circle)

    def move_in_circle(self):
        twist = Twist()
        
        twist.linear.x = 2.0  
        twist.angular.z = 1.0  
    
        self.publisher.publish(twist)
        self.get_logger().info('Turtle is moving')

def main(args=None):
    rclpy.init(args=args) 
    turtle_circle = TurtleCircle()  
    rclpy.spin(turtle_circle)  
    turtle_circle.destroy_node()  
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
