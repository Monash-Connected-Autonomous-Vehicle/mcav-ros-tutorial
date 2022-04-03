import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range

class Task3(Node):

    def __init__(self):
        super().__init__('task3')
        self.ebrake_publisher_ = self.create_publisher(Twist,"cmd_vel",10)
        self.vel_subscriber_ = self.create_subscription(Twist,"cmd_vel", self.vel_callback, 10)
        self.laser_subscriber_ = self.create_subscription(Range, 'laser/range', self.laser_callback, 10)
        self.get_logger().info('task3')
        self.stopping_distance = 0

    def laser_callback(self, msg: Range):
        '''
        Implement logic to stop the car when there is an obstacle within 5 meters of it
        '''
        # self.get_logger().info('distance from object: {}'.format(msg.range))

        if msg.range <= self.stopping_distance:
            message = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=0.0))

            self.ebrake_publisher_.publish(message)
    
    def vel_callback(self, msg: Twist):
        '''
        Implement logic to decide what distance the car should be from an object before it stops
        '''
        self.stopping_distance = ((msg.linear.x ** 2) / 20) + 5

        self.get_logger().info('stopping distance: {}'.format(self.stopping_distance))


def main(args=None):
    rclpy.init(args=args)
    task3 = Task3()
    rclpy.spin(task3)

    task3.destroy_node()
    rclpy.shutdown()

if __name__=="main":
    main()



