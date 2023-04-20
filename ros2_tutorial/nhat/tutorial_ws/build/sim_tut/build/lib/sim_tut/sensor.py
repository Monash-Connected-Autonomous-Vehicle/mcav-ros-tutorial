# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sensor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Bool, '/ebrake_active', 10)

    def listener_callback(self, msg):
        points = pc2.read_points_list(msg)
        min_dist = float("inf")
        for point in points:
            distance = ((point.x)**2 + (point.y)**2 + (point.z)**2) ** 0.5
            # self.get_logger().info(f"{point.x.item()}")
            min_dist = distance if min_dist > distance else min_dist
            if point.z > 0 and abs(point.y) < 2.0 and point.x > 0 and distance < 5.0 and distance > 2.0:
                # self.get_logger().info("STOPPPPPP")
                new_msg = Bool()
                new_msg.data = True
                self.publisher_.publish(new_msg)
                break
        #self.get_logger().info(f"KEEP BALLIN {min_dist}")
        new_msg = Bool()
        new_msg.data = False
        self.publisher_.publish(new_msg)
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
