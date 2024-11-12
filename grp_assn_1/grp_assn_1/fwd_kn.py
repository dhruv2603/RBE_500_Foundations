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
import math
import numpy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')
        self.publisher_ = self.create_publisher(String, 'end_pose', 10)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_vals',
            self.calculate_end_pose,
            10)
        self.subscription  # prevent unused variable warning

    def calculate_end_pose(self,msg):
        #Implement the calculation of end effector pose


def main(args=None):
    rclpy.init(args=args)

    forward_kinematics = ForwardKinematics()

    rclpy.spin(forward_kinematics)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    forward_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()