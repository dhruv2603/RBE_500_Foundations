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
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState


def HTM_to_Pose(H):
    position_vec  = H[:3,3]
    rotation_mat  = H[:3,:3]
    rotation = R.from_matrix(rotation_mat)
    quaternion = rotation.as_quat()
    return position_vec,quaternion


class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')
        self.publisher_ = self.create_publisher(Pose, 'end_pose', 10)
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.calculate_end_pose,
            10)
        self.subscription  # prevent unused variable warning

    def calculate_end_pose(self,msg):
        #Implement the calculation of end effector pose
        self.get_logger().info(f"{msg.name}")
        
        
        q1 = msg.position[0]  # stores the values of incoming float32multiarray
        q2 = msg.position[1]
        q3 = msg.position[2]
        q4 = msg.position[3]
        q5 = msg.position[4]   
        '''
        The values of links are given
        and the DH parameters are calculated according to the diagram as follows 
        '''
        l0 = 36.076
        l1 = 60.25
        l2 = 128.0
        l3 = 24.0
        l4 = 124.0
        l5 = 133.4
        
        '''
        Here  A0, A1, A2, A3, A4, are homogenous transformations matrices 
        as calculated in the PDF file
        '''
        "For Link 0"
        A0 = numpy.array([[ 1, 0, 0, 0],
                          [ 0, 1, 0, 0], 
                          [ 0, 0, 1, l0],
                          [ 0,0,0,1]] , dtype= object)


        "For Link 1"
        a1 = 0
        theta1 = q1
        d1 = l1
        alpha1 = -numpy.pi/2
        
        A1 = numpy.array([[ math.cos(theta1), -math.sin(theta1)*math.cos(alpha1),  math.sin(theta1)*math.sin(alpha1), a1*math.cos(theta1)],
                          [ math.sin(theta1),  math.cos(theta1)*math.cos(alpha1), -math.cos(theta1)*math.sin(alpha1), a1*math.sin(theta1)], 
                          [ 0, math.sin(alpha1), math.cos(alpha1), d1],
                          [ 0,0,0,1]] , dtype= object)
        
        "For Link 2"
        a2 = numpy.sqrt(numpy.square(l2) + numpy.square(l3))
        theta2 = -numpy.arctan2(l2,l3) + q2
        d2 = 0 
        alpha2 = 0
        
        A2 = numpy.array([[ math.cos(theta2), -math.sin(theta2)*math.cos(alpha2),  math.sin(theta2)*math.sin(alpha2), a2*math.cos(theta2)],
                          [ math.sin(theta2),  math.cos(theta2)*math.cos(alpha2), -math.cos(theta2)*math.sin(alpha2), a2*math.sin(theta2)], 
                          [ 0, math.sin(alpha2), math.cos(alpha2), d2],
                          [ 0,0,0,1]] , dtype= object)
        
        "For Link 3"
        a3 = l4
        theta3 = numpy.arctan2(l2,l3) + q3
        d3 = 0
        alpha3 = 0
        
        A3 = numpy.array([[ math.cos(theta3), -math.sin(theta3)*math.cos(alpha3),  math.sin(theta3)*math.sin(alpha3), a3*math.cos(theta3)],
                          [ math.sin(theta3),  math.cos(theta3)*math.cos(alpha3), -math.cos(theta3)*math.sin(alpha3), a3*math.sin(theta3)], 
                          [ 0, math.sin(alpha3), math.cos(alpha3), d3],
                          [ 0,0,0,1]] , dtype= object)
        
        "For Link 4"
        a4 = l5
        theta4 = q4
        d4 = 0
        alpha4 = 0
        
        A4 = numpy.array([[ math.cos(theta4), -math.sin(theta4)*math.cos(alpha4),  math.sin(theta4)*math.sin(alpha4), a4*math.cos(theta4)],
                          [ math.sin(theta4),  math.cos(theta4)*math.cos(alpha4), -math.cos(theta4)*math.sin(alpha4), a4*math.sin(theta4)], 
                          [ 0, math.sin(alpha4), math.cos(alpha4), d4],
                          [ 0,0,0,1]] , dtype= object)

        
        
        '''
        Here we multiply the all transformation matrices to get the final one 
        '''

        H = numpy.matmul(numpy.matmul(numpy.matmul(numpy.matmul(A0,A1),A2),A3),A4)

        print(H)
        position,angle = HTM_to_Pose(H)

        '''
        Publish the pose to end_pose topic
        '''
        msg_pub = Pose()    
        msg_pub.position.x  = position[0]
        msg_pub.position.y  = position[1]
        msg_pub.position.z  = position[2]
        msg_pub.orientation.x = angle[0]
        msg_pub.orientation.y = angle[1]
        msg_pub.orientation.z = angle[2]
        msg_pub.orientation.w = angle[3]
        self.publisher_.publish(msg_pub)
        


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