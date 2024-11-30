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
from rclpy.qos import QoSProfile
import math
import numpy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from custom_interfaces.srv import JointVel
from sensor_msgs.msg import JointState
import time

class JointVeltoTwist(Node):
    qos = QoSProfile(depth=10)
    def __init__(self):
        super().__init__('calculate_twist_service')
        self.joint_pos = self.create_subscription(JointState,'joint_states',self.joint_pos_callback,self.qos)
        self.srv = self.create_service(JointVel,'cal_end_eff_twist',self.cal_end_eff_twist)
        self.joint_pos_msg = []
        
        self.l0 = 36.076
        self.l1 = 60.25
        self.l2 = 128.0
        self.l3 = 24.0
        self.l4 = 124.0
        self.l5 = 133.4

    def joint_pos_callback(self,msg):
        self.joint_pos_msg = msg.position


    def transformation_matrix(self,a,theta,d,alpha):
        A = numpy.array([
            [numpy.cos(theta), -numpy.sin(theta)*numpy.cos(alpha), numpy.sin(theta)*numpy.sin(alpha), a*numpy.cos(theta)],
            [numpy.sin(theta), numpy.cos(theta)*numpy.cos(alpha), -numpy.cos(theta)*numpy.sin(alpha), a*numpy.sin(theta)],
            [0,                numpy.sin(alpha),                   numpy.cos(alpha),                  d],
            [0, 0, 0, 1],
        ])
        return A
    
    def cal_end_eff_twist(self,request,response):
        joint_velocities = request.joint_vel.data
        v1 = joint_velocities[0]
        v2 = joint_velocities[1]
        v3 = joint_velocities[2]
        v4 = joint_velocities[3]
        q_dot = numpy.array([[v1],
                            [v2],
                            [v3],
                            [v4]])

        q1 = self.joint_pos_msg[0]
        q2 = self.joint_pos_msg[1]
        q3 = self.joint_pos_msg[2]
        q4 = self.joint_pos_msg[3]

        #Calculate Homogeneous transformation Matrices
        A0 = numpy.array([[ 1, 0, 0, 0],
                          [ 0, 1, 0, 0], 
                          [ 0, 0, 1, self.l0],
                          [ 0,0,0,1]])
        
        
        "For Link 1"
        a1 = 0
        theta1 = q1
        d1 = self.l1
        alpha1 = -numpy.pi/2
        A1 = self.transformation_matrix(a1,theta1,d1,alpha1)
        H1 = numpy.matmul(A1,A0)

        "For Link 2"
        a2 = numpy.sqrt(numpy.square(self.l2) + numpy.square(self.l3))
        theta2 = -numpy.arctan2(self.l2,self.l3) + q2
        d2 = 0 
        alpha2 = 0
        A2 = self.transformation_matrix(a2,theta2,d2,alpha2)
        H2 = numpy.matmul(H1,A2)

        "For Link 3"
        a3 = self.l4
        theta3 = numpy.arctan2(self.l2,self.l3) + q3
        d3 = 0
        alpha3 = 0
        A3 = self.transformation_matrix(a3,theta3,d3,alpha3)
        H3 = numpy.matmul(H2,A3)

        "For Link 4"
        a4 = self.l5
        theta4 = q4
        d4 = 0
        alpha4 = 0
        A4 = self.transformation_matrix(a4,theta4,d4,alpha4)
        H4 = numpy.matmul(H3,A4)

        #Calculate origins

        o_gnd = numpy.array([
                        [0],
                        [0],
                        [0]])
        o0 = A0[:3,3]
        o1 = H1[:3,3]
        o2 = H2[:3,3]
        o3 = H3[:3,3]
        o4 = H4[:3,3]

        z_gnd = numpy.array([
                        [0],
                        [0],
                        [1]])
        z0 = A0[:3,2]
        z1 = H1[:3,2]
        z2 = H2[:3,2]
        z3 = H3[:3,2]
        z4 = H4[:3,2]

        #Calculate Jacobians

        Jv1 = numpy.cross(z0,o4 -o0).reshape(3,1)
        Jw1 = z0.reshape(3,1)
        J1 = numpy.vstack([Jv1,Jw1])


        Jv2 = numpy.cross(z1,o4 -o1).reshape(3,1)
        Jw2 = z1.reshape(3,1)
        J2 = numpy.vstack([Jv2,Jw2])

        Jv3 = numpy.cross(z2,o4 -o2).reshape(3,1)
        Jw3 = z2.reshape(3,1)
        J3 = numpy.vstack([Jv3,Jw3])

        Jv4 = numpy.cross(z3,o4 -o3).reshape(3,1)
        Jw4 = z3.reshape(3,1)
        J4 = numpy.vstack([Jv4,Jw4])

        J = numpy.hstack([J1,J2,J3,J4])

        twist = numpy.matmul(J,q_dot)
        self.get_logger().warning(f"twist type: {twist.dtype}")
        response.end_eff_twist.linear.x = float(twist[0])
        response.end_eff_twist.linear.y = float(twist[1])
        response.end_eff_twist.linear.z = float(twist[2])
        response.end_eff_twist.angular.x = float(twist[3])
        response.end_eff_twist.angular.y = float(twist[4])
        response.end_eff_twist.angular.z = float(twist[5])
        return response


def main(args=None):
    rclpy.init(args=args)

    joint_vel_to_twist_srv = JointVeltoTwist()

    rclpy.spin(joint_vel_to_twist_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
