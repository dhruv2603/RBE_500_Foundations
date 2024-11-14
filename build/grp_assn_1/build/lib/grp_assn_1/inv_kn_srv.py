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
# from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R
from custom_interfaces.srv import InvKin
from open_manipulator_msgs.srv import SetJointPosition

goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]

# def HTM_to_Pose(H):
#     position_vec  = H[:3,3]
#     rotation_mat  = H[:3,:3]
#     rotation = R.from_matrix(rotation_mat)
#     quaternion = rotation.as_quat()
#     return position_vec,quaternion

class InverseKinematicsService(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_service')
        self.srv = self.create_service(InvKin,'calc_inv_kin',self.calc_inv_kn)
        self.goal_joint_space_req = SetJointPosition.Request()
        self.tool_control_req = SetJointPosition.Request()

    def calc_inv_kn(self,request,response, path_time):
        x = request.pose.position.x
        y = request.pose.position.y
        z = request.pose.position.z

        self.get_logger().info(f"Received pose: {request.pose.position.x}, {request.pose.position.y}, {request.pose.position.z}, "
                               f"{request.pose.orientation.x}, {request.pose.orientation.y}, {request.pose.orientation.z}, {request.pose.orientation.w}")
        self.get_logger().info('Incoming request\nx:%d y:%d z:%d'%(x,y,z))

        # q = Quaternion()
        # q[0] = request.pose.orientation.x
        # q[1] = request.pose.orientation.y
        # q[2] = request.pose.orientation.z
        # q[3] = request.pose.orientation.w

        l0 = 36.076
        l1 = 60.25
        l2 = 128.0
        l3 = 24.0
        l4 = 124.0
        l5 = 133.4

        theta_1 = numpy.arctan2(y,x)
        
        r  = numpy.sqrt(numpy.square(x) + numpy.square(y))
        r2 = l0 + l1 - z - l5
        D  = (numpy.square(l2) + numpy.square(l3) + numpy.square(l4) - numpy.square(r) - numpy.square(r2))/(2*l4*numpy.sqrt(numpy.square(l2) + numpy.square(l3)))

        theta_y = numpy.arctan2(numpy.sqrt(1 - numpy.square(D)),D)
        theta_v = numpy.arctan2(l3,l2)
        
        theta_3 = numpy.pi/2 + theta_v - theta_y

        D2 = (numpy.square(r) + numpy.square(r2) + numpy.square(l2) + numpy.square(l3) - numpy.square(l4))/(2*numpy.sqrt(numpy.square(r) + numpy.square(r2))*numpy.sqrt(numpy.square(l2) + numpy.square(l3)))

        theta_z = numpy.arctan2(numpy.sqrt(1 - numpy.square(D2)),D2)
        theta_w = numpy.arctan2(r,r2)

        theta_2 = numpy.pi - theta_v - theta_z - theta_w

        theta_4 = numpy.pi/2 - theta_2 - theta_3

        # msg = Float32MultiArray()
        # response = msg.data
        response.joint_vals.data = [theta_1, theta_2, theta_3, theta_4]

        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.goal_joint_space_req.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

        

        return response




def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_service = InverseKinematicsService()

    rclpy.spin(inverse_kinematics_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
