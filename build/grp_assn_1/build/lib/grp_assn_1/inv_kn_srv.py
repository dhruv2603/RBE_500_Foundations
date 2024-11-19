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
# from scipy.spatial.transform import Rotation as R
from custom_interfaces.srv import InvKin
# from open_manipulator_msgs.srv import SetJointPosition
import time

class InverseKinematicsService(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_service')
        # self.srv = self.create_service(InvKin,'calc_inv_kin',self.calc_inv_kn)
        self.srv = self.create_service(InvKin,'pick_object',self.pick_object)
        
        self.l0 = 36.076
        self.l1 = 60.25
        self.l2 = 128.0
        self.l3 = 24.0
        self.l4 = 124.0
        self.l5 = 133.4

    def calc_inv_kn(self,request,response, path_time=0.1):
        x = request.pose.position.x
        y = request.pose.position.y
        z = request.pose.position.z

        self.get_logger().info(f"Received pose: {request.pose.position.x}, {request.pose.position.y}, {request.pose.position.z}, "
                               f"{request.pose.orientation.x}, {request.pose.orientation.y}, {request.pose.orientation.z}, {request.pose.orientation.w}")
        self.get_logger().info('Incoming request\nx:%d y:%d z:%d'%(x,y,z))

        theta_1 = numpy.arctan2(y,x)
        
        r  = numpy.sqrt(numpy.square(x) + numpy.square(y))
        r2 = self.l0 + self.l1 - z - self.l5
        D  = (numpy.square(self.l2) + numpy.square(self.l3) + numpy.square(self.l4) - numpy.square(r) - numpy.square(r2))/(2*self.l4*numpy.sqrt(numpy.square(self.l2) + numpy.square(self.l3)))

        theta_y = numpy.arctan2(numpy.sqrt(1 - numpy.square(D)),D)
        theta_v = numpy.arctan2(self.l3,self.l2)
        
        theta_3 = numpy.pi/2 + theta_v - theta_y

        D2 = (numpy.square(r) + numpy.square(r2) + numpy.square(self.l2) + numpy.square(self.l3) - numpy.square(self.l4))/(2*numpy.sqrt(numpy.square(r) + numpy.square(r2))*numpy.sqrt(numpy.square(self.l2) + numpy.square(self.l3)))

        theta_z = numpy.arctan2(numpy.sqrt(1 - numpy.square(D2)),D2)
        theta_w = numpy.arctan2(r,r2)

        theta_2 = numpy.pi - theta_v - theta_z - theta_w

        theta_4 = numpy.pi/2 - theta_2 - theta_3

        response.joint_vals.data = [theta_1, theta_2, theta_3, theta_4]

        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [theta_1, theta_2, theta_3, theta_4, 0.0]
        self.goal_joint_space_req.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

        

        return response
    
    def calc_theta(self,x,y,z):
        r  = numpy.sqrt(numpy.square(x) + numpy.square(y))
        r2 = self.l0 + self.l1 - z - self.l5
        D  = (numpy.square(self.l2) + numpy.square(self.l3) + numpy.square(self.l4) - numpy.square(r) - numpy.square(r2))/(2*self.l4*numpy.sqrt(numpy.square(self.l2) + numpy.square(self.l3)))

        theta_y = numpy.arctan2(numpy.sqrt(1 - numpy.square(D)),D)
        theta_v = numpy.arctan2(self.l3,self.l2)
        
        theta_3 = numpy.pi/2 + theta_v - theta_y

        D2 = (numpy.square(r) + numpy.square(r2) + numpy.square(self.l2) + numpy.square(self.l3) - numpy.square(self.l4))/(2*numpy.sqrt(numpy.square(r) + numpy.square(r2))*numpy.sqrt(numpy.square(self.l2) + numpy.square(self.l3)))

        theta_z = numpy.arctan2(numpy.sqrt(1 - numpy.square(D2)),D2)
        theta_w = numpy.arctan2(r,r2)

        theta_2 = numpy.pi - theta_v - theta_z - theta_w

        theta_4 = numpy.pi/2 - theta_2 - theta_3

        return theta_2, theta_3, theta_4

        
    
    def pick_object(self,request,response, path_time=10.0):
        x = request.pose.position.x
        y = request.pose.position.y
        z = request.pose.position.z

        self.get_logger().info(f"Received pose: {request.pose.position.x}, {request.pose.position.y}, {request.pose.position.z}, "
                               f"{request.pose.orientation.x}, {request.pose.orientation.y}, {request.pose.orientation.z}, {request.pose.orientation.w}")
        self.get_logger().info('Incoming request\nx:%d y:%d z:%d'%(x,y,z))

        theta_1 = numpy.arctan2(y,x)
        #######################################
        ##Go to the intermediate position
        # z_mid = 80

        theta_2, theta_3, theta_4 = self.calc_theta(x,y,z)

        # self.send_goal_joint_space(theta_1, theta_2, theta_3, theta_4, path_time)
        # self.get_logger().info('I am at z mid')
                
        # time.sleep(1)
        ##Open gripper
        # self.send_tool_control_request(theta_1, theta_2, theta_3, theta_4, 0.01, path_time)
        # time.sleep(1)
        # self.get_logger().info('Opened gripper at z mid')

        # ########################################
        # ##Go to the object

        # theta_2, theta_3, theta_4 = self.calc_theta(x,y,z)

        # response.joint_vals.data = [theta_1, theta_2, theta_3, theta_4]

        # self.send_goal_joint_space(theta_1, theta_2, theta_3, theta_4, path_time)
        # self.get_logger().info('I am at object')

        
        # time.sleep(1)
        # ##Close gripper
        # self.send_tool_control_request(theta_1, theta_2, theta_3, theta_4, -0.01, path_time)
        # time.sleep(1)
        # self.get_logger().info('Closed tool at object')

        # ########################################
        # ##Go to the intermediate position

        # theta_2, theta_3, theta_4 = self.calc_theta(x,y,z_mid)

        # self.send_goal_joint_space(theta_1, theta_2, theta_3, theta_4, path_time)
        # self.get_logger().info('Picked up object')

        
        ######################################
        response.joint_vals.data = [theta_1, theta_2, theta_3, theta_4]
        return response




def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_service = InverseKinematicsService()

    rclpy.spin(inverse_kinematics_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
