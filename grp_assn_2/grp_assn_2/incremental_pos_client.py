import sys
import rclpy
from rclpy.node import Node
import time
# import math
import numpy
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Quaternion
# from scipy.spatial.transform import Rotation as R
from custom_interfaces.srv import InvKin
from open_manipulator_msgs.srv import SetJointPosition
from custom_interfaces.srv import Twist

class IncrementalPositionClient(Node):
    def __init__(self):
        super().__init__('incremental_position_client')
        
        self.move_joints_client = self.create_client(SetJointPosition,'goal_joint_space_path')
        self.move_joints_client_req = SetJointPosition.Request()

        self.twist_to_joint__vel_client = self.create_client(
            Twist,
            'cal_joint_vel',
        ) # Create client for end effector to joint velocity kinematics service
        self.twist_to_joint_vel_client_req = Twist.Request()

        self.l0 = 36.076
        self.l1 = 60.25
        self.l2 = 128.0
        self.l3 = 24.0
        self.l4 = 124.0
        self.l5 = 133.4

    def send_move_joint_request(self, theta_1, theta_2, theta_3, theta_4, path_time):
        self.move_joints_client_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.move_joints_client_req.joint_position.position = [theta_1, theta_2, theta_3, theta_4, 0.0]
        self.move_joints_client_req.path_time = path_time

        try:
            self.future = self.move_joints_client.call_async(self.move_joints_client_req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result

        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))
            return None  

    def send_twist_to_joint_vel_request(self,vx, vy, vz, wx, wy, wz):
        self.twist_to_joint_vel_client_req.end_eff_twist.linear.x = vx
        self.twist_to_joint_vel_client_req.end_eff_twist.linear.y = vy
        self.twist_to_joint_vel_client_req.end_eff_twist.linear.z = vz
        self.twist_to_joint_vel_client_req.end_eff_twist.angular.x = wx
        self.twist_to_joint_vel_client_req.end_eff_twist.angular.y = wy
        self.twist_to_joint_vel_client_req.end_eff_twist.angular.z = wz

        # Check if the service is availble
        if not self.twist_to_joint__vel_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("Twist Service not available")
            return None

        self.future = self.twist_to_joint__vel_client.call_async(self.twist_to_joint_vel_client_req)
        try:
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=10.0)
            if self.future.done():
                response = self.future.result()
                return response
            else:
                self.get_logger().info("EndEffectorJoint Service call timedout")
                return None
        except Exception as e:
            self.get_logger().info('Sending End effector Twist Failed %r' % (e,))
            return None

    def transformation_matrix(self,a,theta,d,alpha):
        A = numpy.array([
            [numpy.cos(theta), -numpy.sin(theta)*numpy.cos(alpha), numpy.sin(theta)*numpy.sin(alpha), a*numpy.cos(theta)],
            [numpy.sin(theta), numpy.cos(theta)*numpy.cos(alpha), -numpy.cos(theta)*numpy.sin(alpha), a*numpy.sin(theta)],
            [0,                numpy.sin(alpha),                   numpy.cos(alpha),                  d],
            [0, 0, 0, 1],
        ])
        return A  
    
    def calc_forward_kinematics(self, q1, q2, q3, q4):
        """ Calculate forward kinematics """
        A0 = numpy.array([[ 1, 0, 0, 0],
                          [ 0, 1, 0, 0], 
                          [ 0, 0, 1, self.l0],
                          [ 0,0,0,1]] , dtype= object)
        
        
        "For Link 1"
        a1 = 0
        theta1 = q1
        d1 = self.l1
        alpha1 = -numpy.pi/2
        A1 = self.transformation_matrix(self,a1,theta1,d1,alpha1)
        H1 = numpy.matmul(A1,A0)

        "For Link 2"
        a2 = numpy.sqrt(numpy.square(self.l2) + numpy.square(self.l3))
        theta2 = -numpy.arctan2(self.l2,self.l3) + q2
        d2 = 0 
        alpha2 = 0
        A2 = self.transformation_matrix(self,a2,theta2,d2,alpha2)
        H2 = numpy.matmul(H1,A2)

        "For Link 3"
        a3 = self.l4
        theta3 = numpy.arctan2(self.l2,self.l3) + q3
        d3 = 0
        alpha3 = 0
        A3 = self.transformation_matrix(self,a3,theta3,d3,alpha3)
        H3 = numpy.matmul(H2,H3)

        "For Link 4"
        a4 = self.l5
        theta4 = q4
        d4 = 0
        alpha4 = 0
        A4 = self.transformation_matrix(self,a4,theta4,d4,alpha4)
        H4 = numpy.matmul(H3,A4)

        # Extract position
        position = H4[:3, 3]
        x = position[0]
        y = position[1]
        z = position[2]

        return x, y, z
    
def main():
    rclpy.init()
    vx = float(0.0)
    vy = float(2.0)
    vz = float(0.0)
    wx = float(0.0)
    wy = float(0.0)
    wz = float(0.0)

    # Create a client node
    incremental_position_client = IncrementalPositionClient()

    """ Step 1: Send end effector velocities to end effector to joint velocity kinematics service """
    total_time = 50
    samples = 50
    sampling_time = total_time/samples

    joint1 = 0.0
    joint2 = 0.0
    joint3 = 0.0
    joint4 = 0.0

    joint_traj = []

    for i in range(samples):
        joint_velocities = incremental_position_client.send_twist_to_joint_vel_request(vx, vy, vz, wx, wy, wz)
        joint1 += joint_velocities.joint_vel.data[0]*sampling_time
        joint2 += joint_velocities.joint_vel.data[1]*sampling_time
        joint3 += joint_velocities.joint_vel.data[2]*sampling_time
        joint4 += joint_velocities.joint_vel.data[3]*sampling_time

        joint_traj.append([joint1, joint2, joint3, joint4, 0.0])
    
    incremental_position_client.get_logger().info("Twist Service: True")

    """ Step 2: Send joint positions to goal joint space planning service """
    step_count = 0
    path = []
    for joints in joint_traj:
        coords = []
        x,y,z = incremental_position_client.calc_forward_kinematics(joints[0], joints[1], joints[2], joints[3])
        coords.append(round(x))
        coords.append(round(y))
        coords.append(round(z))
        path.append(coords)
        # Send request to goal joint space path planning service
        response = incremental_position_client.send_move_joint_request(joints[0], joints[1], joints[2], joints[3], sampling_time)
        incremental_position_client.get_logger().info('Joint Space Path Planning: %s' % (response.is_planned))
        step_count += 1
        time.sleep(0.5)

if __name__ == '__main__':
    main()

