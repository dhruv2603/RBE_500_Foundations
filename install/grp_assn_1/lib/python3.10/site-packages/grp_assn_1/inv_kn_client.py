import sys
import rclpy
from rclpy.node import Node
import time
# import math
# import numpy
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Quaternion
# from scipy.spatial.transform import Rotation as R
from custom_interfaces.srv import InvKin
from open_manipulator_msgs.srv import SetJointPosition

class InverseKinematicsClient(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_client')
        # self.cli = self.create_client(InvKin, 'calc_inv_kin')
        self.new_cli = self.create_client(InvKin,'pick_object')
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_joint_space_req = SetJointPosition.Request()

        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.tool_control_req = SetJointPosition.Request()

        while not self.new_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = InvKin.Request()
    
    def send_goal_joint_space(self, theta_1, theta_2, theta_3, theta_4, path_time):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [theta_1, theta_2, theta_3, theta_4, 0.0]
        self.goal_joint_space_req.path_time = path_time

        try:
            future = self.goal_joint_space.call_async(self.goal_joint_space_req)
            rclpy.spin_until_future_complete(self, future)
            # future_1 = self.goal_joint_space.call_async(self.goal_joint_space_req)
            # rclpy.spin_until_future_complete(self, future_1)

        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))
    
    def send_tool_control_request(self, theta_1, theta_2, theta_3, theta_4, theta_5, path_time):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [theta_1, theta_2, theta_3, theta_4, theta_5]
        self.tool_control_req.path_time = path_time

        try:
            time.sleep(1)
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)
            time.sleep(2)
            # future_2 = self.tool_control_result = self.tool_control.call_async(self.tool_control_req)
            # rclpy.spin_until_future_complete(self, future_2)
        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

    def send_request(self, x, y, z, ox, oy, oz, ow):
        self.req.pose.position.x = x
        self.req.pose.position.y = y
        self.req.pose.position.z = z
        self.req.pose.orientation.x = ox
        self.req.pose.orientation.y = oy
        self.req.pose.orientation.z = oz
        self.req.pose.orientation.w = ow

        self.get_logger().info(f'Sending request: {self.req.pose}')

        # self.future = self.cli.call_async(self.req)
        # self.new_future = self.new_cli.call_async(self.req)

        future = self.new_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
    
        if future.done():
            try:
                response = future.result()
                return response.joint_vals.data  # Return joint values from IK solution
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return None

def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_client = InverseKinematicsClient()
    x = float(sys.argv[1])
    y = float(sys.argv[2]) 
    z = float(sys.argv[3])
    ox = float(sys.argv[4])
    oy = float(sys.argv[5])
    oz = float(sys.argv[6])
    ow = float(sys.argv[7])
    response = inverse_kinematics_client.send_request(x, y, 70.0, ox, oy, oz, ow)

    while rclpy.ok():
        rclpy.spin_once(inverse_kinematics_client)
        # if inverse_kinematics_client.new_future.done():
        try:
            # response = inverse_kinematics_client.new_future.result()
            theta = response  # Get joint angles from IK 
            inverse_kinematics_client.send_goal_joint_space(theta[0], theta[1], theta[2], theta[3], 10.0)
            time.sleep(2)
            inverse_kinematics_client.send_tool_control_request(theta[0], theta[1], theta[2], theta[3], 0.01, 2.0)
            time.sleep(2)

            theta = inverse_kinematics_client.send_request(x, y, z, ox, oy, oz, ow)
            inverse_kinematics_client.send_goal_joint_space(theta[0], theta[1], theta[2], theta[3], 2.0)
            time.sleep(2)
            inverse_kinematics_client.send_tool_control_request(theta[0], theta[1], theta[2], theta[3], -0.01, 2.0)
            time.sleep(2)

            theta = inverse_kinematics_client.send_request(x, y, 70.0, ox, oy, oz, ow)
            inverse_kinematics_client.send_goal_joint_space(theta[0], theta[1]-0.2, theta[2], theta[3], 2.0)
            time.sleep(2)
            inverse_kinematics_client.send_goal_joint_space(-theta[0], theta[1]-0.2, theta[2], theta[3], 2.0)
            time.sleep(2)

            theta = inverse_kinematics_client.send_request(x, y, z, ox, oy, oz, ow)
            inverse_kinematics_client.send_goal_joint_space(-theta[0], theta[1], theta[2], theta[3], 2.0)
            time.sleep(2)
            inverse_kinematics_client.send_tool_control_request(-theta[0], theta[1], theta[2], theta[3], 0.01, 2.0)
            time.sleep(2)
            inverse_kinematics_client.send_goal_joint_space(-theta[0], theta[1]-0.5, theta[2], theta[3], 2.0)
            time.sleep(2)










        except Exception as e:
            inverse_kinematics_client.get_logger().error(f'Service call failed: {e}')
            
        break
        
        # theta = response.joint_vals.data
        # theta_1 = theta[0]
        # theta_2 = theta[1]
        # theta_3 = theta[2]
        # theta_4 = theta[3]
        # inverse_kinematics_client.send_goal_joint_space(theta_1,theta_2,theta_3,theta_4,2.0)
        # rclpy.spin_once(inverse_kinematics_client)

    
        # time.sleep(2)
        # inverse_kinematics_client.send_tool_control_request(theta_1,theta_2,theta_3,theta_4,0.01,2.0)
        # rclpy.spin_once(inverse_kinematics_client)
        # time.sleep(2)

        ########################################################
        # inverse_kinematics_client.send_request(x, y, z, ox, oy, oz, ow)
        # rclpy.spin_once(inverse_kinematics_client)
        # if inverse_kinematics_client.new_future.done():
        #     try:
        #         response = inverse_kinematics_client.new_future.result()
        #         new_response = inverse_kinematics_client.new_future.result()
        #     except Exception as e:
        #         inverse_kinematics_client.get_logger().info(
        #             'Service call failed %r' % (e,))
        #     else:
        #         # inverse_kinematics_client.get_logger().info(
        #         #     'Result of calc_inv_kin:\n%s' %
        #         #     (response.joint_vals.data,))
        #         inverse_kinematics_client.get_logger().info(
        #             'Result of pick_object:\n%s' %
        #             (new_response.joint_vals.data,))
        #     break
    
        # theta = response.joint_vals.data
        # theta_1 = theta[0]
        # theta_2 = theta[1]
        # theta_3 = theta[2]
        # theta_4 = theta[3]
        # inverse_kinematics_client.send_goal_joint_space(theta_1,theta_2,theta_3,theta_4,2.0)
        # rclpy.spin_once(inverse_kinematics_client)
    
        # time.sleep(2)
        # inverse_kinematics_client.send_tool_control_request(theta_1,theta_2,theta_3,theta_4,-0.01,2.0)
        # rclpy.spin_once(inverse_kinematics_client)
        # time.sleep(2)



    inverse_kinematics_client.destroy_node()


    rclpy.shutdown()


if __name__ == '__main__':
    main()