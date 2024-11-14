import sys
import rclpy
from rclpy.node import Node
# import math
# import numpy
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Quaternion
# from scipy.spatial.transform import Rotation as R
from custom_interfaces.srv import InvKin

class InverseKinematicsClient(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_client')
        self.cli = self.create_client(InvKin, 'calc_inv_kin')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = InvKin.Request()

    def send_request(self):
        self.req.pose.position.x = float(sys.argv[1])
        self.req.pose.position.y = float(sys.argv[2])
        self.req.pose.position.z = float(sys.argv[3])
        self.req.pose.orientation.x = float(sys.argv[4])
        self.req.pose.orientation.y = float(sys.argv[5])
        self.req.pose.orientation.z = float(sys.argv[6])
        self.req.pose.orientation.w = float(sys.argv[7])

        self.get_logger().info(f'Sending request: {self.req.pose}')

        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_client = InverseKinematicsClient()
    inverse_kinematics_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(inverse_kinematics_client)
        if inverse_kinematics_client.future.done():
            try:
                response = inverse_kinematics_client.future.result()
            except Exception as e:
                inverse_kinematics_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                inverse_kinematics_client.get_logger().info(
                    'Result of calc_inv_kin:\n%s' %
                    (response.joint_vals.data,))
            break

    inverse_kinematics_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()