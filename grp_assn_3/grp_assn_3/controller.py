#!/usr/bin/env python3

# ros2 service call /set_position custom_interfaces/srv/PosRef "{reference: 2094.0}"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from custom_interfaces.srv import PosRef
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetCurrent
from dynamixel_sdk_custom_interfaces.srv import GetPosition
import time
import numpy as np

class PDController(Node):   
    def __init__(self):
        super().__init__('controller')
        
        # Controller parameters
        self.Kp = 0.3  # Start with small gains
        self.Kd = 0.01
        self.desired_position = 1900.0
        self.current_positon = 0.0
        self.previous_position = 0.0
        self.sampling_time = 0.01  # 100Hz control loop
        
        # Publishers and subscribers
        # self.effort_publisher = self.create_publisher(
        #     Float64, 
        #     '/actuator4/effort_command', 
        #     10)
        
        self.current_publisher = self.create_publisher(
            SetCurrent, 
            'set_current', 
            10)
            
        # Client to get position
        self.position_client = self.create_client(
            GetPosition,
            'get_position')
        if not self.position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Get position service not available')
        
        self.get_position_req = GetPosition.Request()
        self.get_position_req.id = 14  # Actuator 4 ID        

            
        # Service for setting reference position
        self.set_position_service = self.create_service(
            PosRef,
            'set_position',
            self.set_position_callback)
        
        # Data logging
        self.start_time = self.get_clock().now().nanoseconds
        self.log_file = open("joint_positions.txt", "w")


        self.get_logger().info('PD Controller node started')
        self.control_loop()

    def get_current_position(self):
        future = self.position_client.call_async(self.get_position_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        while future.result() is None:
            print("none")
            time.sleep(0.05)
        return future.result().position
    
    def set_current(self, current):
        msg = SetCurrent()
        msg.id = 14  # Actuator 4 ID
        msg.current = current
        print(current)
        self.current_publisher.publish(msg)
    
    
    def set_position_callback(self, request, response):
        self.desired_position = request.reference
        self.get_logger().info(f'New reference position: {self.desired_position}')
        
        response.error = float(self.desired_position - self.current_position)
        return response
    
    def control_loop(self):
        self.start_time = self.get_clock().now().nanoseconds / 1_000_000
        target_duration = 10 * 1000 
        while True:
        # Get current position
            self.current_position = self.get_current_position()
            
            # Calculate position error
            position_error = self.desired_position - self.current_position
            
            # Calculate velocity (derivative of position)
            velocity = (self.current_position - self.previous_position) / self.sampling_time
            
            # PD control law
            control_output = self.Kp * position_error - self.Kd * velocity
            # control_output = max(-50, min(50, control_output))
            
            # Apply control effort
            self.set_current(int(np.ceil(control_output)))
            
            # Log data
            current_time = self.get_clock().now().nanoseconds / 1_000_000 - self.start_time
            self.log_file.write(f"{current_time}, {self.desired_position}, {self.current_position}\n")
            if current_time >= target_duration:
                self.get_logger().info('10 seconds elapsed, exiting control loop.')
                break
            
            # Store current position for next iteration
            self.previous_position = self.current_position
            
            # Debug info
            self.get_logger().info(f'Position Error: {position_error}')
            self.get_logger().info(f'Control Output: {control_output}')

            time.sleep(0.01)

    def __del__(self):
        self.log_file.close()


def main():

    rclpy.init()
    controller = PDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()