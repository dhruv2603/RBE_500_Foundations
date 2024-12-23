# Foundations of Robotics Hands-on Project
### *RBE500: Foundations of Robotics - [Worcester Polytechnic Institute](https://www.wpi.edu/), Fall 2024*
# OpenManipulatorX_ROS2
ROS2 Humble packages to control Open Manipulator X

## Build
Copy the contents of the entire repository into the src/ directory of a new ROS2 workspace.
```
mkdir -p rbe500_ws/src
cd rbe500_ws
git clone https://github.com/hylander2126/OpenManipulatorX_ROS2.git ./src
```

Run the following to build the packages in your new workspace:
```
colcon build --symlink-install
```
You may get some warning mesages, ignore them for now as long as everything successfully builds.


Don't forget to source your workspace:
```
source install/setup.bash
```

After connecting to the robot over USB, run the following to begin position control:
```
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

Now run the example code to move the robot:
```
ros2 run rbe500-example basic_robot_control
```
Or for python:
```
ros2 run rbe500_example_py basic_robot_control
```
---
RBE 500 - Foundations of Robotics 2024 taught by Professor Berk Calli at Worcester Polytechnic Institute Robotics Engineering Department


# Group Assignments

## Group Assignment 1

### Overview
1. Implement a forward kinematics node that:
    * Subscribes to the joint values topic and reads them from the robot's joint states topic.
    * Calculates the end effector pose
    * Publishes the pose as a ROS topic
2. Implement an inverse kinematics node that has a service client implementation that takes a desired pose of the end effector from the user and returns joint positions as response.
3. Place an object at a particular location on the chess board. Calculate the inverse kinematics for that gripper pose to find the point angles. Move the robot to that position, pick the object and lift it.

### Running the assignment
Start by running the Launch file of Open Manipulator using the command
```bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```
Run the forward kinematics node using the command
```bash
ros2 run grp_assn_1 fwd_kin_exec
```
To view the Pose run the command below
```bash
ros2 topic echo /end_pose
```
Run the inverse kinematics service node using the command
```bash
ros2 run grp_assn_1 inv_kn_server
```
For calculating the inverse kinematics call the service into action, for eg:
```bash
ros2 service call /calc_inv_kin custom_interfaces/srv/InvKin "{ pose: { position: { x: 138.42, y: -75.91, z: 62.9}, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
For picking up the object call the other service:
ros2 service call /pick_object custom_interfaces/srv/InvKin "{ pose: { position: { x: 201.98, y: 110.36, z: 41.89}, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

## Group Assignment 2

### Overview
1. Implement a node wih two services:
    * One takes joint velocities and converts them to end effector velocities.
    * The other takes end effector velocities and converts them to joint velocities.
2. Create a node that provides incremental position references to the robot joints i.e. q_ref = q_ref_old + delta_q * sampling_time. The node would then send the q_ref to the joint position controllers of the robot as joint goals.
3. Give a constant velocity reference in the '+y' direction. Convert this velocity to the joint space velocities using the jacobian and feed it as a reference (detla_q) to the incremental position reference node.

### Running the assignment
Start by running the Launch file of Open Manipulator using the command
```bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```
Run the Joint Velocities to Twist service
```bash
ros2 run grp_assn_2 cal_twist
```
Call the service to calculate the end effector Twist
```bash
ros2 service call /cal_end_eff_twist custom_interfaces/srv/JointVel "{ joint_vel: { data: [0.1, 0.1, 0.1, 0.1]}}"
```
Run the Twist to Joint Velocities service
```bash
ros2 run grp_assn_2 cal_joint_vel
```
Call the service to calculate the joint velocities
```bash
ros2 service call /cal_joint_vel custom_interfaces/srv/Twist "{end_eff_twist: {linear: {x: 361.55, y: 193.5, z: 18.65}, angular: {x: -1.738, y: 2.444, z: 1.0}}}"
```
Run the incremental position client
```bash
ros2 run grp_assn_2 incremental_pos
```

## Group assignment 3

### Overview
1. Implement a PD position controller for the robot actuator 4 using current (effort) control mode.
2. Read the joint position values for actuator 4, receive a position reference value through a service and publish the joint current(effort) with high sampling rate to the actuator.

### Running the assignment
Start by running the below command for current control mode:
```bash
ros2 run dynamixel_sdk_examples combined_read_write
```
Run the PD control node using the command:
```bash
ros2 run grp_assn_3 pd_control
```
Set the desired position of the actuator by calling the service:
```bash
ros2 service call /set_position custom_interfaces/srv/PosRef "{reference: 2094.0}"
```