# ROS2-DynamixelMotorControl
ROS2 package for Dynamixel Motor Control

Uses Dynamixel motor control Python scripts integrated with ROS2 to run protocol_0 motors.
Call "rosrun run_motors control_motor [motor_id]" to set up three subscribers (set_position, set_speed, set_torque). Can be used for multiple motors.
Publish to these topics in order to alter the position, speed, or torque (with arguments in Int32 std_msgs format).
