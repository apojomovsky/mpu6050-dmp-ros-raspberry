# MPU6050 DMP ROS

This project leverages the Raspberry Pi's capability to interface with an MPU6050 sensor via I2C/smbus. It utilizes the sensor's Digital Motion Processor (DMP) to deliver fully filtered readings of orientation, linear acceleration, and angular velocity.

The DMP employs a suite of proprietary MotionFusion algorithms as an effective alternative to the algorithms found in the [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools) package. The processed values are published as standard `sensor_msgs/Imu` messages, ready for use in ROS applications.

This implementation builds upon the foundational work from [thisisG's MPU6050-I2C-Python-Class](https://github.com/thisisG/MPU6050-I2C-Python-Class), with added support for ROS Noetic and compatibility with the Raspberry Pi platform.
