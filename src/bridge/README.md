# bluerov_node: A ROS package to control BlueRov2

**_bluerov_node_** is a [ROS](http://ros.org/ "Robot Operating System") package which uses [pyros](https://github.com/asmodehn/pyros) to communicate with ROS and [pymavlink](https://github.com/ArduPilot/pymavlink) to interact with the ROV.

## Usage

Once the BlueRov is connected to your computer, run:

```bash
$ roslaunch bluerov_ros_playground bluerov_node.launch
```
You can select the input device with: `bluerov_node_device:=udp:localhost:14550` or `bluerov_node_device:=/dev/ttyACM0` argument.
The default value is `udp:192.168.2.1:14550`.

## Topics
The folowing topics are available:
`$ rostopic list`

 - **/BlueRov2/arm** - `std_msgs/Bool`
 - **/BlueRov2/battery** - `sensor_msgs/BatteryState`
 - **/BlueRov2/camera/image_raw** - `sensor_msgs/Image`
 - **/BlueRov2/imu/data** - `sensor_msgs/Imu`
 - **/BlueRov2/mode/set** - `std_msgs/String`
 - **/BlueRov2/odometry** - `nav_msgs/Odometry`
 - **/BlueRov2/rc_channel1/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/rc_channel2/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/rc_channel3/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/rc_channel4/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/rc_channel5/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/rc_channel6/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/rc_channel7/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/rc_channel8/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo1/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo2/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo3/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo4/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo5/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo6/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo7/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/servo8/set_pwm** - `std_msgs/UInt16`
 - **/BlueRov2/setpoint_velocity/cmd_vel** - `geometry_msgs/TwistStamped`
 - **/BlueRov2/state** - `std_msgs/String`
 - **/rosout** - `rosgraph_msgs/Log`
 - **/rosout_agg** - `rosgraph_msgs/Log`

# Some commands

```bash
# Set manual mode
$ rostopic pub -1 /BlueRov2/mode/set std_msgs/String "manual"
# Arm the vehicle
$ rostopic pub -1 /BlueRov2/arm std_msgs/Bool 1
# Set angular and linear speed
$ rostopic pub -r 4 /BlueRov2/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "{header: auto, twist: {linear: {x: 10.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
# Set MAIN OUT pwm value
$ rostopic pub -r 4 /BlueRov2/servo1/set_pwm std_msgs/UInt16  1500
# Visualize camera image
$ rosrun image_view image_view image:=/BlueRov2/camera/image_raw
# See ROV state
$ rostopic echo /BlueRov2/state
# Watch battery information
$ rostopic echo /BlueRov2/battery
# IMU information
$ rostopic echo /BlueRov2/imu/data
```