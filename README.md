# BlueRov-ROS-playground
Scripts to help BlueRov integration with ROS

<pre>
                      +-----------------------+         +------------------------+
                      |     <b>Raspberry Pi</b>      |         |    <b>Topside Commputer</b>   |
                      |    <b>ip 192.168.2.2</b>     |         |     <b>ip 192.168.2.1</b>     |
                      |                       |         |                        |
+-------+  Telemetry  | +-------------------+ |         |                        |
|Pixhawk<-------------->USB         <b>MAVProxy</b>| |         |                        |
+-------+    Pilot    | +                   + |         | +--------------------+ |
            Control   | |            udpbcast<----------->:14550         <b>MAVROS</b>| |
                      | +-------------------+ |  Pilot  | |(UDP)               | |
                      |                       | Control | |                    | |
                      | +-------------------+ |         | |       (ROS)        | |
+---------+           | CSI+2       <b>raspivid</b>| |         | +------+/mavros+-----+ |
|Raspberry+------------>camera              | |         |           +            |
| Camera  |           | port                | |         |           |            |
+---------+           | +                   | |         | +---------v----------+ |
                      | |                   | |         | |subs.py      pubs.py| |
                      | +------------+stdout+ |         | |                    | |
                      |                  +    |         | |                    | |
                      |             Raw  |    |         | |                    | |
                      |             H264 |    |         | |                    | |
                      |                  v    |         | |      <b>user.py</b>       | |
                      | +------------+ fdsrc+ |         | |                    | |
                      | |<b>gstreamer</b>          | |         | |                    | |
                      | |                   + |         | :5600 video.py       | |
                      | |             udpsink+----------->(UDP)                | |
                      | +-------------------+ |  Video  | +---------^----------+ |
                      |                       | Stream  |           |            |
                      +-----------------------+         |           +            |
                                                        | +--------/joy--------+ |
                                                        | |<b>joy</b>     (ROS)       | |         +--------+
                                                        | |                  USB<----------+Joystick|
                                                        | +--------------------+ |  Pilot  +--------+
                                                        |                        | Control
                                                        +------------------------+
</pre>
### Requirements ###
- ros-desktop-full
  - kinetic or newer
- [mavros](http://wiki.ros.org/mavros)

### Installation ###
 1. `$ cd ros_workspace_path/src`
 2. `$ git clone https://github.com/patrickelectric/bluerov_ros_playground`
 3. `$ cd ../`
 4. `$ catkin_make --pkg bluerov_ros_playground`
    - if using ROS source:
        - ` $./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --pkg bluerov_ros_playground`
 5. Reload your ROS env.
    - bash: `$ source devel/setup.sh`
    - zsh: `$ source devel/setup.sh`

### Running with SITL ###
- Run ArduPilot SITL

    1. [Download ArduPilot and configure SITL](http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
    2. `$ cd ardupilot/ArsuSub`
    3. `$ sim_vehicle.py`

    - To test mavros communication with SITL:
        - `$ roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@`

- Fake video stream

    - `$ gst-launch-1.0 videotestsrc ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc ! rtph264pay ! udpsink host=127.0.0.1 port=5600`
        - If necessary, change video source and resolution.
        - To test the udp reception: `gst-launch-1.0 -v udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! fpsdisplaysink sync=false text-overlay=false`

- Launch user example

    - `roslaunch bluerov_ros_playground user_mav.launch`

- Gazebo

    This example allow SITL communication with Gazebo, right now the only interaction that happen is the thruster control using [thruster pwm fitting](https://colab.research.google.com/notebook#fileId=1CEDW9ONTJ8Aik-HVsqck8Y_EcHYLg0zK).
    - Run SITL and start gazebo.launch
    - `roslaunch bluerov_ros_playground gazebo.launch`

- RVIZ
    - `roslaunch bluerov_ros_playground rviz.launch`



### Topics ###
If you need more information about the topics and what you can access, take a look [here](doc/topics_and_data.md).

### Test ###
- View 3D model:
    - RVIZ: `roslaunch bluerov_ros_playground rviz.launch`
    - Gazebo: `roslaunch bluerov_ros_playground gazebo.launch`
