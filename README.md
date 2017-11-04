# BlueRov-ROS-playground
Scripts to help BlueRov integration with ROS

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
- Fake video stream
    - `$ ffmpeg -i /dev/video0 -b 50k -r 20 -s 1080x720 -f mpegts udp://@127.0.0.1:5600`
        -  If necessary, change video source and resolution
- Launch example
    - `roslaunch bluerov_ros_playground user_mav.launch`


### Topics ###
If you need more information about the topics and what you can access, take a look [here](doc/topics_and_data.md).

### Test ###
- View 3D model:
    - RVIZ: `roslaunch bluerov_ros_playground rviz.launch`
    - Gazebo: `roslaunch bluerov_ros_playground gazebo.launch`
