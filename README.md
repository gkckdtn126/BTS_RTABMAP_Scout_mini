# **BTS project**
---
## Implementation of Autonomous Driving System with AgileX SCOUT-mini gazebo simulator

### **0. Install the Gazebo software**

Gazebo is a robot simulator. Gazebo simulates multiple robots in a 3D environments, with extensive dynamic interaction between objects.

**Gazebo Download Link** : [http://gazebosim.org](http://gazebosim.org/)

Download and install gazebo. You can go to the website : http://gazebosim.org/install

### **1. Installation**

1. **development Environment ubuntu 20.04 + [ROS Noetic desktop full](http://wiki.ros.org/noetic/Installation/Ubuntu)**

2. **Build ROS packages for Scout simulator**
        
    * Create worksapce, download ROS packages
    ```
    mkdir -p ~/scout_ws/src
    cd ~/scout_ws/src
    git clone --recurse-submodules https://github.com/hjinnkim/BTS_scout_mini.git
    ```

3.  **Install required ROS packages**
    
    a. Noetic
    ```(dependencies) and build from source
    cd BTS_scout_mini
    sh install_dependencies_noetic.sh
    ```

    b. Melodic
    ```
    sudo apt-get install libasio-dev ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-joint-state-publisher-gui ros-melodic-teleop-twist-keyboard ros-melodic-navigation ros-melodic-gmapping
    ```

4. **Install dependencies and build**
    ```
    cd ~/scout_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make

### **2. Usage**
1. **SCOUT-mini description**
    ```
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_description display_scout_mini.launch 
    ```

2. **Launch gazebo simulator and teleop control**
    
    a. Launch gazebo simulator
    ```
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_gazebo_sim scout_mini_playpen.launch
    ```

    b. Run teleop controller (w, a, s, d)
        
    ```
    //Open another terminal

    cd ~/scout_ws
    roslaunch scout_teleop scout_teleop_key.launch 
    ```

### **3. 2D Navigation**
1. **SLAM mapping**
    
    a. Gmapping
    ```
    // Run gmapping slam

    roslaunch scout_slam scout_slam.launch
    ```
    ```
    // Save map (Another terminal)

    roslaunch scout_slam gmapping_save.launch
    ```

    b. Cartographer
    
    To be added

2. **Navigation**
    ```
    // Run navigation

    roslaunch scout_navigation scout_navigation.launch
    ```

### ** 4. 3D navigation**
1. **RTAB MAP**


    a. Start roscore in master computer
    ```
    roscore
    ```
    b. Start Can connection and bring up scout_mini 
    ```
    rosrun scout_bringup bringup_can2usb.bash
    ```
    ```
    roslaunch scout_bringup scout_mini_robot_base.launch
    ```
    c. Start realsense bringup and imu filter 
    ```
    roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation" enable_gyro:=true enable_accel:=true
    ```
    ```
    rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
    ```
    d. RTAB MAP launch
    ```
    roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3 --Grid/Sensor 1 --Vis/CorGuessWinSize 60 --Odom/FilteringStrategy 0 --" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu 
    ```
    d.1 smooth(parameter tuning)
    ```
    roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3 --Grid/Sensor 1 --Grid/CellSize 0.07 --Grid/3D false --Vis/CorGuessWinSize 60 Odom/Strategy 0 --Odom/FilteringStrategy 0 --OdomF2M/MaxSize 5000 --OdomF2M/ScanMaxSize 5000 --Odom/ResetCountdown 1--" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
    
    roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3 --Grid/Sensor 1 --Grid/CellSize 0.07 --Grid/3D false --Vis/CorGuessWinSize 60 Odom/Strategy 0 --Odom/FilteringStrategy 0 --OdomF2M/MaxSize 5000 --OdomF2M/ScanMaxSize 5000 --Mem/IncrementalMemory true --Odom/ResetCountdown 1--" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu localization:=true publish_tf_odom:=false publish_tf_map:=false
     
     roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3 --Grid/Sensor 1 --Grid/CellSize 0.07 --Grid/3D false --Vis/CorGuessWinSize 60 Odom/Strategy 0 --Odom/FilteringStrategy 0 --OdomF2M/MaxSize 5000 --Grid/Sensor 2 --OdomF2M/ScanMaxSize 5000 --Odom/ResetCountdown 1--" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=true wait_imu_to_init:=true imu_topic:=/rtabmap/imu
     
     roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3 --Grid/Sensor 0 --Grid/CellSize 0.07 --Grid/3D false --Vis/CorGuessWinSize 60 Odom/Strategy 0 --Odom/FilteringStrategy 0 --OdomF2M/MaxSize 5000 --OdomF2M/ScanMaxSize 5000 --Odom/ResetCountdown 1 Mem/IncrementalMemory true--" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=true wait_imu_to_init:=true imu_topic:=/rtabmap/imu subscribe_scan:=true

     
     roslaunch rtabmap_ros rtabmap.launch localization:=true

     
     ```
2. **Map Save**

    a. Go to socut_slam, save the map (Orin)
    ```
    rosrun map_server map_saver -f /home/orin/catkin_ws/src/scout_bringup/scout_slam/maps/map2 map:=/rtabmap/grid_map
    ```
3. **Navigation**

    a. Navigation (Orin)
    ```
    roslaunch rplidar_ros rplidar_s1.launch
    roslaunch scout_localization ekf_camera_rtab.launch
    roslaunch scout_navigation scout_navigation_rtab.launch
    ```

