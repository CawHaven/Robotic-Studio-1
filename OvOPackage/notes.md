## To launch Gazebo and turtlebot model
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo custom.launch.py use_sim_time:=True
```
Note that this one has a modified launch file

## To control Turtlebot
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

## 2D Cartographer
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map_slam.yaml use_sim_time:=True
```
OR
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_slam turtlebot3_slam.launch.py use_sim_time:=True
```
## 3D Cartographer
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch cartographer_ros demo_turtlebot3_3d.launch
```

## Rviz
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
OR
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/map_slam.yaml use_sim_time:=True
```

## Save Map
```
ros2 run nav2_map_server map_saver_cli -f ~/map_slam
```

## SLAM Filtered
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True scan_topic:=/filtered_scan
```

## Open sprint as node
```
ros2 run robotics_studio_1 sprint3
```

## Building Specific Package with out world modification
```
colcon build --packages-select robotics_studio_1
source install/setup.bash
```

## setting up custom world:
* inside of ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo
* move the world and models files into it
* inside of launch folder
* modify or add
```
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo),
        'worlds',
        'small_house.world' // <-- to this
    )
```



## Misc
```
// ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

// ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz use_sim_time:=True
```