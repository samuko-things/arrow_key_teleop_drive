# arrow_key_teleop_drive

this is a ros2 teleop package similar to the normal teleop packge but using arrow keys to drive your robot in a fun way. it uses the pynput python library.


## Dependencies

ensure the following are installed on your PC before you start cloning and using the package

- pynput python library for keyboard interfacing

  ```shell
  sudo apt install python3-pynput
  ```


## Setting up the package in your ros workspace

- clone the repo or in the src folder of your preferred ROS2 workspace.

```shell
git clone https://github.com/samuko-things/arrow_key_teleop_drive.git
```

- To build the arrow_key_teleop_drive package, go to the root folder of your ros2 workspace and run the following command below. (DON'T FORGET TO SOURCE YOUR WORKSPACE)

```shell
colcon build --packages-select arrow_key_teleop_drive --symlink-install
```


## Drive your robot with teleop

```
This node takes arrow keypresses from the keyboard 
and publishes TwistSTamped or Twist (velocicty comands)
messages to control your robot. It makes use of the  
pynput keyboard python library

NOTE!: it runs when terminal is not in focus 
-------------------------------------------------------
drive around with arrow keys:

  [forward-left]  [forward]    [forward-right]
                      |
  [rotate left] -------------- [rotate right]
                      |
  [reverse-left]  [reverse]    [reverse-right]

stops when no arrow key is pressed

q/z : increase/decrease both speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

H - toggle Holonomic (Straffing) Mode
CTRL+C - quit program
--------------------------------------------------------
```

- basic telop node launch

```shell
ros2 launch arrow_key_teleop_drive arrow_key_teleop_drive.launch.py
```

- sample launch with parameters
  
```shell
ros2 launch arrow_key_teleop_drive arrow_key_teleop_drive.launch.py speed:=0.25 turn:=0.8 topic:=cmd_vel_teleop stamped:=True frame_id:=odom
```

## Parameters

- **stamped** (bool, default: false)

> If false (the default), publish a geometry_msgs/msg/Twist message. If true, publish a geometry_msgs/msg/TwistStamped message.

- **frame_id** (string, default: '')

> When stamped is true, the frame_id to use when publishing the geometry_msgs/msg/TwistStamped message.

- **speed** (double, default: 0.5)

> The speed the node starts with by default.

- **turn** (double, default: 1.0)

> The turn rate (rad/s) the node starts with by default.

- **topic** (string, default: cmd_vel)

> Topic you want to publish to.


## Possible Future Improvements
- adding speed ramping
- etc.