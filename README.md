# arrow_key_teleop_drive

this is a ros2 teleop package similar to the normal teleop packge but using arrow keys to drive your robot in a fun way. it uses the pynput python library.

## Dependencies

ensure the following are installed on your PC before you start cloning and using the package

- setuptools for ros2 python packages

  ```shell
  pip3 install setuptools==58.2.0
  ```
- pynput python library for keyboard interfacing

  ```shell
  pip3 install pynput
  ```

## Setting up the package in your ros workspace
- clone (recommended) or Download the repo or in the src folder of your preferred ROS2 workspace.

  ```shell
  git clone https://github.com/samuko-things/arrow_key_teleop_drive.git
  ```
- To build the arrow_key_teleop_drive package, go to the root folder of your ros2 workspace and run the following command below. (DON'T FORGET TO SOURCE YOUR WORKSPACE)

  ```shell
  colcon build --packages-select arrow_key_teleop_drive --symlink-install
  ```

<br/>
<br/>


## Drive your robot with teleop

NOTE that the package publishes to the /cmd_vel topic. It requires you to set the linear velocity (v) and angular velocity (w), you want your robot to move at, as argument to run the package. Below is an example of how to run the package to drive your robot.

  ```shell
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive
  ```
drive the robot easily using the arrow keys

```
---------------------------------------------------
drive around with arrow keys:

  [forward-left]  [forward]    [forward-right]
                      |
  [rotate left] -------------- [rotate right]
                      |
  [reverse-left]  [reverse]    [reverse-right]

stops when no arrow key is pressed

R - reset to default speed

Q - increase v by +0.05
Z - reduce v by -0.05

W - increase w by +0.1
X - reduce w by -0.1
----------------------------------------------------
```