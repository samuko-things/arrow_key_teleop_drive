import sys
import os
import signal

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Header

from pynput.keyboard import Key, Listener



arg_msg = """
Expected arguments:
<linear vel in (m/s)> <angular vel in (rad/sec)> <use_stamped_vel - true/false>
example arg -> 0.125 0.7 true
example arg -> 0.125 0.7 false
"""

def process_args_vel():
  try:
    v = float(sys.argv[1])
    w = float(sys.argv[2])

    if sys.argv[3] == "false" or sys.argv[3] == "False":
      use_stamped_vel = False
    elif sys.argv[3] == "true" or sys.argv[3] == "True":
      use_stamped_vel = True
    else:
      raise ValueError("Invalid third argument -> required: true/false")

    if use_stamped_vel == True:
      use_stamped_msg = "Publishing TwistStamped Msg on /cmd_vel_teleop"
    else:
      use_stamped_msg = "Publishing Twist Msg on /cmd_vel_teleop"

    msg = f'{use_stamped_msg}\nv={v} and w={w}'
    print(msg)
    return use_stamped_msg, use_stamped_vel, v, w
  except Exception as e:
    print(arg_msg)
    exit()



msg = """
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

H - toggle Holonomic (Straffing) Mode
Q - quit (ctrl C)
--------------------------------------------------------
"""



class ArrowKeyTeleop(Node):
  def __init__(self):
    super().__init__(node_name="arrow_key_teleop") # initialize the node name

    self.use_stamped_msg, self.use_stamped_vel, self.LINEAR_DEFAULT, self.ANGULAR_DEFAULT = process_args_vel()

    self.LINEAR = self.LINEAR_DEFAULT
    self.ANGULAR = self.ANGULAR_DEFAULT
    
    # also used during velocity printing
    self.vx = 0.000
    self.vy = 0.000
    self.w = 0.000

    self.prev_vx = self.vx
    self.prev_vy = self.vy
    self.prev_w = self.w
    #----------------------------

    if self.use_stamped_vel:
      self.send_cmd = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
    else:
      self.send_cmd = self.create_publisher(Twist, '/cmd_vel_teleop', 10)

    self.publish_freq = 10
    self.timer = self.create_timer(1/self.publish_freq, self.timer_callback)
    
    self.status = 0

    self.holonomic_mode = False
    self.upPressed = False
    self.downPressed = False
    self.leftPressed = False
    self.rightPressed = False

    # ...or, in a non-blocking fashion:
    listener = Listener(on_press=self.on_press, on_release=self.on_release, suppress=True)
    listener.start()

    print(msg)

  def print_speed(self):
    if self.prev_vx == self.vx and self.prev_vy == self.vy and self.prev_w == self.w:
      pass
    else:
      if (self.status == 20):
        print(msg)
      self.status = (self.status + 1) % 21

      if self.holonomic_mode:
        print(f'{self.use_stamped_msg}\nHOLONOMIC MODE: true\nvx(m/s)={round(self.vx,3)}\tvy(m/s)={round(self.vy,3)}\tw(rad/s)=0.000\n')
      else:
        print(f'{self.use_stamped_msg}\nHOLONOMIC MODE: false\nvx(m/s)={round(self.vx,3)}\tvy(m/s)=0.000\tw(rad/s)={round(self.w,3)}\n')

      self.prev_vx = self.vx
      self.prev_vy = self.vy
      self.prev_w = self.w
  
  def reset_speed(self):
    self.LINEAR = self.LINEAR_DEFAULT
    self.ANGULAR = self.ANGULAR_DEFAULT
    print(f'Reset to default-> LINEAR:{self.LINEAR}, ANGULAR:{self.ANGULAR}')


  def publish_cmd_vel(self, vx, vy, w):
    if self.use_stamped_vel:
      cmd_vel = TwistStamped()
      cmd_vel.header = Header()
      cmd_vel.header.stamp = self.get_clock().now().to_msg()
      cmd_vel.header.frame_id = 'odom'
      if self.holonomic_mode:
        cmd_vel.twist.linear.x = vx
        cmd_vel.twist.linear.y = vy
        cmd_vel.twist.angular.z = 0.0
      else:
        cmd_vel.twist.linear.x = vx
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.angular.z = w
      self.send_cmd.publish(cmd_vel)
    else:
      cmd_vel = Twist()
      if self.holonomic_mode:
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.angular.z = 0.0
      else:
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = w
      self.send_cmd.publish(cmd_vel)


  def timer_callback(self):
    if self.upPressed and self.leftPressed:
      self.vx = self.LINEAR
      self.vy = self.LINEAR
      self.w = self.ANGULAR
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.upPressed and self.rightPressed:
      self.vx = self.LINEAR
      self.vy = -self.LINEAR
      self.w = -self.ANGULAR
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.downPressed and self.leftPressed:
      self.vx = -self.LINEAR
      self.vy = self.LINEAR
      self.w = self.ANGULAR
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.downPressed and self.rightPressed:
      self.vx = -self.LINEAR
      self.vy = -self.LINEAR
      self.w = -self.ANGULAR
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.upPressed:
      self.vx = self.LINEAR
      self.vy = 0.000
      self.w = 0.000
      self.publish_cmd_vel(self.vx, self.vy, self.w)
    
    elif self.downPressed:
      self.vx = -self.LINEAR
      self.vy = 0.000
      self.w = 0.000
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.leftPressed:
      self.vx = 0.000
      self.vy = self.LINEAR
      self.w = self.ANGULAR
      self.publish_cmd_vel(self.vx, self.vy, self.w)
    
    elif self.rightPressed:
      self.vx = 0.000
      self.vy = -self.LINEAR
      self.w = -self.ANGULAR
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    else:
      self.vx = 0.000
      self.vy = 0.000
      self.w = 0.000
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    self.print_speed()



  def on_press(self, key):
    if key == Key.up:
      self.upPressed = True
      self.downPressed = False
            
    elif key == Key.down:
      self.upPressed = False
      self.downPressed = True

    if key == Key.left:
      self.leftPressed = True
      self.rightPressed = False
            
    elif key == Key.right:
      self.leftPressed = False
      self.rightPressed = True

    if hasattr(key, 'char'):
      if key.char == 'Q' or key.char == 'q':
        os.kill(os.getpid(), signal.SIGINT)

      if key.char == 'R' or key.char == 'r':
        self.reset_speed()

      if key.char == 'H' or key.char == 'h':
        if self.holonomic_mode:
          self.holonomic_mode = False
          print("HOLONOMIC MODE: false\n")
        else:
          self.holonomic_mode = True
          print("HOLONOMIC MODE: true\n")

      # elif key.char == 'Q' or key.char == 'q':
      #   self.LINEAR_DEFAULT += 0.05
      #   if self.LINEAR_DEFAULT > 1.0:
      #     self.LINEAR_DEFAULT = 1.0    
      #   print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.LINEAR_DEFAULT, self.ANGULAR_DEFAULT))

      # elif key.char == 'Z' or key.char == 'z':
      #   self.LINEAR_DEFAULT -= 0.05
      #   if self.LINEAR_DEFAULT < 0.05:
      #     self.LINEAR_DEFAULT = 0.05
      #   print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.LINEAR_DEFAULT, self.ANGULAR_DEFAULT))

      # elif key.char == 'W' or key.char == 'w':
      #   self.ANGULAR_DEFAULT += 0.1
      #   if self.ANGULAR_DEFAULT > 3.0:
      #     self.ANGULAR_DEFAULT = 3.0
      #   print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.LINEAR_DEFAULT, self.ANGULAR_DEFAULT))

      # elif key.char == 'X' or key.char == 'x':
      #   self.ANGULAR_DEFAULT -= 0.1
      #   if self.ANGULAR_DEFAULT < 0.1:
      #     self.ANGULAR_DEFAULT = 0.1
      #   print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.LINEAR_DEFAULT, self.ANGULAR_DEFAULT))


            
  def on_release(self, key):
    if key == Key.up:
      self.upPressed = False
            
    if key == Key.down:
      self.downPressed = False

    if key == Key.left:
      self.leftPressed = False
            
    if key == Key.right:
      self.rightPressed = False
        


def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)

  mobo_bot_teleop = ArrowKeyTeleop()
  rclpy.spin(mobo_bot_teleop)
  mobo_bot_teleop.destroy_node()

  rclpy.shutdown() 



if __name__=='__main__':
  main()