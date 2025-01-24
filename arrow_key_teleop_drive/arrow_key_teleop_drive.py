import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from pynput.keyboard import Key, Listener






arg_msg = """
You can also enter velocity args in format
<linear vel in (m/s)> <angular vel in (rad/sec)>
"""

def process_args_vel():
  try:
    v = float(sys.argv[1])
    w = float(sys.argv[2])
    msg = f'using v={v} and w={w}'
    print(msg)
    return v, w
  except Exception as e:
    v = 0.1
    w = 0.5
    msg = f'using v={v} and w={w}'
    print(msg)
    print(arg_msg)
    return v, w
    # # print(e)
    # print(arg_msg)
    # exit()



msg = """
This node takes arrow keypresses from the keyboard 
and publishes Twist (velocicty comands) messages to
control your diff drive robot. It makes use of the 
pynput keyboard python library

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
"""



class ArrowKeyTeleop(Node):
  def __init__(self):
    super().__init__(node_name="arrow_key_teleop") # initialize the node name

    self.default_v, self.default_w = process_args_vel()
    self.v = 0.000
    self.w = 0.000

    self.prev_v = self.v
    self.prev_w = self.w

    self.send_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
    self.status = 0

    self.upPressed = False
    self.downPressed = False
    self.leftPressed = False
    self.rightPressed = False


    # ...or, in a non-blocking fashion:
    listener = Listener(on_press=self.on_press, on_release=self.on_release)
    listener.start()

    print(msg)
  

  def reset_speed(self):
        self.speed = self.default_speed
        self.turn = self.default_turn

  def print_speed(self, v, w):
    if self.prev_v == self.v and self.prev_w == self.w:
      pass
    else:
      if (self.status == 20):
        print(msg)
      self.status = (self.status + 1) % 21

      print('currently:\tv(m/s)=%f\tw(rad/s)=%f' % (v, w))
      self.prev_v = self.v
      self.prev_w = self.w
  
  def reset_speed(self):
    self.default_v, self.default_w = process_args_vel()
    print('reset_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.default_v, self.default_w))


  def publish_cmd_vel(self, v, w):
    cmd_vel = Twist()
    cmd_vel.linear.x = v
    cmd_vel.linear.y = 0.000
    cmd_vel.linear.z = 0.000
    cmd_vel.angular.x = 0.000
    cmd_vel.angular.y = 0.000
    cmd_vel.angular.z = w
    self.send_cmd.publish(cmd_vel)

    self.print_speed(v, w)


  def timer_callback(self):
    if self.upPressed and self.leftPressed:
      self.v = self.default_v
      self.w = self.default_w
      self.publish_cmd_vel(self.v, self.w)

    elif self.upPressed and self.rightPressed:
      self.v = self.default_v
      self.w = -self.default_w
      self.publish_cmd_vel(self.v, self.w)

    elif self.downPressed and self.leftPressed:
      self.v = -self.default_v
      self.w = self.default_w
      self.publish_cmd_vel(self.v, self.w)

    elif self.downPressed and self.rightPressed:
      self.v = -self.default_v
      self.w = -self.default_w
      self.publish_cmd_vel(self.v, self.w)

    elif self.upPressed:
      self.v = self.default_v
      self.w = 0.000
      self.publish_cmd_vel(self.v, self.w)
    
    elif self.downPressed:
      self.v = -self.default_v
      self.w = 0.000
      self.publish_cmd_vel(self.v, self.w)

    elif self.leftPressed:
      self.v = 0.000
      self.w = self.default_w
      self.publish_cmd_vel(self.v, self.w)
    
    elif self.rightPressed:
      self.v = 0.000
      self.w = -self.default_w
      self.publish_cmd_vel(self.v, self.w)

    else:
      self.v = 0.000
      self.w = 0.000
      self.publish_cmd_vel(self.v, self.w)



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
      # if key.char in self.speed_ctrl_keys:
      if key.char == 'R' or key.char == 'r':
        self.reset_speed()

      elif key.char == 'Q' or key.char == 'q':
        self.default_v += 0.05
        if self.default_v > 1.0:
          self.default_v = 1.0    
        print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.default_v, self.default_w))

      elif key.char == 'Z' or key.char == 'z':
        self.default_v -= 0.05
        if self.default_v < 0.05:
          self.default_v = 0.05
        print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.default_v, self.default_w))

      elif key.char == 'W' or key.char == 'w':
        self.default_w += 0.1
        if self.default_w > 3.0:
          self.default_w = 3.0
        print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.default_v, self.default_w))

      elif key.char == 'X' or key.char == 'x':
        self.default_w -= 0.1
        if self.default_w < 0.1:
          self.default_w = 0.1
        print('new_speed:\tv(m/s)=%f\tw(rad/s)=%f' % (self.default_v, self.default_w))


            
  def on_release(self, key):
    if key == Key.up:
      self.upPressed = False
            
    if key == Key.down:
      self.downPressed = False

    if key == Key.left:
      self.leftPressed = False
            
    if key == Key.right:
      self.rightPressed = False

    if key == Key.esc:
      # Stop listener
      return False
        






def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  mobo_bot_teleop = ArrowKeyTeleop()

  # spin the node so the call back function is called
  rclpy.spin(mobo_bot_teleop)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  mobo_bot_teleop.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown() 



if __name__=='__main__':
  main()