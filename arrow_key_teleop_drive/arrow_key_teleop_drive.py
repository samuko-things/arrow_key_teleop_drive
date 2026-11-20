import sys
import os
import signal

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Header

from pynput.keyboard import Key, Listener




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

q/z : increase/decrease both speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

H - toggle Holonomic (Straffing) Mode
CTRL+C - quit program
--------------------------------------------------------
"""



class ArrowKeyTeleop(Node):
  def __init__(self):
    super().__init__(node_name="arrow_key_teleop_drive")

    # Declare parameters in a single batch
    self.declare_parameters(
        namespace='',
        parameters=[
            ('speed', 0.25),
            ('turn', 0.8),
            ('stamped', True),
            ('frame_id', 'odom'),
            ('topic', 'cmd_vel_teleop'),
        ]
    )

    # Retrieve parameter values
    self.speed = self.get_parameter('speed').value
    self.turn = self.get_parameter('turn').value
    self.stamped = self.get_parameter('stamped').value
    self.frame_id = self.get_parameter('frame_id').value
    self.topic = self.get_parameter('topic').value

    self.linear_vel = self.speed
    self.angular_vel = self.turn

    self.vx = 0.0
    self.vy = 0.0
    self.w = 0.0

    self.prev_vx = self.vx
    self.prev_vy = self.vy
    self.prev_w = self.w

    if self.stamped:
        self.send_cmd = self.create_publisher(TwistStamped, self.topic, 10)
        self.msg = f'Publishing TwistStamped Message on {self.topic}'
    else:
        self.send_cmd = self.create_publisher(Twist, self.topic, 10)
        self.msg = f'Publishing Twist Message on {self.topic}'

    self.publish_freq = 10.0
    self.timer = self.create_timer(1.0 / self.publish_freq, self.timer_callback)

    self.status = 0
    self.holonomic_mode = False
    self.upPressed = False
    self.downPressed = False
    self.leftPressed = False
    self.rightPressed = False

    # Attach listener to self so it persists and can be stopped on shutdown
    self.listener = Listener(
        on_press=self.on_press,
        on_release=self.on_release,
        suppress=True
    )
    self.listener.start()

    self.get_logger().info(msg)
    self.get_logger().info(self.msg)

  def print_speed(self):
    if self.prev_vx == self.vx and self.prev_vy == self.vy and self.prev_w == self.w:
      pass
    else:
      if (self.status == 20):
        self.get_logger().info(msg)
      self.status = (self.status + 1) % 21

      if self.holonomic_mode:
        self.get_logger().info(f'{self.msg}\nHOLONOMIC MODE: true\nvx(m/s)={round(self.vx,3)}\tvy(m/s)={round(self.vy,3)}\tw(rad/s)=0.000\n')
      else:
        self.get_logger().info(f'{self.msg}\nHOLONOMIC MODE: false\nvx(m/s)={round(self.vx,3)}\tvy(m/s)=0.000\tw(rad/s)={round(self.w,3)}\n')

      self.prev_vx = self.vx
      self.prev_vy = self.vy
      self.prev_w = self.w
  
  def reset_speed(self):
    self.linear_vel = self.speed
    self.angular_vel = self.turn
    self.get_logger().info(f'Reset Speed -> LINEAR:{self.linear_vel}, ANGULAR:{self.angular_vel}')


  def publish_cmd_vel(self, vx, vy, w):
    if self.stamped:
      cmd_vel = TwistStamped()
      cmd_vel.header = Header()
      cmd_vel.header.stamp = self.get_clock().now().to_msg()
      cmd_vel.header.frame_id = self.frame_id
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
      self.vx = self.linear_vel
      self.vy = self.linear_vel
      self.w = self.angular_vel
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.upPressed and self.rightPressed:
      self.vx = self.linear_vel
      self.vy = -self.linear_vel
      self.w = -self.angular_vel
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.downPressed and self.leftPressed:
      self.vx = -self.linear_vel
      self.vy = self.linear_vel
      self.w = self.angular_vel
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.downPressed and self.rightPressed:
      self.vx = -self.linear_vel
      self.vy = -self.linear_vel
      self.w = -self.angular_vel
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.upPressed:
      self.vx = self.linear_vel
      self.vy = 0.000
      self.w = 0.000
      self.publish_cmd_vel(self.vx, self.vy, self.w)
    
    elif self.downPressed:
      self.vx = -self.linear_vel
      self.vy = 0.000
      self.w = 0.000
      self.publish_cmd_vel(self.vx, self.vy, self.w)

    elif self.leftPressed:
      self.vx = 0.000
      self.vy = self.linear_vel
      self.w = self.angular_vel
      self.publish_cmd_vel(self.vx, self.vy, self.w)
    
    elif self.rightPressed:
      self.vx = 0.000
      self.vy = -self.linear_vel
      self.w = -self.angular_vel
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
      # if key.char == 'Q' or key.char == 'q':
      #   os.kill(os.getpid(), signal.SIGINT)

      if key.char == 'R' or key.char == 'r':
        self.reset_speed()

      if key.char == 'H' or key.char == 'h':
        if self.holonomic_mode:
          self.holonomic_mode = False
          self.get_logger().info("HOLONOMIC MODE: false\n")
        else:
          self.holonomic_mode = True
          self.get_logger().info("HOLONOMIC MODE: true\n")

      if key.char == 'Q' or key.char == 'q':
        self.linear_vel += round(0.1*self.linear_vel, 3)
        self.angular_vel += round(0.1*self.angular_vel, 3)
        
        self.get_logger().info(f'Increase Speed -> LINEAR:{self.linear_vel}, ANGULAR:{self.angular_vel}')

      elif key.char == 'Z' or key.char == 'z':
        self.linear_vel -= round(0.1*self.linear_vel, 3)
        self.angular_vel -= round(0.1*self.angular_vel, 3)

        if self.linear_vel < 0.001:
          self.linear_vel = 0.0
        if self.angular_vel < 0.001:
          self.angular_vel = 0.0
          
        self.get_logger().info(f'Decrease Speed -> LINEAR:{self.linear_vel}, ANGULAR:{self.angular_vel}')

      elif key.char == 'W' or key.char == 'w':
        self.linear_vel += round(0.1*self.linear_vel, 3)
        
        self.get_logger().info(f'Increase Speed -> LINEAR:{self.linear_vel}, ANGULAR:{self.angular_vel}')

      elif key.char == 'X' or key.char == 'x':
        self.linear_vel -= round(0.1*self.linear_vel, 3)

        if self.linear_vel < 0.001:
          self.linear_vel = 0.0
          
        self.get_logger().info(f'Decrease Speed -> LINEAR:{self.linear_vel}, ANGULAR:{self.angular_vel}')

      elif key.char == 'E' or key.char == 'e':
        self.angular_vel += round(0.1*self.angular_vel, 3)
        
        self.get_logger().info(f'Increase Speed -> LINEAR:{self.linear_vel}, ANGULAR:{self.angular_vel}')

      elif key.char == 'C' or key.char == 'c':
        self.angular_vel -= round(0.1*self.angular_vel, 3)

        if self.angular_vel < 0.001:
          self.angular_vel = 0.0
          
        self.get_logger().info(f'Decrease Speed -> LINEAR:{self.linear_vel}, ANGULAR:{self.angular_vel}')


            
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

  node = ArrowKeyTeleop()
  rclpy.spin(node)
  node.destroy_node()

  rclpy.shutdown() 



if __name__=='__main__':
  main()