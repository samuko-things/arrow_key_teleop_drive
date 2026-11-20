from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  
  speed = LaunchConfiguration('speed')
  turn = LaunchConfiguration('turn')
  stamped = LaunchConfiguration('stamped')
  frame_id = LaunchConfiguration('frame_id')
  topic = LaunchConfiguration('topic')

  declare_speed_cmd = DeclareLaunchArgument(
      'speed', 
      default_value='0.5', 
      description='default linear velocity'
  )
  declare_turn_cmd = DeclareLaunchArgument(
      'turn', 
      default_value='1.0', 
      description='default angular velocity'
  )
  declare_stamped_cmd = DeclareLaunchArgument(
      'stamped', 
      default_value='False', 
      description='whether to use stamped velocity or not'
  )
  declare_frame_id_cmd = DeclareLaunchArgument(
      'frame_id', 
      default_value='', 
      description='frame_id if stamped command is being used'
  )
  declare_topic_cmd = DeclareLaunchArgument(
      'topic', 
      default_value='cmd_vel', 
      description='topic to publish command velocity on'
  )

  # Explicitly enforce types so passing e.g. speed:=1 doesn't cause a type mismatch
  params = {
      'speed': ParameterValue(speed, value_type=float),
      'turn': ParameterValue(turn, value_type=float),
      'stamped': ParameterValue(stamped, value_type=bool),
      'frame_id': ParameterValue(frame_id, value_type=str),
      'topic': ParameterValue(topic, value_type=str),
  }

  teleop_node = Node(
      package='arrow_key_teleop_drive',
      executable='arrow_key_teleop_drive',
      # name='arrow_key_teleop_drive',
      output='screen',
      # emulate_tty=True,  # Ensures print() and logs flush immediately to the terminal
      parameters=[params],
  )

  ld = LaunchDescription()
  
  ld.add_action(declare_speed_cmd)
  ld.add_action(declare_turn_cmd)
  ld.add_action(declare_stamped_cmd)
  ld.add_action(declare_frame_id_cmd)
  ld.add_action(declare_topic_cmd)
  
  ld.add_action(teleop_node)

  return ld