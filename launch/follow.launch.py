from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    # start turtlesim
    turtlesim_node = Node(package='turtlesim', executable='turtlesim_node', name='sim')

    # spawn turtle2 after turtlesim starts (we use a TimerAction to delay spawn)
    spawn_node = Node(package='turtlesim', executable='turtle_spawn', name='spawn_turtle')  # turtlesim provides spawn service tool differently; we'll call service manually below

    # Note: we will not use a spawn Node executable because default turtlesim package does not include a spawn executable by that name.
    # Instead, after launching, run the service call to spawn turtle2:
    # ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 8.0, theta: 0.0, name: 'turtle2'}"

    leader_tf = Node(package='turtle_tf_follow', executable='leader_tf_broadcaster')
    turtle2_tf = Node(package='turtle_tf_follow', executable='turtle2_tf_broadcaster')
    follower = Node(package='turtle_tf_follow', executable='follower_controller')

    ld.add_action(turtlesim_node)
    # small delays to let turtlesim initialize before nodes that subscribe to /turtleX/pose
    ld.add_action(TimerAction(period=1.0, actions=[leader_tf]))
    ld.add_action(TimerAction(period=1.2, actions=[turtle2_tf]))
    ld.add_action(TimerAction(period=1.4, actions=[follower]))

    return ld
