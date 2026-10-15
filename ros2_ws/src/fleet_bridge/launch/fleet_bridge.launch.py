"""One fleet bridge per simulated robot; run alongside multi_robot.launch.py.

Credentials come from the environment (FLEET_USER, FLEET_PASSWORD; see
.env.example), never from launch arguments. From inside the vision_bot
container the fleet API on the Windows host is http://host.docker.internal:<port>.

  ros2 launch fleet_bridge fleet_bridge.launch.py \
      api_base_url:=http://host.docker.internal:8000/api/v1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# (namespace, fleet serial number) for each robot in multi_robot.launch.py
ROBOTS = (("robot1", "VB-SIM-1"), ("robot2", "VB-SIM-2"))


def generate_launch_description():
    api = LaunchConfiguration("api_base_url")
    bridges = [
        Node(package="fleet_bridge", executable="bridge_node", name="fleet_bridge",
             namespace=namespace, output="screen",
             parameters=[{"robot_id": serial, "api_base_url": api, "use_sim_time": True}])
        for namespace, serial in ROBOTS
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            "api_base_url",
            default_value=EnvironmentVariable("FLEET_API_BASE_URL",
                                              default_value="http://localhost:8000/api/v1")),
        *bridges,
    ])
