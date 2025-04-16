from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    launch_description = []

    drone_configs = [
        {"name": "px4_1", "radius": 5.0},
        {"name": "px4_2", "radius": 10.0},
        {"name": "px4_3", "radius": 15.0},
    ]

    # Optional: start Gazebo in headless mode if you're not already doing this manually
    launch_description.append(
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "--headless-rendering"],
            output="screen"
        )
    )

    # Only launch controller nodes
    for config in drone_configs:
        name = config["name"]
        radius = config["radius"]

        launch_description.append(
            Node(
                package='px4_flight',
                executable='fly_ring',
                name='controller',
                namespace=name,
                output='screen',
                parameters=[
                    {"radius": radius},
                    {"altitude": -5.0},
                    {"speed": 5.0},
                    {"drone_namespace": name},
                ]
            )
        )

    return LaunchDescription(launch_description)