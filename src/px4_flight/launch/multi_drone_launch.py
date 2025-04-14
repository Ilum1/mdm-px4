from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

#env = os.environ.copy()
#env["PX4_SIM_MODEL"] = "gz_x500"


def generate_launch_description():
    launch_description = []

    drone_configs = [
        {"name": "drone1", "radius": 5.0, "x": 0, "y": 0},
        {"name": "drone2", "radius": 10.0, "x": 2, "y": 2},
        {"name": "drone3", "radius": 15.0, "x": -2, "y": -2},
    ]

    # Path to the ROMFS
    romfs_path = "/home/ros/PX4-Autopilot/ROMFS/px4fmu_common"

    # Start Gazebo Sim (headless)
    launch_description.append(
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "--headless-rendering"],
            output="screen"
        )
    )


    for config in drone_configs:
        drone_name = config["name"]
        radius = config["radius"]
        x = config["x"]
        y = config["y"]

        # Spawn drone model
        launch_description.append(
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "ros_gz_sim", "create",
                    "-name", drone_name,
                    "-x", str(x),
                    "-y", str(y),
                    "-z", "0.2",
                    "-topic", f"/model/{drone_name}/cmd_spawn"
                ],
                output="screen"
            )
        )

        # PX4 SITL instance - ensure unique instance ID for each drone
        # launch_description.append(
        #     ExecuteProcess(
        #         cmd=[
        #             "/home/ros/PX4-Autopilot/build/px4_sitl_default/bin/px4",
        #             romfs_path,  # Use the updated romfs_path here
        #             "-i", str(drone_configs.index(config) + 1)  # +1 to avoid index 0
        #         ],
        #         output="screen"
        #     )
        # )
        launch_description.append(
            ExecuteProcess(
                cmd=[
                    "/home/ros/PX4-Autopilot/build/px4_sitl_default/bin/px4",
                    romfs_path,
                    "-i", str(drone_configs.index(config) + 1)
                ]#,
                #env=env, 
                #env={**os.environ, "PX4_SIM_MODEL": "gz_x500"}#,
                #output="screen"
            )
        )

        # Launch your controller node with namespace
        launch_description.append(
            Node(
                package='px4_flight',
                executable='fly_ring',
                name=f"{drone_name}_controller",
                namespace=drone_name,
                output='screen',
                parameters=[
                    {"radius": radius},
                    {"altitude": -5.0},
                    {"speed": 5.0}
                ]
            )
        )

    return LaunchDescription(launch_description)
