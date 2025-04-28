import rclpy
import math
import time
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus

class MultiDroneFlight(Node):
    def __init__(self, drone_namespace=''):
        # Initialize the Node with a namespace (for each drone)
        super().__init__('multi_drone_flight', namespace=drone_namespace)

        # Declare parameters with default values if they don't exist
        if not self.has_parameter('radius'):
            self.declare_parameter('radius', 5.0)  # Default radius is 5 meters
        self.radius = self.get_parameter('radius').value

        if not self.has_parameter('altitude'):
            self.declare_parameter('altitude', -5.0)  # Default altitude is -5 meters
        self.altitude = self.get_parameter('altitude').value

        if not self.has_parameter('speed'):
            self.declare_parameter('speed', 5.0)  # Default speed is 5 meters per second
        self.speed = self.get_parameter('speed').value

        # Set initial values for drone position and state
        self.center_x = 0.0
        self.center_y = 0.0
        self.angle = 0.0
        self.start_time = time.time()
        self.is_armed = False
        self.ready = False
        self.setpoint_counter = 0

        # Create publishers for offboard control, trajectory setpoint, and vehicle commands
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', 10)

        # Subscribe to vehicle status to check if PX4 is ready for offboard mode
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            10
        )

        # Timer to send setpoints periodically (every 0.1 seconds)
        self.create_timer(0.1, self.send_setpoint)

    def vehicle_status_callback(self, msg):
        # Check if PX4 is ready (nav_state 0 means the drone is in manual mode)
        if not self.ready and msg.nav_state == 0:  # You can also use msg.arming_state == 0 for armed check
            self.ready = True
            self.get_logger().info('PX4 ready. Arming and setting Offboard mode...')
            self.arm()  # Arm the drone
            self.set_offboard_mode()  # Set Offboard mode for the drone

    def arm(self):
        # Arm the drone if it's not already armed
        if not self.is_armed:
            self.get_logger().info('Arming...')
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)  # 1.0 means arm
            self.is_armed = True
        else:
            self.get_logger().info('Already armed.')

    def set_offboard_mode(self):
        # Set the drone to Offboard mode
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # 6.0 means Offboard mode
        self.get_logger().info('Offboard mode set.')

    def send_vehicle_command(self, command, *params):
        # Helper method to send vehicle commands to PX4
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = params[0] if len(params) > 0 else 0.0
        cmd.param2 = params[1] if len(params) > 1 else 0.0
        cmd.param3 = params[2] if len(params) > 2 else 0.0
        cmd.param4 = params[3] if len(params) > 3 else 0.0
        self.vehicle_command_pub.publish(cmd)

    def send_setpoint(self):
        # Calculate the drone's position in a circular path
        elapsed_time = time.time() - self.start_time  # Time since the start
        self.angle = (elapsed_time * self.speed / self.radius) % (2 * math.pi)  # Angle for circular path
        x = self.center_x + self.radius * math.cos(self.angle)  # X position
        y = self.center_y + self.radius * math.sin(self.angle)  # Y position

        # Create and publish a TrajectorySetpoint message with calculated position
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [x, y, self.altitude]  # Position (x, y, z)
        traj_msg.yaw = self.angle + math.pi / 2  # Yaw angle
        traj_msg.timestamp = int(time.time() * 1e6)  # Timestamp in microseconds
        self.trajectory_pub.publish(traj_msg)

        # Send OffboardControlMode message to enable position control
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True  # Enabling position control
        offboard_msg.timestamp = int(time.time() * 1e6)  # Timestamp in microseconds
        self.offboard_mode_pub.publish(offboard_msg)

        # Log the current setpoint and position
        self.get_logger().info(f'Setpoint: x={x:.2f}, y={y:.2f}, z={self.altitude:.2f}')
        self.get_logger().info(f"Drone is at position x={x:.2f}, y={y:.2f}, z={self.altitude:.2f}")

        # After 10 setpoints, send offboard mode again and arm the drone
        self.setpoint_counter += 1
        if self.setpoint_counter == 10:
            self.get_logger().info('Sending offboard mode after 10 setpoints.')
            self.set_offboard_mode()
            self.arm()

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2

    # List of namespaces for the drones
    px4_namespaces = ['px4_1', 'px4_2', 'px4_3']
    # List of radii for each drone's circular path
    radii = [5.0, 10.0, 15.0]

    nodes = []

    # Initialize nodes for each drone with corresponding namespace and radius
    for i, ns in enumerate(px4_namespaces):
        node = MultiDroneFlight(drone_namespace=ns)
        node.radius = radii[i]  # Set the radius for each drone
        nodes.append(node)

    executor = rclpy.executors.MultiThreadedExecutor()  # Create an executor to handle multiple nodes
    for node in nodes:
        executor.add_node(node)  # Add each node to the executor

    try:
        executor.spin()  # Start spinning the executor (keeping the nodes running)
    finally:
        for node in nodes:
            node.destroy_node()  # Clean up nodes after execution
        rclpy.shutdown()  # Shut down ROS 2
