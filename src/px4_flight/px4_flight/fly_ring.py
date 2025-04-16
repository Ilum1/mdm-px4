import rclpy
import math
import time
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus

class MultiDroneFlight(Node):
    def __init__(self, drone_namespace=''):
        super().__init__('multi_drone_flight', namespace=drone_namespace)

        # Check if the parameter already exists before declaring it
        if not self.has_parameter('radius'):
            self.declare_parameter('radius', 5.0)
        self.radius = self.get_parameter('radius').value

        if not self.has_parameter('altitude'):
            self.declare_parameter('altitude', -5.0)
        self.altitude = self.get_parameter('altitude').value

        if not self.has_parameter('speed'):
            self.declare_parameter('speed', 5.0)
        self.speed = self.get_parameter('speed').value

        self.center_x = 0.0
        self.center_y = 0.0
        self.angle = 0.0
        self.start_time = time.time()
        self.is_armed = False
        self.ready = False
        self.setpoint_counter = 0

        # Use relative topic names (they'll get prefixed with namespace)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', 10)

        # Subscribe to vehicle status to wait for PX4 to be ready
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            10
        )

        # Send a setpoint at a fixed interval
        self.create_timer(0.1, self.send_setpoint)

    def vehicle_status_callback(self, msg):
        # Check if PX4 is ready (nav_state 0 means the drone is in manual mode)
        if not self.ready and msg.nav_state == 0:  # Or use msg.arming_state == 0 for armed check
            self.ready = True
            self.get_logger().info('PX4 ready. Arming and setting Offboard mode...')
            self.arm()
            self.set_offboard_mode()

    def arm(self):
        if not self.is_armed:
            self.get_logger().info('Arming...')
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.is_armed = True
        else:
            self.get_logger().info('Already armed.')

    def set_offboard_mode(self):
        # Send offboard mode command to PX4
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Offboard mode
        self.get_logger().info('Offboard mode set.')

    def send_vehicle_command(self, command, *params):
        # Helper method to send commands to PX4
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = params[0] if len(params) > 0 else 0.0
        cmd.param2 = params[1] if len(params) > 1 else 0.0
        cmd.param3 = params[2] if len(params) > 2 else 0.0
        cmd.param4 = params[3] if len(params) > 3 else 0.0
        self.vehicle_command_pub.publish(cmd)

    def send_setpoint(self):
        elapsed_time = time.time() - self.start_time
        self.angle = (elapsed_time * self.speed / self.radius) % (2 * math.pi)
        x = self.center_x + self.radius * math.cos(self.angle)
        y = self.center_y + self.radius * math.sin(self.angle)

        traj_msg = TrajectorySetpoint()
        traj_msg.position = [x, y, self.altitude]
        traj_msg.yaw = self.angle + math.pi / 2
        traj_msg.timestamp = int(time.time() * 1e6)
        self.trajectory_pub.publish(traj_msg)

        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.timestamp = int(time.time() * 1e6)
        self.offboard_mode_pub.publish(offboard_msg)

        self.get_logger().info(f'Setpoint: x={x:.2f}, y={y:.2f}, z={self.altitude:.2f}')
        self.get_logger().info(f"Drone is at position x={x:.2f}, y={y:.2f}, z={self.altitude:.2f}")

        # Increment setpoint counter and send offboard mode once enough setpoints are sent
        self.setpoint_counter += 1
        if self.setpoint_counter == 10:
            self.get_logger().info('Sending offboard mode after 10 setpoints.')
            self.set_offboard_mode()
            self.arm()

def main(args=None):
    rclpy.init(args=args)

    px4_namespaces = ['px4_1', 'px4_2', 'px4_3']
    radii = [5.0, 10.0, 15.0]

    nodes = []

    for i, ns in enumerate(px4_namespaces):
        node = MultiDroneFlight(drone_namespace=ns)
        # Set parameters directly instead of declaring them
        node.radius = radii[i]
        nodes.append(node)

    executor = rclpy.executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
