import rclpy
import math
import time
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint

class MultiDroneFlight(Node):
    def __init__(self):
        self.is_armed = False

        super().__init__('multi_drone_flight')

        # Declare and get parameters
        self.declare_parameter('radius', 5.0)
        self.declare_parameter('altitude', -5.0)
        self.declare_parameter('speed', 5.0)

        self.radius = self.get_parameter('radius').value
        self.altitude = self.get_parameter('altitude').value
        self.speed = self.get_parameter('speed').value

        self.center_x = 0.0
        self.center_y = 0.0
        self.angle = 0.0
        self.start_time = time.time()

        # Use relative topic names — they will be automatically namespaced
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)

        self.create_timer(0.1, self.send_setpoint)
        self.create_timer(1.0, self.start_offboard_mode)

    def start_offboard_mode(self):
        self.get_logger().info('Setting mode to OFFBOARD...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.create_timer(0.5, self.arm)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(time.time() * 1e6)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        if not self.is_armed:
            self.get_logger().info('Arming...')
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.is_armed = True
        else:
            self.get_logger().info('Already armed.')


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

def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneFlight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
