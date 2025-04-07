import rclpy
import math
import time
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint

class RingFlight(Node):
    def __init__(self):
        super().__init__('ring_flight')
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        self.timer = self.create_timer(0.1, self.send_setpoint)
        self.radius = 40.0  # Radius of the circular path
        self.altitude = -5.0  # Altitude in NED frame (negative for up)
        self.speed = 20.0  # Speed in m/s
        self.angle = 1.0  # Starting angle
        self.center_x = 0.0
        self.center_y = 0.0

        self.declare_parameters(namespace='', parameters=[
            ('radius', self.radius),
            ('altitude', self.altitude),
            ('speed', self.speed)
        ])

        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        self.start_time = time.time()
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.arm()

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
        self.get_logger().info('Arming...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def send_setpoint(self):
        elapsed_time = time.time() - self.start_time
        self.angle = (elapsed_time * self.speed / self.radius) % (2 * math.pi)
        x = self.center_x + self.radius * math.cos(self.angle)
        y = self.center_y + self.radius * math.sin(self.angle)

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [x, y, self.altitude]
        trajectory_msg.yaw = self.angle + math.pi / 2  # Face forward
        trajectory_msg.timestamp = int(time.time() * 1e6)
        self.trajectory_pub.publish(trajectory_msg)

        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.timestamp = int(time.time() * 1e6)
        self.offboard_mode_pub.publish(offboard_msg)

        self.get_logger().info(f'Setpoint: x={x:.2f}, y={y:.2f}, z={self.altitude:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = RingFlight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
