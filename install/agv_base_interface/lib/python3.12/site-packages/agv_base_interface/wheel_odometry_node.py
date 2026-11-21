
#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        self.declare_parameter('odom_vx_std', 0.03)
        self.declare_parameter('odom_vy_std', 0.01)
        self.declare_parameter('odom_wz_std', 0.05)
        self.add_on_set_parameters_callback(
            self.validate_parameters
        )
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_separation', 0.250)
        self.declare_parameter('left_joint', 'wheel_left_joint')
        self.declare_parameter('right_joint', 'wheel_right_joint')
        self.declare_parameter(
            'input_topic', '/wheel/encoder_states')
        self.declare_parameter(
            'output_topic', '/odom_encoder_test')

        self.radius = float(
            self.get_parameter('wheel_radius').value)
        self.separation = float(
            self.get_parameter('wheel_separation').value)

        if self.radius <= 0 or self.separation <= 0:
            raise ValueError('Invalid wheel dimensions')

        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last = None

        self.pub = self.create_publisher(
            Odometry,
            self.get_parameter('output_topic').value,
            10
        )

        self.sub = self.create_subscription(
            JointState,
            self.get_parameter('input_topic').value,
            self.on_encoder,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Wheel odometry: R={self.radius}, '
            f'L={self.separation}'
        )

    @staticmethod
    def stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_encoder(self, msg):
        try:
            li = msg.name.index(self.left_joint)
            ri = msg.name.index(self.right_joint)

            left = float(msg.position[li])
            right = float(msg.position[ri])
        except (ValueError, IndexError):
            return

        if not math.isfinite(left) or not math.isfinite(right):
            return

        t = self.stamp_to_sec(msg.header.stamp)

        v = 0.0
        w = 0.0

        if self.last is None or t < self.last[2]:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.last = (left, right, t)
        else:
            dt = t - self.last[2]

            if dt <= 0.0:
                return

            # Quãng đường mỗi bánh
            dl = (left - self.last[0]) * self.radius
            dr = (right - self.last[1]) * self.radius

            # Động học robot vi sai
            ds = (dr + dl) / 2.0
            dyaw = (dr - dl) / self.separation

            # Tích phân pose theo góc giữa bước
            self.x += ds * math.cos(
                self.yaw + dyaw / 2.0)
            self.y += ds * math.sin(
                self.yaw + dyaw / 2.0)

            self.yaw = math.atan2(
                math.sin(self.yaw + dyaw),
                math.cos(self.yaw + dyaw)
            )

            v = ds / dt
            w = dyaw / dt

            self.last = (left, right, t)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'

        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y

        out.pose.pose.orientation.z = math.sin(
            self.yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(
            self.yaw / 2.0)

        out.twist.twist.linear.x = v
        out.twist.twist.angular.z = w

        # Do khong chac chan gia dinh ban dau.
        # Can hieu chinh theo encoder thuc te.
        pose_stds = [
            0.10, 0.10, 1000.0,
            1000.0, 1000.0, 0.10
        ]

        twist_stds = [
            float(self.get_parameter('odom_vx_std').value),
            float(self.get_parameter('odom_vy_std').value),
            1000.0,
            1000.0,
            1000.0,
            float(self.get_parameter('odom_wz_std').value)
        ]

        for i in range(6):
            out.pose.covariance[i * 6 + i] = (
                pose_stds[i] ** 2
            )
            out.twist.covariance[i * 6 + i] = (
                twist_stds[i] ** 2
            )

        self.pub.publish(out)

    def validate_parameters(self, params):
        for p in params:
            if p.name in (
                'odom_vx_std',
                'odom_vy_std',
                'odom_wz_std'
            ):
                if (
                    not isinstance(p.value, float)
                    or not math.isfinite(p.value)
                    or not 0.0 < p.value <= 100.0
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=f'Invalid parameter: {p.name}'
                    )

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
