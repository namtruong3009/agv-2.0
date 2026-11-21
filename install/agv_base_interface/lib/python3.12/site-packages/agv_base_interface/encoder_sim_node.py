
#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray


class EncoderSimNode(Node):
    def __init__(self):
        super().__init__('encoder_sim_node')

        self.declare_parameter('ticks_per_wheel_rev', 2048)
        self.declare_parameter('left_joint', 'wheel_left_joint')
        self.declare_parameter('right_joint', 'wheel_right_joint')
        self.declare_parameter('input_topic', '/joint_states')
        self.declare_parameter(
            'output_topic', '/wheel/encoder_states')

        self.ticks_per_rev = int(
            self.get_parameter('ticks_per_wheel_rev').value)
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value

        if self.ticks_per_rev <= 0:
            raise ValueError('ticks_per_wheel_rev must be > 0')

        self.rad_per_tick = 2.0 * math.pi / self.ticks_per_rev

        self.origin = None
        self.last_counts = (0, 0)
        self.last_stamp = None

        self.encoder_pub = self.create_publisher(
            JointState,
            self.get_parameter('output_topic').value,
            10
        )

        self.ticks_pub = self.create_publisher(
            Int64MultiArray, '/wheel/encoder_ticks', 10)

        self.sub = self.create_subscription(
            JointState,
            self.get_parameter('input_topic').value,
            self.on_joint_states,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Encoder: {self.ticks_per_rev} ticks/wheel_rev'
        )

    @staticmethod
    def stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_joint_states(self, msg):
        try:
            li = msg.name.index(self.left_joint)
            ri = msg.name.index(self.right_joint)

            left_angle = float(msg.position[li])
            right_angle = float(msg.position[ri])
        except (ValueError, IndexError):
            return

        if not math.isfinite(left_angle) or \
           not math.isfinite(right_angle):
            return

        stamp = self.stamp_to_sec(msg.header.stamp)

        # Reset khi thời gian mô phỏng quay về đầu
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.origin = None

        # Lấy góc hiện tại làm mốc encoder = 0
        if self.origin is None:
            self.origin = (left_angle, right_angle)
            self.last_counts = (0, 0)
            self.last_stamp = stamp

            counts = (0, 0)
            velocities = (0.0, 0.0)
        else:
            if stamp <= self.last_stamp:
                return

            counts = (
                round(
                    (left_angle - self.origin[0])
                    / self.rad_per_tick
                ),
                round(
                    (right_angle - self.origin[1])
                    / self.rad_per_tick
                )
            )

            dt = stamp - self.last_stamp

            velocities = (
                (counts[0] - self.last_counts[0])
                * self.rad_per_tick / dt,
                (counts[1] - self.last_counts[1])
                * self.rad_per_tick / dt
            )

            self.last_counts = counts
            self.last_stamp = stamp

        # Góc và tốc độ bánh đã lượng tử hóa
        out = JointState()
        out.header = msg.header
        out.name = [self.left_joint, self.right_joint]
        out.position = [
            c * self.rad_per_tick for c in counts
        ]
        out.velocity = list(velocities)
        self.encoder_pub.publish(out)

        # Xung encoder để giám sát
        ticks = Int64MultiArray()
        ticks.data = [int(counts[0]), int(counts[1])]
        self.ticks_pub.publish(ticks)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
