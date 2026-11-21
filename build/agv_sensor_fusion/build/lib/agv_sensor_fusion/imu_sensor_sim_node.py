	
#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, MagneticField


def world_to_body(q, vx, vy, vz):
    # q = rotation body -> world
    # inverse(q) rotates world -> body
    n = math.sqrt(
        q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    )
    if n < 1e-8:
        return None

    x = -q.x / n
    y = -q.y / n
    z = -q.z / n
    w = q.w / n

    tx = 2.0 * (y*vz - z*vy)
    ty = 2.0 * (z*vx - x*vz)
    tz = 2.0 * (x*vy - y*vx)

    return (
        vx + w*tx + y*tz - z*ty,
        vy + w*ty + z*tx - x*tz,
        vz + w*tz + x*ty - y*tx
    )


class ImuSensorSimNode(Node):

    def __init__(self):
        super().__init__('imu_sensor_sim_node')

        # MPU6050: standard deviation
        self.declare_parameter('gyro_xy_std', 0.01)
        self.declare_parameter('imu_wz_std', 0.03)
        self.declare_parameter('accel_std', 0.08)
        self.declare_parameter('gyro_z_bias', 0.002)

        # QMC5883L: magnetic field in Tesla
        self.declare_parameter('mag_std', 0.0000003)
        self.declare_parameter('mag_bias_x', 0.0)
        self.declare_parameter('mag_bias_y', 0.0)
        self.declare_parameter('mag_bias_z', 0.0)

        self.add_on_set_parameters_callback(
            self.validate_parameters
        )

        self.rng = random.Random(42)

        self.imu_pub = self.create_publisher(
            Imu, '/imu/data_raw', qos_profile_sensor_data
        )

        self.mag_pub = self.create_publisher(
            MagneticField, '/imu/mag',
            qos_profile_sensor_data
        )

        self.sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'MPU6050 + QMC5883L simulator started'
        )

    def param(self, name):
        return float(self.get_parameter(name).value)

    def validate_parameters(self, params):
        noise_names = {
            'gyro_xy_std', 'imu_wz_std',
            'accel_std', 'mag_std'
        }

        for p in params:
            if p.name not in noise_names:
                continue

            value = p.value
            if (not isinstance(value, float)
                    or not math.isfinite(value)
                    or value <= 0.0):
                return SetParametersResult(
                    successful=False,
                    reason=f'Invalid {p.name}'
                )

        return SetParametersResult(successful=True)

    def noisy(self, value, std):
        return value + self.rng.gauss(0.0, std)

    def imu_callback(self, msg):
        gyro_xy_std = self.param('gyro_xy_std')
        gyro_z_std = self.param('imu_wz_std')
        accel_std = self.param('accel_std')
        gyro_z_bias = self.param('gyro_z_bias')
        mag_std = self.param('mag_std')

        # ------- MPU6050 simulation -------
        raw = Imu()
        raw.header = msg.header

        raw.angular_velocity.x = self.noisy(
            msg.angular_velocity.x, gyro_xy_std
        )
        raw.angular_velocity.y = self.noisy(
            msg.angular_velocity.y, gyro_xy_std
        )
        raw.angular_velocity.z = self.noisy(
            msg.angular_velocity.z + gyro_z_bias,
            gyro_z_std
        )

        raw.linear_acceleration.x = self.noisy(
            msg.linear_acceleration.x, accel_std
        )
        raw.linear_acceleration.y = self.noisy(
            msg.linear_acceleration.y, accel_std
        )
        raw.linear_acceleration.z = self.noisy(
            msg.linear_acceleration.z, accel_std
        )

        # Raw MPU6050 does not measure orientation
        raw.orientation_covariance[0] = -1.0

        gyro_vars = [
            gyro_xy_std**2,
            gyro_xy_std**2,
            gyro_z_std**2
        ]

        for i in range(3):
            raw.angular_velocity_covariance[i*3+i] = (
                gyro_vars[i]
            )
            raw.linear_acceleration_covariance[i*3+i] = (
                accel_std**2
            )

        # ------- QMC5883L simulation -------
        # Approximate Earth magnetic field, ENU:
        # North = +Y, Up = +Z
        # Magnitudes are illustrative, not site calibrated.
        magnetic_world = (
            0.0,
            25.0e-6,
            -40.0e-6
        )

        field_body = world_to_body(
            msg.orientation, *magnetic_world
        )

        # Do not synthesize mag without valid orientation
        if field_body is not None:
            mag = MagneticField()
            mag.header = msg.header

            mag.magnetic_field.x = self.noisy(
                field_body[0] + self.param('mag_bias_x'),
                mag_std
            )
            mag.magnetic_field.y = self.noisy(
                field_body[1] + self.param('mag_bias_y'),
                mag_std
            )
            mag.magnetic_field.z = self.noisy(
                field_body[2] + self.param('mag_bias_z'),
                mag_std
            )

            for i in range(3):
                mag.magnetic_field_covariance[i*3+i] = (
                    mag_std**2
                )

            self.mag_pub.publish(mag)

        self.imu_pub.publish(raw)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSensorSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

