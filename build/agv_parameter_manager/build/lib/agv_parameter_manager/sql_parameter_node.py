
#!/usr/bin/env python3
import os
import math
import pyodbc

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically


class SqlParameterNode(Node):
    def __init__(self):
        super().__init__('sql_parameter_node')

        self.declare_parameter(
            'target_node', '/wheel_odometry_node'
        )
        self.declare_parameter('poll_seconds', 1.0)

        self.target = self.get_parameter(
            'target_node').value

        self.client = self.create_client(
            SetParametersAtomically,
            self.target + '/set_parameters_atomically'
        )

        self.mapping = {
            'odom_vx_std': 'odom_vx_std',
            'odom_vy_std': 'odom_vy_std',
            'odom_wz_std': 'odom_wz_std',
        }

        self.last_applied = {}
        self.pending = False

        self.conn_str = (
            'DRIVER={ODBC Driver 18 for SQL Server};'
            f"SERVER={os.getenv('AGV_SQL_HOST', '172.31.32.1')},1433;"
            f"DATABASE={os.getenv('AGV_SQL_DB', 'AGV_Config')};"
            f"UID={os.getenv('AGV_SQL_USER', 'agv_runtime')};"
            f"PWD={os.environ['AGV_SQL_PASSWORD']};"
            'Encrypt=yes;'
            'TrustServerCertificate=yes;'
        )

        self.timer = self.create_timer(
            float(self.get_parameter('poll_seconds').value),
            self.check_sql
        )

        self.get_logger().info(
            f'SQL parameter manager -> {self.target}'
        )

    def read_sql(self):
        with pyodbc.connect(
            self.conn_str, timeout=3
        ) as conn:
            cur = conn.cursor()
            cur.execute("""
                SELECT ParameterName, ParameterValue
                FROM dbo.EKF_Parameters
            """)
            return {
                str(name): float(value)
                for name, value in cur.fetchall()
            }

    def check_sql(self):
        if self.pending:
            return

        if not self.client.service_is_ready():
            self.get_logger().warning(
                f'Waiting for {self.target} parameter service',
                throttle_duration_sec=5.0
            )
            return

        try:
            values = self.read_sql()
        except Exception as exc:
            self.get_logger().error(
                f'SQL read failed: {exc}',
                throttle_duration_sec=5.0
            )
            return

        changes = {}
        for sql_name, ros_name in self.mapping.items():
            if sql_name not in values:
                continue

            value = values[sql_name]
            if not math.isfinite(value) or not 0 < value <= 100:
                self.get_logger().error(
                    f'Invalid {sql_name}={value}'
                )
                continue

            if self.last_applied.get(ros_name) != value:
                changes[ros_name] = value

        if not changes:
            return

        request = SetParametersAtomically.Request()

        request.parameters = [
            Parameter(
                name=name,
                value=value
            ).to_parameter_msg()
            for name, value in changes.items()
        ]

        self.pending = True
        future = self.client.call_async(request)

        def on_done(result_future):
            self.pending = False
            try:
                result = result_future.result().result

                if result.successful:
                    self.last_applied.update(changes)
                    self.get_logger().info(
                        f'SQL applied: {changes}'
                    )
                else:
                    self.get_logger().error(
                        f'Parameter rejected: {result.reason}'
                    )
            except Exception as exc:
                self.get_logger().error(
                    f'Parameter update failed: {exc}'
                )

        future.add_done_callback(on_done)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SqlParameterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
