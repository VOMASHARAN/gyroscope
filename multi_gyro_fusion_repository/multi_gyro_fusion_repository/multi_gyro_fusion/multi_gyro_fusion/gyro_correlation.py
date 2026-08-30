import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import numpy as np


class GyroCorrelation(Node):

    def __init__(self):
        super().__init__('gyro_correlation')

        self.data = {
            i: {'x': [], 'y': [], 'z': []}
            for i in range(1, 9)
        }

        self.max_samples = 1000

        for i in range(1, 9):
            topic = f'/gyro/gyro{i}/data'

            self.create_subscription(
                Vector3,
                topic,
                lambda msg, gyro_id=i:
                    self.gyro_callback(msg, gyro_id),
                10
            )

            self.get_logger().info(f'Subscribed to {topic}')

        self.get_logger().info('Collecting gyro data...')

    def gyro_callback(self, msg, gyro_id):
        if len(self.data[gyro_id]['x']) < self.max_samples:
            self.data[gyro_id]['x'].append(msg.x)
            self.data[gyro_id]['y'].append(msg.y)
            self.data[gyro_id]['z'].append(msg.z)

        if all(
            len(self.data[i]['x']) >= self.max_samples
            for i in range(1, 9)
        ):
            self.calculate_correlation()
            self.get_logger().info(
                'Correlation calculation complete.'
            )
            raise SystemExit

    def calculate_correlation(self):
        print("\n" + "=" * 70)
        print("             GYROSCOPE CORRELATION ANALYSIS")
        print("=" * 70)

        print("\nSamples collected per gyro:")
        for i in range(1, 9):
            print(f"Gyro {i}: {len(self.data[i]['x'])} samples")

        for axis in ['x', 'y', 'z']:
            print("\n" + "=" * 70)
            print(f"                    {axis.upper()}-AXIS CORRELATION")
            print("=" * 70)

            axis_data = np.array([
                self.data[i][axis]
                for i in range(1, 9)
            ])

            corr = np.corrcoef(axis_data)
            self.print_matrix(corr)

        print("\n" + "=" * 70)
        print("             CORRELATION RANGE CHECK")
        print("=" * 70)

        for axis in ['x', 'y', 'z']:
            axis_data = np.array([
                self.data[i][axis]
                for i in range(1, 9)
            ])

            corr = np.corrcoef(axis_data)
            self.check_range(corr, axis.upper())

    def print_matrix(self, matrix):
        print(
            "\n             G1       G2       G3       G4       "
            "G5       G6       G7       G8"
        )

        for i in range(8):
            print(f"G{i + 1}   ", end="")
            for j in range(8):
                print(f"{matrix[i][j]:8.3f}", end=" ")
            print()

    def check_range(self, matrix, axis):
        values = []

        for i in range(8):
            for j in range(i + 1, 8):
                values.append(matrix[i][j])

        values = np.array(values)

        average = np.mean(values)
        minimum = np.min(values)
        maximum = np.max(values)

        print(f"\n{axis}-AXIS:")
        print(
            f"Average correlation : {average:.4f} "
            f"({average * 100:.2f}%)"
        )
        print(
            f"Minimum correlation : {minimum:.4f} "
            f"({minimum * 100:.2f}%)"
        )
        print(
            f"Maximum correlation : {maximum:.4f} "
            f"({maximum * 100:.2f}%)"
        )

        if average >= 0.40 and maximum <= 0.55:
            print("RESULT: PASS")
        else:
            print(
                "RESULT: DOES NOT MEET "
                "40%-55% REQUIREMENT"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GyroCorrelation()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
