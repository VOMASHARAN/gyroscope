import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class GyroPublisher(Node):

    def __init__(self):
        super().__init__('gyro_publisher')

        self.declare_parameter('gyro_id', 1)
        self.declare_parameter('noise_stddev', 0.01)
        self.declare_parameter('bias_x', 0.0)
        self.declare_parameter('bias_y', 0.0)
        self.declare_parameter('bias_z', 0.0)
        self.declare_parameter('frequency', 100.0)

        # ARW, RRW and first-order Gauss-Markov noise parameters
        self.declare_parameter('arw', 0.01)
        self.declare_parameter('rrw', 0.001)
        self.declare_parameter('markov_correlation', 0.5)
        self.declare_parameter('markov_tau', 1.0)
        self.declare_parameter('markov_stddev', 0.01)

        self.gyro_id = self.get_parameter('gyro_id').value
        self.noise_stddev = self.get_parameter('noise_stddev').value
        self.bias_x = self.get_parameter('bias_x').value
        self.bias_y = self.get_parameter('bias_y').value
        self.bias_z = self.get_parameter('bias_z').value
        self.frequency = self.get_parameter('frequency').value

        self.arw = self.get_parameter('arw').value
        self.rrw = self.get_parameter('rrw').value
        self.markov_correlation = self.get_parameter('markov_correlation').value
        self.markov_tau = self.get_parameter('markov_tau').value
        self.markov_stddev = self.get_parameter('markov_stddev').value

        self.markov_x = 0.0
        self.markov_y = 0.0
        self.markov_z = 0.0

        topic_name = f'/gyro/gyro{self.gyro_id}/data'

        self.publisher = self.create_publisher(Vector3, topic_name, 10)

        self.timer = self.create_timer(
            1.0 / self.frequency,
            self.publish_gyro
        )

        self.time = 0.0

        self.get_logger().info(f'Gyroscope {self.gyro_id} started')
        self.get_logger().info(f'Publishing: {topic_name}')
        self.get_logger().info(f'ARW = {self.arw}')
        self.get_logger().info(f'RRW = {self.rrw}')
        self.get_logger().info(
            f'Markov correlation = {self.markov_correlation}'
        )
        self.get_logger().info(f'Markov tau = {self.markov_tau} s')
        self.get_logger().info(f'Markov stddev = {self.markov_stddev}')

    def update_markov_noise(self):
        dt = 1.0 / self.frequency
        alpha = math.exp(-dt / self.markov_tau)

        innovation_std = (
            self.markov_stddev *
            math.sqrt(max(0.0, 1.0 - alpha ** 2))
        )

        self.markov_x = (
            alpha * self.markov_x +
            random.gauss(0.0, innovation_std)
        )
        self.markov_y = (
            alpha * self.markov_y +
            random.gauss(0.0, innovation_std)
        )
        self.markov_z = (
            alpha * self.markov_z +
            random.gauss(0.0, innovation_std)
        )

    def publish_gyro(self):
        true_x = 0.5 * math.sin(self.time)
        true_y = 0.3 * math.cos(self.time)
        true_z = 0.2

        self.update_markov_noise()

        dt = 1.0 / self.frequency

        # White-noise contribution associated with ARW
        white_std = self.arw / math.sqrt(dt)

        white_x = random.gauss(0.0, white_std)
        white_y = random.gauss(0.0, white_std)
        white_z = random.gauss(0.0, white_std)

        # Slowly varying bias/random-walk contribution associated with RRW
        rrw_step_std = self.rrw * math.sqrt(dt)

        self.bias_x += random.gauss(0.0, rrw_step_std)
        self.bias_y += random.gauss(0.0, rrw_step_std)
        self.bias_z += random.gauss(0.0, rrw_step_std)

        measurement = Vector3()
        measurement.x = true_x + self.bias_x + white_x + self.markov_x
        measurement.y = true_y + self.bias_y + white_y + self.markov_y
        measurement.z = true_z + self.bias_z + white_z + self.markov_z

        self.publisher.publish(measurement)

        self.time += dt


def main(args=None):
    rclpy.init(args=args)
    node = GyroPublisher()

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
