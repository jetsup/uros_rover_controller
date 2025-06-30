import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3
from random import random


class RoverController(Node):  # publisher node
    def __init__(self):
        super().__init__("rover_controller")

        # Define a QoS profile for best effort communication
        # This matches the rclc_subscription_init_best_effort on the ESP32
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Apply the QoS profile to your publisher
        self.left_motor_publisher = self.create_publisher(
            Vector3, "uros_left_motor_control", qos_profile
        )
        self.right_motor_publisher = self.create_publisher(
            Vector3, "uros_right_motor_control", qos_profile
        )
        self.vehicle_control_publisher = self.create_publisher(
            Vector3, "uros_vehicle_control", qos_profile
        )

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("RoverController node started, publishing...")

    def timer_callback(self):
        msg_left_motor = Vector3()
        msg_left_motor.x = random()
        msg_left_motor.y = random()
        msg_left_motor.z = random()

        # Publish on the left motor control topic
        self.left_motor_publisher.publish(msg_left_motor)
        self.get_logger().info(
            f"Publishing to uros_left_motor_control: {msg_left_motor}"
        )

        msg_right_motor = Vector3()
        msg_right_motor.x = random()
        msg_right_motor.y = random()
        msg_right_motor.z = random()
        self.right_motor_publisher.publish(msg_right_motor)
        self.get_logger().info(
            f"Publishing to uros_right_motor_control: {msg_right_motor}"
        )

        msg_vehicle_control = Vector3()
        msg_vehicle_control.x = random()
        msg_vehicle_control.y = random()
        msg_vehicle_control.z = random()
        self.vehicle_control_publisher.publish(msg_vehicle_control)
        self.get_logger().info(
            f"Publishing to uros_vehicle_control: {msg_vehicle_control}"
        )


def main(args=None):
    rclpy.init(args=args)

    rover_controller = RoverController()

    try:
        rclpy.spin(rover_controller)
    except KeyboardInterrupt:
        pass
    finally:
        rover_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
