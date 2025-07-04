import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3
from random import random

from pynput import keyboard


class RoverController(Node):  # publisher node
    def __init__(self):
        super().__init__("rover_controller")

        self.vehicle_speed = 0.0  # a value 0-255
        self.left_turning_speed = 0.0
        self.right_turning_speed = 0.0
        self.direction = 0  # -1 for reverse, 0 for stop, 1 for forward

        self.should_move_forward = False
        self.should_move_backward = False
        self.should_turn_left = False
        self.should_turn_right = False

        self.headlights_on = False
        self.tail_lights_on = False
        self.hoot_vehicle = False

        self.msg_left_motor = Vector3()
        self.msg_right_motor = Vector3()
        # (head_lights, tail_lights, hoot_vehicle)
        self.msg_vehicle_control = Vector3()

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
        self.vehicle_vehicle_publisher = self.create_publisher(
            Vector3, "uros_vehicle_control", qos_profile
        )

        # self.timer = self.create_timer(0.5, self.timer_callback)
        self.key_press_timer = self.create_timer(0.01, self.keyboard_timer_callback)
        self.get_logger().info("RoverController node started, publishing...")

        # Create a pynput listener
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()  # Start listening for events
        self.get_logger().info("Keyboard listener started.")

    def on_press(self, key):
        try:
            # self.get_logger().info(f'Key Pressed: {key.char}')
            # You could also publish this to a ROS2 topic
            # self.publisher_.publish(std_msgs.msg.String(data=key.char))
            if key.char == "w":
                if self.vehicle_speed < 255 and self.vehicle_speed >= 0:
                    self.vehicle_speed += 2.0
                elif self.vehicle_speed < 0:
                    self.vehicle_speed += 5.0
                self.should_move_forward = True
                # self.get_logger().info(f"Increasing speed: {self.vehicle_speed}")
            elif key.char == "s":
                if self.vehicle_speed > -255 and self.vehicle_speed <= 0:
                    self.vehicle_speed -= 2.0
                elif self.vehicle_speed > 0:
                    self.vehicle_speed -= 5.0
                self.should_move_backward = True
                # self.get_logger().info(f"Decreasing speed: {self.vehicle_speed}")
            elif key.char == "a":
                if self.left_turning_speed < 255:
                    self.left_turning_speed += 4.0
                self.should_turn_left = True
                # self.get_logger().info(
                #     f"Increasing left turning speed: {self.left_turning_speed}"
                # )
            elif key.char == "d":
                if self.right_turning_speed < 255:
                    self.right_turning_speed += 4
                self.should_turn_right = True
                # self.get_logger().info(
                #     f"Increasing right turning speed: {self.right_turning_speed}"
                # )
            elif key.char == " ":
                self.direction = 0  # Stop
                self.vehicle_speed = 0.0
                self.left_turning_speed = 0.0
                self.right_turning_speed = 0.0
                self.should_move_forward = False
                self.should_move_backward = False
                self.should_turn_left = False
                self.should_turn_right = False
                # self.get_logger().info("Stopping vehicle and resetting speeds.")
            elif key.char == "r":
                self.direction = -1  # Reverse
                self.vehicle_speed = 0.0
                self.left_turning_speed = 0.0
                self.right_turning_speed = 0.0
                self.should_move_backward = True
                # self.get_logger().info("Setting direction to reverse.")
            elif key.char == "f":
                self.direction = 1  # Forward
                self.vehicle_speed = 0.0
                self.left_turning_speed = 0.0
                self.right_turning_speed = 0.0
                self.should_move_forward = True
                # self.get_logger().info("Setting direction to forward.")
            elif key.char == "x":
                self.direction = 0  # Stop
                self.vehicle_speed = 0.0
                self.left_turning_speed = 0.0
                self.right_turning_speed = 0.0
                self.should_move_forward = False
                self.should_move_backward = False
                self.should_turn_left = False
                self.should_turn_right = False
                # self.get_logger().info("Setting direction to stop.")
            elif key.char == "h":
                self.hoot_vehicle = True
                # self.get_logger().info(f"Reversing lights {'on' if self.reversing_lights_on else 'off'}.")
            elif key.char == "l":
                self.headlights_on = not self.headlights_on
                # self.get_logger().info(f"Headlights {'on' if self.headlights_on else 'off'}.")
            elif key.char == "b":
                self.tail_lights_on = not self.tail_lights_on
                # self.get_logger().info(f"Tail lights {'on' if self.tail_lights_on else 'off'}.")
        # except AttributeError:
        except Exception as e:
            # Handle special keys (e.g., space, enter, shift)
            # self.get_logger().info(f"Special Key Pressed: {key}")
            pass  # Ignore special keys for now

    def on_release(self, key):
        if key.char == "w":
            self.should_move_forward = False
        elif key.char == "s":
            self.should_move_backward = False
        elif key.char == "a":
            self.should_turn_left = False
        elif key.char == "d":
            self.should_turn_right = False
        elif key.char == "h":
            self.hoot_vehicle = False

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

        msg_lights_control = Vector3()
        msg_lights_control.x = random()
        msg_lights_control.y = random()
        msg_lights_control.z = random()
        self.vehicle_lights_publisher.publish(msg_lights_control)
        self.get_logger().info(
            f"Publishing to uros_vehicle_control: {msg_lights_control}"
        )

    def keyboard_timer_callback(self):
        # check if the key is pressed and if not start doing the above
        if not self.should_move_forward:
            if self.vehicle_speed > 0:
                self.vehicle_speed -= 1.0
                if self.vehicle_speed < 0:
                    self.vehicle_speed = 0.0
        if not self.should_move_backward:
            if self.vehicle_speed < 0:
                self.vehicle_speed += 1.0
                if self.vehicle_speed > 0:
                    self.vehicle_speed = 0.0

        if not self.should_turn_left:
            if self.left_turning_speed > 0:
                self.left_turning_speed -= 2.0
                if self.left_turning_speed < 0:
                    self.left_turning_speed = 0.0
            elif self.left_turning_speed < 0:
                if self.right_turning_speed > 0:
                    self.right_turning_speed -= 2.0
                else:
                    self.left_turning_speed += 2.0
                if self.left_turning_speed > 0:
                    self.left_turning_speed = 0.0

        if not self.should_turn_right:
            if self.right_turning_speed > 0:
                self.right_turning_speed -= 2.0
                if self.right_turning_speed < 0:
                    self.right_turning_speed = 0.0
            elif self.right_turning_speed < 0:
                if self.left_turning_speed > 0:
                    self.left_turning_speed -= 2.0
                else:
                    self.right_turning_speed += 2.0
                if self.right_turning_speed > 0:
                    self.right_turning_speed = 0.0

        if (
            self.vehicle_speed > 255
            or self.left_turning_speed > 255
            or self.right_turning_speed > 255
        ):
            self.vehicle_speed = 255.0
            self.left_turning_speed = 255.0
            self.right_turning_speed = 255.0

        if (
            self.vehicle_speed < -255
            or self.left_turning_speed < -255
            or self.right_turning_speed < -255
        ):
            self.vehicle_speed = -255.0
            self.left_turning_speed = -255.0
            self.right_turning_speed = -255.0

        # print(
        #     f"Vehicle Speed: {self.vehicle_speed}, Left Turning Speed: {self.left_turning_speed}, Right Turning Speed: {self.right_turning_speed}, Direction: {self.direction}"
        # )
        # Print the current state of the rover
        self.get_logger().info(
            f"Forward: {self.should_move_forward}, Backward: {self.should_move_backward}, Left: {self.should_turn_left}, Right: {self.should_turn_right}, Speed: {self.vehicle_speed}, Left Turn Speed: {self.left_turning_speed}, Right Turn Speed: {self.right_turning_speed}"
        )

        # Publish the current state of the rover
        self.msg_left_motor.x = self.left_turning_speed
        self.msg_left_motor.y = self.vehicle_speed
        self.msg_left_motor.z = float(self.direction)

        self.msg_right_motor.x = self.right_turning_speed
        self.msg_right_motor.y = self.vehicle_speed
        self.msg_right_motor.z = float(self.direction)

        self.msg_vehicle_control.x = 1.0 if self.headlights_on else 0.0
        self.msg_vehicle_control.y = 1.0 if self.tail_lights_on else 0.0
        self.msg_vehicle_control.z = 1.0 if self.hoot_vehicle else 0.0

        self.left_motor_publisher.publish(self.msg_left_motor)
        self.right_motor_publisher.publish(self.msg_right_motor)
        self.vehicle_vehicle_publisher.publish(self.msg_vehicle_control)


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
