import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3

import tkinter as tk
from tkinter import ttk


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

        # The timer for publishing will now be managed by the Tkinter loop
        self.get_logger().info("RoverController node started.")

    # --- Methods for controlling rover state from GUI ---
    def set_move_forward(self, state: bool):
        self.should_move_forward = state
        if state:
            self.direction = 1  # Ensure direction is forward when moving forward
        # print(f"Set move forward to {state}") # Debug print

    def set_move_backward(self, state: bool):
        self.should_move_backward = state
        if state:
            self.direction = -1  # Ensure direction is reverse when moving backward
        # print(f"Set move backward to {state}") # Debug print

    def set_turn_left(self, state: bool):
        self.should_turn_left = state
        # print(f"Set turn left to {state}") # Debug print

    def set_turn_right(self, state: bool):
        self.should_turn_right = state
        # print(f"Set turn right to {state}") # Debug print

    def stop_all_motion(self):
        self.direction = 0  # Stop
        self.vehicle_speed = 0.0
        self.left_turning_speed = 0.0
        self.right_turning_speed = 0.0
        self.should_move_forward = False
        self.should_move_backward = False
        self.should_turn_left = False
        self.should_turn_right = False
        self.get_logger().info("Stopping vehicle and resetting speeds.")

    def set_direction_forward(self):
        self.direction = 1
        self.get_logger().info("Setting direction to forward.")

    def set_direction_reverse(self):
        self.direction = -1
        self.get_logger().info("Setting direction to reverse.")

    def set_direction_stop(self):
        self.direction = 0
        self.get_logger().info("Setting direction to stop.")

    def toggle_headlights(self):
        self.headlights_on = not self.headlights_on
        self.get_logger().info(f"Headlights {'on' if self.headlights_on else 'off'}.")

    def toggle_tail_lights(self):
        self.tail_lights_on = not self.tail_lights_on
        self.get_logger().info(f"Tail lights {'on' if self.tail_lights_on else 'off'}.")

    def set_hoot_vehicle(self, state: bool):
        self.hoot_vehicle = state
        self.get_logger().info(f"Hoot vehicle {'on' if self.hoot_vehicle else 'off'}.")

    def update_rover_state(self):
        # Apply speed changes based on flags
        if self.should_move_forward:
            if self.vehicle_speed < 255:
                self.vehicle_speed += 5.0
            # If currently reversing, prioritize stopping or accelerating forward faster
            if self.vehicle_speed < 0:
                self.vehicle_speed += 5.0
        elif self.should_move_backward:
            if self.vehicle_speed > -255:
                self.vehicle_speed -= 5.0
            # If currently going forward, prioritize stopping or accelerating backward faster
            if self.vehicle_speed > 0:
                self.vehicle_speed -= 5.0
        else:  # No forward/backward key pressed, decelerate
            if self.vehicle_speed > 0:
                self.vehicle_speed -= 1.0
                if self.vehicle_speed < 0:
                    self.vehicle_speed = 0.0
            elif self.vehicle_speed < 0:
                self.vehicle_speed += 1.0
                if self.vehicle_speed > 0:
                    self.vehicle_speed = 0.0

        if self.should_turn_left:
            if self.right_turning_speed != 0:
                self.right_turning_speed -= 4.0
                if self.right_turning_speed < 0:
                    self.right_turning_speed = 0.0
                return  # Don't increase left speed if turning left
            if self.left_turning_speed < 255:
                self.left_turning_speed += 4.0
        else:
            if self.left_turning_speed > 0:
                self.left_turning_speed -= 2.0
                if self.left_turning_speed < 0:
                    self.left_turning_speed = 0.0

        if self.should_turn_right:
            if self.left_turning_speed != 0:
                self.left_turning_speed -= 4.0
                if self.left_turning_speed < 0:
                    self.left_turning_speed = 0.0
                return  # Don't increase right speed if turning right
            if self.right_turning_speed < 255:
                self.right_turning_speed += 4.0
        else:
            if self.right_turning_speed > 0:
                self.right_turning_speed -= 2.0
                if self.right_turning_speed < 0:
                    self.right_turning_speed = 0.0

        # Clamp speeds within valid range
        self.vehicle_speed = max(-255.0, min(self.vehicle_speed, 255.0))
        self.left_turning_speed = max(0.0, min(self.left_turning_speed, 255.0))
        self.right_turning_speed = max(0.0, min(self.right_turning_speed, 255.0))

        left_motor_final_speed = self.vehicle_speed
        right_motor_final_speed = self.vehicle_speed

        if (
            self.should_turn_left and not self.should_turn_right
        ) or self.left_turning_speed > 0:
            if self.vehicle_speed >= 0:  # Moving forward or stopped
                left_motor_final_speed -= self.left_turning_speed
            else:  # Moving backward
                left_motor_final_speed += self.left_turning_speed
        elif (
            self.should_turn_right and not self.should_turn_left
        ) or self.right_turning_speed > 0:
            if self.vehicle_speed >= 0:
                right_motor_final_speed -= self.right_turning_speed
            else:
                right_motor_final_speed += self.right_turning_speed

        print(
            f"Left F: {left_motor_final_speed:.2f} Left: {self.left_turning_speed} Right F: {right_motor_final_speed:.2f} Right: {self.right_turning_speed}"
        )

        # Clamp motor outputs again (in case speed dips below 0 or overshoots)
        left_motor_final_speed = max(-255.0, min(left_motor_final_speed, 255.0))
        right_motor_final_speed = max(-255.0, min(right_motor_final_speed, 255.0))

        self.msg_left_motor.x = left_motor_final_speed
        self.msg_left_motor.y = float(self.direction)
        self.msg_left_motor.z = 0.0  # Unused for this motor

        self.msg_right_motor.x = right_motor_final_speed
        self.msg_right_motor.y = float(self.direction)
        self.msg_right_motor.z = 0.0  # Unused for this motor

        self.msg_vehicle_control.x = 1.0 if self.headlights_on else 0.0
        self.msg_vehicle_control.y = 1.0 if self.tail_lights_on else 0.0
        self.msg_vehicle_control.z = 1.0 if self.hoot_vehicle else 0.0

        self.left_motor_publisher.publish(self.msg_left_motor)
        self.right_motor_publisher.publish(self.msg_right_motor)
        self.vehicle_vehicle_publisher.publish(self.msg_vehicle_control)

        # self.get_logger().info(
        #     f"Speed: {self.vehicle_speed:.2f}, Left Turn: {self.left_turning_speed:.2f}, Right Turn: {self.right_turning_speed:.2f}, Dir: {self.direction}, HL: {self.headlights_on}, TL: {self.tail_lights_on}, Hoot: {self.hoot_vehicle}"
        # )


class RoverGUI(tk.Tk):
    def __init__(self, rover_controller_node: RoverController):
        super().__init__()
        self.title("Rover Controller")
        self.geometry("450x640")

        self.rover_controller = rover_controller_node
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.create_widgets()
        self.bind_keys()
        self.update_gui_and_rover_state()  # Start the recurring update

    def create_widgets(self):
        # Styling
        self.style = ttk.Style()
        self.style.configure("TFrame", padding=10, relief="groove")
        self.style.configure("TButton", font=("Helvetica", 12))
        self.style.configure("TLabel", font=("Helvetica", 10))

        # Main Frame
        main_frame = ttk.Frame(self, padding="10 10 10 10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Speed Control
        speed_frame = ttk.LabelFrame(
            main_frame, text="Speed Control", padding="10 10 10 10"
        )
        speed_frame.grid(row=0, column=0, columnspan=2, pady=10, padx=10, sticky="ew")

        ttk.Button(
            speed_frame,
            text="W: Forward",
            command=lambda: self.rover_controller.set_move_forward(True),
        ).grid(row=0, column=1, pady=5)
        ttk.Button(
            speed_frame,
            text="A: Turn Left",
            command=lambda: self.rover_controller.set_turn_left(True),
        ).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(
            speed_frame,
            text="S: Backward",
            command=lambda: self.rover_controller.set_move_backward(True),
        ).grid(row=1, column=1, pady=5)
        ttk.Button(
            speed_frame,
            text="D: Turn Right",
            command=lambda: self.rover_controller.set_turn_right(True),
        ).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(
            speed_frame,
            text="Space: STOP",
            command=self.rover_controller.stop_all_motion,
        ).grid(row=2, column=1, pady=10)

        # Direction Control
        direction_frame = ttk.LabelFrame(
            main_frame, text="Direction Lock", padding="10 10 10 10"
        )
        direction_frame.grid(
            row=1, column=0, columnspan=2, pady=10, padx=10, sticky="ew"
        )
        ttk.Button(
            direction_frame,
            text="F: Forward Lock",
            command=self.rover_controller.set_direction_forward,
        ).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(
            direction_frame,
            text="R: Reverse Lock",
            command=self.rover_controller.set_direction_reverse,
        ).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(
            direction_frame,
            text="X: Stop Lock",
            command=self.rover_controller.set_direction_stop,
        ).grid(row=0, column=2, padx=5, pady=5)

        # Lights and Hoot Control
        lights_frame = ttk.LabelFrame(
            main_frame, text="Vehicle Controls", padding="10 10 10 10"
        )
        lights_frame.grid(row=2, column=0, columnspan=2, pady=10, padx=10, sticky="ew")

        self.headlight_button = ttk.Button(
            lights_frame,
            text="L: Toggle Headlights",
            command=self.rover_controller.toggle_headlights,
        )
        self.headlight_button.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        self.taillight_button = ttk.Button(
            lights_frame,
            text="B: Toggle Tail Lights",
            command=self.rover_controller.toggle_tail_lights,
        )
        self.taillight_button.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        self.hoot_button = ttk.Button(
            lights_frame,
            text="H: Hoot",
            command=lambda: self.rover_controller.set_hoot_vehicle(
                not self.rover_controller.hoot_vehicle
            ),
        )
        self.hoot_button.grid(
            row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        # Status Display
        status_frame = ttk.LabelFrame(
            main_frame, text="Rover Status", padding="10 10 10 10"
        )
        status_frame.grid(row=3, column=0, columnspan=2, pady=10, padx=10, sticky="ew")

        self.speed_label = ttk.Label(status_frame, text="Speed: 0.0")
        self.speed_label.grid(row=0, column=0, sticky="w", pady=2)
        self.left_turn_label = ttk.Label(status_frame, text="Left Turn Speed: 0.0")
        self.left_turn_label.grid(row=1, column=0, sticky="w", pady=2)
        self.right_turn_label = ttk.Label(status_frame, text="Right Turn Speed: 0.0")
        self.right_turn_label.grid(row=2, column=0, sticky="w", pady=2)
        self.direction_label = ttk.Label(status_frame, text="Direction: Stop")
        self.direction_label.grid(row=3, column=0, sticky="w", pady=2)
        self.headlights_label = ttk.Label(status_frame, text="Headlights: Off")
        self.headlights_label.grid(row=4, column=0, sticky="w", pady=2)
        self.tail_lights_label = ttk.Label(status_frame, text="Tail Lights: Off")
        self.tail_lights_label.grid(row=5, column=0, sticky="w", pady=2)
        self.hoot_label = ttk.Label(status_frame, text="Hoot: Off")
        self.hoot_label.grid(row=6, column=0, sticky="w", pady=2)

        # Configure columns to expand evenly
        for i in range(3):
            speed_frame.grid_columnconfigure(i, weight=1)
        for i in range(3):
            direction_frame.grid_columnconfigure(i, weight=1)
        for i in range(2):
            lights_frame.grid_columnconfigure(i, weight=1)
        status_frame.grid_columnconfigure(0, weight=1)

    def bind_keys(self):
        self.bind(
            "<KeyPress-w>", lambda event: self.rover_controller.set_move_forward(True)
        )
        self.bind(
            "<KeyRelease-w>",
            lambda event: self.rover_controller.set_move_forward(False),
        )

        self.bind(
            "<KeyPress-s>", lambda event: self.rover_controller.set_move_backward(True)
        )
        self.bind(
            "<KeyRelease-s>",
            lambda event: self.rover_controller.set_move_backward(False),
        )

        self.bind(
            "<KeyPress-a>", lambda event: self.rover_controller.set_turn_left(True)
        )
        self.bind(
            "<KeyRelease-a>", lambda event: self.rover_controller.set_turn_left(False)
        )

        self.bind(
            "<KeyPress-d>", lambda event: self.rover_controller.set_turn_right(True)
        )
        self.bind(
            "<KeyRelease-d>", lambda event: self.rover_controller.set_turn_right(False)
        )

        # Arrow keys
        self.bind(
            "<KeyPress-Up>", lambda event: self.rover_controller.set_move_forward(True)
        )
        self.bind(
            "<KeyRelease-Up>", lambda event: self.rover_controller.set_move_forward(False)
        )

        self.bind(
            "<KeyPress-Down>", lambda event: self.rover_controller.set_move_backward(True)
        )
        self.bind(
            "<KeyRelease-Down>", lambda event: self.rover_controller.set_move_backward(False)
        )

        self.bind(
            "<KeyPress-Left>", lambda event: self.rover_controller.set_turn_left(True)
        )
        self.bind(
            "<KeyRelease-Left>", lambda event: self.rover_controller.set_turn_left(False)
        )

        self.bind(
            "<KeyPress-Right>", lambda event: self.rover_controller.set_turn_right(True)
        )
        self.bind(
            "<KeyRelease-Right>", lambda event: self.rover_controller.set_turn_right(False)
        )

        self.bind(
            "<KeyPress-space>", lambda event: self.rover_controller.stop_all_motion()
        )
        # No KeyRelease for space as it's a one-shot stop

        self.bind(
            "<KeyPress-r>", lambda event: self.rover_controller.set_direction_reverse()
        )
        self.bind(
            "<KeyPress-f>", lambda event: self.rover_controller.set_direction_forward()
        )
        self.bind(
            "<KeyPress-x>", lambda event: self.rover_controller.set_direction_stop()
        )

        self.bind(
            "<KeyPress-h>", lambda event: self.rover_controller.set_hoot_vehicle(True)
        )
        self.bind(
            "<KeyRelease-h>",
            lambda event: self.rover_controller.set_hoot_vehicle(False),
        )

        self.bind(
            "<KeyPress-l>", lambda event: self.rover_controller.toggle_headlights()
        )
        self.bind(
            "<KeyPress-b>", lambda event: self.rover_controller.toggle_tail_lights()
        )

    def update_gui_and_rover_state(self):
        # Process ROS2 events
        rclpy.spin_once(self.rover_controller, timeout_sec=0.01)

        self.rover_controller.update_rover_state()

        # Update GUI labels
        self.speed_label.config(
            text=f"Speed: {self.rover_controller.vehicle_speed:.2f}"
        )
        self.left_turn_label.config(
            text=f"Left Turn Speed: {self.rover_controller.left_turning_speed:.2f}"
        )
        self.right_turn_label.config(
            text=f"Right Turn Speed: {self.rover_controller.right_turning_speed:.2f}"
        )

        direction_text = "Stop"
        if self.rover_controller.direction == 1:
            direction_text = "Forward"
        elif self.rover_controller.direction == -1:
            direction_text = "Reverse"
        self.direction_label.config(text=f"Direction: {direction_text}")

        self.headlights_label.config(
            text=f"Headlights: {'On' if self.rover_controller.headlights_on else 'Off'}"
        )
        self.tail_lights_label.config(
            text=f"Tail Lights: {'On' if self.rover_controller.tail_lights_on else 'Off'}"
        )
        self.hoot_label.config(
            text=f"Hoot: {'On' if self.rover_controller.hoot_vehicle else 'Off'}"
        )

        # Schedule the next update
        self.after(10, self.update_gui_and_rover_state)  # Update every 10 ms (100 Hz)

    def on_closing(self):
        self.rover_controller.get_logger().info(
            "Shutting down Rover GUI and ROS2 node."
        )
        self.rover_controller.destroy_node()
        rclpy.shutdown()
        self.destroy()


def main(args=None):
    try:
        rclpy.init(args=args)

        rover_controller = RoverController()

        app = RoverGUI(rover_controller)
        app.mainloop()
    except KeyboardInterrupt:
        print("Shutting down Rover Controller GUI.")
    except Exception as e:
        print(f"Error in Rover Controller GUI: {e}")
    finally:
        try:
            rover_controller.destroy_node() # type: ignore
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")


if __name__ == "__main__":
    main()
