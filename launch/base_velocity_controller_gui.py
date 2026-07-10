#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from ocs2_msgs.msg import WalkingVelocityCommand
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Joystick:
    def __init__(self, parent, label, on_change):
        self._on_change = on_change
        self._canvas = tk.Canvas(parent, width=220, height=220, bg="#303030", highlightthickness=0)
        self._cx = 110.0
        self._cy = 110.0
        self._radius = 85.0
        self._x = 0.0
        self._y = 0.0
        self._canvas.create_text(110, 14, text=label, fill="white", font=("Helvetica", 11, "bold"))
        self._canvas.create_oval(
            self._cx - self._radius,
            self._cy - self._radius,
            self._cx + self._radius,
            self._cy + self._radius,
            outline="#808080",
            width=2,
        )
        self._canvas.create_line(self._cx - self._radius, self._cy, self._cx + self._radius, self._cy, fill="#555555")
        self._canvas.create_line(self._cx, self._cy - self._radius, self._cx, self._cy + self._radius, fill="#555555")
        self._knob = self._canvas.create_oval(98, 98, 122, 122, fill="#4a90e2", outline="")
        self._canvas.bind("<Button-1>", self._move)
        self._canvas.bind("<B1-Motion>", self._move)
        self._canvas.bind("<ButtonRelease-1>", self._release)

    def _move(self, event):
        dx = float(event.x) - self._cx
        dy = float(event.y) - self._cy
        norm = max((dx * dx + dy * dy) ** 0.5, 1e-9)
        scale = min(1.0, self._radius / norm)
        self._x = max(-1.0, min(1.0, dx * scale / self._radius))
        self._y = max(-1.0, min(1.0, -dy * scale / self._radius))
        self._canvas.coords(
            self._knob,
            self._cx + dx * scale - 12,
            self._cy + dy * scale - 12,
            self._cx + dx * scale + 12,
            self._cy + dy * scale + 12,
        )
        self._on_change()

    def _release(self, _event):
        self._on_change()

    def reset(self):
        self._x = 0.0
        self._y = 0.0
        self._canvas.coords(self._knob, 98, 98, 122, 122)
        self._on_change()

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y


class VelocityCommandGui(tk.Tk):
    def __init__(self, publish):
        super().__init__()
        self.title("Humanoid MPC Base Velocity Controller")
        self.configure(bg="#2c2c2c")
        self._publish = publish

        container = tk.Frame(self, bg="#2c2c2c")
        container.pack(padx=16, pady=16)

        self._linear = Joystick(container, "Linear velocity (pelvis frame)", self._changed)
        self._linear._canvas.grid(row=0, column=0, padx=10)
        self._yaw = Joystick(container, "Yaw rate", self._changed)
        self._yaw._canvas.grid(row=0, column=1, padx=10)

        height_frame = tk.Frame(container, bg="#2c2c2c")
        height_frame.grid(row=0, column=2, padx=16, sticky="ns")
        tk.Label(
            height_frame,
            text="Pelvis height [m]",
            bg="#2c2c2c",
            fg="white",
        ).pack()
        self._height = tk.Scale(
            height_frame,
            from_=1.0,
            to=0.2,
            resolution=0.001,
            orient=tk.VERTICAL,
            length=180,
            showvalue=True,
            bg="#2c2c2c",
            fg="white",
            highlightthickness=0,
        )
        self._height.set(0.7925)
        self._height.pack()

        controls = tk.Frame(self, bg="#2c2c2c")
        controls.pack(pady=(0, 16))
        tk.Button(controls, text="Center", command=self._reset, width=10).pack(side=tk.LEFT, padx=6)
        tk.Label(
            controls,
            text="v_x, v_y, yaw are normalized to the configured MPC limits",
            bg="#2c2c2c",
            fg="#cccccc",
        ).pack(side=tk.LEFT, padx=6)

        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(40, self._publish_periodically)

    def _changed(self):
        return

    def _reset(self):
        self._linear.reset()
        self._yaw.reset()
        self._height.set(0.7925)

    def _publish_periodically(self):
        # Joystick vertical axis = forward (vx); horizontal = lateral (ROS +y is left,
        # so a right-drag must command negative vy).
        self._publish(self._linear.y, -self._linear.x, self._height.get(), self._yaw.y)
        self.after(40, self._publish_periodically)

    def _close(self):
        self.destroy()


class PublisherNode(Node):
    def __init__(self):
        super().__init__("base_velocity_controller_gui")
        qos = QoSProfile(depth=25, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(
            WalkingVelocityCommand,
            "/humanoid/walking_velocity_command",
            qos,
        )

    def publish_command(self, linear_x, linear_y, height, yaw_rate):
        message = WalkingVelocityCommand()
        message.linear_velocity_x = float(max(-1.0, min(1.0, linear_x)))
        message.linear_velocity_y = float(max(-1.0, min(1.0, linear_y)))
        message.desired_pelvis_height = float(max(0.2, min(1.0, height)))
        message.angular_velocity_z = float(max(-1.0, min(1.0, yaw_rate)))
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = PublisherNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    app = VelocityCommandGui(node.publish_command)
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
