#!/usr/bin/env python3

import math
import threading
import tkinter as tk

import rclpy
from ocs2_msgs.msg import WalkingVelocityCommand
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


KNOB_IDLE_COLOR = "#4a90e2"
KNOB_ACTIVE_COLOR = "#4a90e2"
KNOB_IDLE_HALF_SIZE = 12.0
KNOB_ACTIVE_HALF_SIZE = 14.0
TITLE_FONT = ("Helvetica", 11, "bold")
VALUE_FONT = ("Helvetica", 11)


class Joystick:
    def __init__(self, parent, label, on_change):
        self._on_change = on_change
        self._canvas = tk.Canvas(parent, width=220, height=240, bg="#303030", highlightthickness=0)
        self._cx = 110.0
        self._cy = 137.0
        self._radius = 85.0
        self._x = 0.0
        self._y = 0.0
        self._dragging = False
        self._hovering = False
        self._canvas.create_text(110, 14, text=label, fill="white", font=TITLE_FONT)
        self._readout = self._canvas.create_text(
            110,
            35,
            text="(vₓ, vᵧ) = (0.00, 0.00)",
            fill="white",
            font=VALUE_FONT,
        )
        self._canvas.create_oval(
            self._cx - self._radius,
            self._cy - self._radius,
            self._cx + self._radius,
            self._cy + self._radius,
            outline="#808080",
            width=2,
        )
        self._canvas.create_line(
            self._cx - self._radius,
            self._cy,
            self._cx + self._radius,
            self._cy,
            fill="#555555",
        )
        self._canvas.create_line(
            self._cx,
            self._cy + self._radius,
            self._cx,
            self._cy - self._radius,
            fill="#555555",
        )
        self._canvas.create_line(
            self._cx,
            self._cy - self._radius + 18,
            self._cx,
            self._cy - self._radius,
            fill="white",
            width=2,
            arrow=tk.LAST,
        )
        self._canvas.create_line(
            self._cx + self._radius - 18,
            self._cy,
            self._cx + self._radius,
            self._cy,
            fill="white",
            width=2,
            arrow=tk.LAST,
        )
        self._knob = self._canvas.create_oval(
            self._cx - KNOB_IDLE_HALF_SIZE,
            self._cy - KNOB_IDLE_HALF_SIZE,
            self._cx + KNOB_IDLE_HALF_SIZE,
            self._cy + KNOB_IDLE_HALF_SIZE,
            fill=KNOB_IDLE_COLOR,
            outline="",
        )
        self._canvas.tag_bind(self._knob, "<Enter>", self._hover_enter)
        self._canvas.tag_bind(self._knob, "<Leave>", self._hover_leave)
        self._canvas.bind("<Button-1>", self._press)
        self._canvas.bind("<B1-Motion>", self._drag)
        self._canvas.bind("<ButtonRelease-1>", self._release)

    def _hover_enter(self, _event):
        self._hovering = True
        self._update_knob_visual()

    def _hover_leave(self, _event):
        self._hovering = False
        self._update_knob_visual()

    def _update_knob_visual(self):
        active = self._dragging or self._hovering
        half_size = KNOB_ACTIVE_HALF_SIZE if active else KNOB_IDLE_HALF_SIZE
        knob_x = self._cx + self._x * self._radius
        knob_y = self._cy - self._y * self._radius
        self._canvas.coords(
            self._knob,
            knob_x - half_size,
            knob_y - half_size,
            knob_x + half_size,
            knob_y + half_size,
        )
        self._canvas.itemconfigure(
            self._knob,
            fill=KNOB_ACTIVE_COLOR if active else KNOB_IDLE_COLOR,
        )

    def _press(self, event):
        self._dragging = True
        self._update_knob_visual()
        self._move(event)

    def _drag(self, event):
        if self._dragging:
            self._move(event)

    def _move(self, event):
        dx = float(event.x) - self._cx
        dy = float(event.y) - self._cy
        norm = max((dx * dx + dy * dy) ** 0.5, 1e-9)
        scale = min(1.0, self._radius / norm)
        self._x = max(-1.0, min(1.0, dx * scale / self._radius))
        self._y = max(-1.0, min(1.0, -dy * scale / self._radius))
        self._update_knob_visual()
        self._on_change()

    def _release(self, _event):
        self._dragging = False
        self._update_knob_visual()
        self._on_change()

    def reset(self):
        self._x = 0.0
        self._y = 0.0
        self._dragging = False
        self._update_knob_visual()
        self._on_change()

    def set_readout(self, velocity_x, velocity_y):
        self._canvas.itemconfigure(
            self._readout,
            text=f"(vₓ, vᵧ) = ({velocity_x:+.2f}, {velocity_y:+.2f})",
        )

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y


class YawRateBar:
    def __init__(self, parent, on_change):
        self._on_change = on_change
        self._canvas = tk.Canvas(parent, width=220, height=240, bg="#303030", highlightthickness=0)
        self._cx = 110.0
        self._cy = 137.0
        self._half_length = 85.0
        self._value = 0.0
        self._knob_x = self._cx
        self._dragging = False
        self._hovering = False

        self._canvas.create_text(110, 14, text="Yaw rate [deg/s]", fill="white", font=TITLE_FONT)
        self._readout = self._canvas.create_text(
            110,
            35,
            text="ω_z = 0.0",
            fill="white",
            font=VALUE_FONT,
        )
        self._canvas.create_line(
            self._cx - self._half_length,
            self._cy,
            self._cx + self._half_length,
            self._cy,
            fill="#808080",
            width=4,
        )
        self._canvas.create_line(self._cx, self._cy - 10, self._cx, self._cy + 10, fill="#555555", width=2)
        self._canvas.create_text(self._cx - self._half_length, self._cy + 26, text="←", fill="white")
        self._canvas.create_text(self._cx + self._half_length, self._cy + 26, text="→", fill="white")
        self._knob = self._canvas.create_oval(
            self._cx - KNOB_IDLE_HALF_SIZE,
            self._cy - KNOB_IDLE_HALF_SIZE,
            self._cx + KNOB_IDLE_HALF_SIZE,
            self._cy + KNOB_IDLE_HALF_SIZE,
            fill=KNOB_IDLE_COLOR,
            outline="",
        )

        self._canvas.tag_bind(self._knob, "<Enter>", self._hover_enter)
        self._canvas.tag_bind(self._knob, "<Leave>", self._hover_leave)
        self._canvas.bind("<Button-1>", self._press)
        self._canvas.bind("<B1-Motion>", self._drag)
        self._canvas.bind("<ButtonRelease-1>", self._release)

    def _hover_enter(self, _event):
        self._hovering = True
        self._update_knob_visual()

    def _hover_leave(self, _event):
        self._hovering = False
        self._update_knob_visual()

    def _update_knob_visual(self):
        active = self._dragging or self._hovering
        half_size = KNOB_ACTIVE_HALF_SIZE if active else KNOB_IDLE_HALF_SIZE
        self._canvas.coords(
            self._knob,
            self._knob_x - half_size,
            self._cy - half_size,
            self._knob_x + half_size,
            self._cy + half_size,
        )
        self._canvas.itemconfigure(
            self._knob,
            fill=KNOB_ACTIVE_COLOR if active else KNOB_IDLE_COLOR,
        )

    def _press(self, event):
        event_x = float(event.x)
        event_y = float(event.y)
        knob_hit = (event_x - self._knob_x) ** 2 + (event_y - self._cy) ** 2 <= 16.0 ** 2
        bar_hit = (
            self._cx - self._half_length <= event_x <= self._cx + self._half_length
            and abs(event_y - self._cy) <= 8.0
        )
        if not (knob_hit or bar_hit):
            return

        self._dragging = True
        self._update_knob_visual()
        self._move(event)

    def _drag(self, event):
        if self._dragging:
            self._move(event)

    def _move(self, event):
        knob_x = max(self._cx - self._half_length, min(self._cx + self._half_length, float(event.x)))
        self._knob_x = knob_x
        self._value = -(knob_x - self._cx) / self._half_length
        self._update_knob_visual()
        self._on_change()

    def _release(self, _event):
        if not self._dragging:
            return
        self._dragging = False
        self._update_knob_visual()
        self._on_change()

    def reset(self):
        self._value = 0.0
        self._knob_x = self._cx
        self._dragging = False
        self._update_knob_visual()
        self._on_change()

    def set_readout(self, yaw_rate_degrees):
        self._canvas.itemconfigure(self._readout, text=f"ω_z = {yaw_rate_degrees:+.1f}")

    @property
    def value(self):
        return self._value


class HeightBar:
    def __init__(self, parent, on_change):
        self._on_change = on_change
        self._canvas = tk.Canvas(parent, width=48, height=150, bg="#2c2c2c", highlightthickness=0)
        self._x = 24.0
        self._top = 10.0
        self._bottom = 140.0
        self._minimum = 0.2
        self._maximum = 1.0
        self._center = 0.7925
        self._value = self._center
        self._dragging = False
        self._hovering = False

        self._canvas.create_line(
            self._x,
            self._top,
            self._x,
            self._bottom,
            fill="#808080",
            width=4,
        )
        center_y = self._value_to_y(self._center)
        self._canvas.create_line(
            self._x - 10,
            center_y,
            self._x + 10,
            center_y,
            fill="#555555",
            width=2,
        )
        knob_y = self._value_to_y(self._value)
        self._knob = self._canvas.create_rectangle(
            self._x - 10,
            knob_y - 8,
            self._x + 10,
            knob_y + 8,
            fill=KNOB_IDLE_COLOR,
            outline="",
        )

        self._canvas.tag_bind(self._knob, "<Enter>", self._hover_enter)
        self._canvas.tag_bind(self._knob, "<Leave>", self._hover_leave)
        self._canvas.bind("<Button-1>", self._press)
        self._canvas.bind("<B1-Motion>", self._drag)
        self._canvas.bind("<ButtonRelease-1>", self._release)

    def _value_to_y(self, value):
        ratio = (self._maximum - value) / (self._maximum - self._minimum)
        return self._top + ratio * (self._bottom - self._top)

    def _y_to_value(self, y):
        ratio = (y - self._top) / (self._bottom - self._top)
        return self._maximum - ratio * (self._maximum - self._minimum)

    def _hover_enter(self, _event):
        self._hovering = True
        self._update_knob_visual()

    def _hover_leave(self, _event):
        self._hovering = False
        self._update_knob_visual()

    def _update_knob_visual(self):
        active = self._dragging or self._hovering
        half_width = 12.0 if active else 10.0
        half_height = 10.0 if active else 8.0
        knob_y = self._value_to_y(self._value)
        self._canvas.coords(
            self._knob,
            self._x - half_width,
            knob_y - half_height,
            self._x + half_width,
            knob_y + half_height,
        )
        self._canvas.itemconfigure(
            self._knob,
            fill=KNOB_ACTIVE_COLOR if active else KNOB_IDLE_COLOR,
        )

    def _press(self, event):
        event_x = float(event.x)
        event_y = float(event.y)
        knob_y = self._value_to_y(self._value)
        knob_hit = abs(event_x - self._x) <= 14.0 and abs(event_y - knob_y) <= 12.0
        bar_hit = abs(event_x - self._x) <= 8.0 and self._top <= event_y <= self._bottom
        if not (knob_hit or bar_hit):
            return

        self._dragging = True
        self._update_knob_visual()
        self._move(event)

    def _drag(self, event):
        if self._dragging:
            self._move(event)

    def _move(self, event):
        knob_y = max(self._top, min(self._bottom, float(event.y)))
        self._value = round(self._y_to_value(knob_y), 3)
        self._update_knob_visual()
        self._on_change(self._value)

    def _release(self, _event):
        if not self._dragging:
            return
        self._dragging = False
        self._update_knob_visual()

    def set(self, value):
        self._value = max(self._minimum, min(self._maximum, float(value)))
        self._update_knob_visual()
        self._on_change(self._value)

    def get(self):
        return self._value


class baseCommandGui(tk.Tk):
    def __init__(
        self,
        publish,
        max_linear_velocity_x,
        max_linear_velocity_y,
        max_yaw_rate,
    ):
        super().__init__()
        self.title("Humanoid MPC Base Command GUI (Pelvis Frame)")
        self.configure(bg="#2c2c2c")
        self._publish = publish
        self._max_linear_velocity_x = max_linear_velocity_x
        self._max_linear_velocity_y = max_linear_velocity_y
        self._max_yaw_rate = max_yaw_rate

        container = tk.Frame(self, bg="#2c2c2c")
        container.pack(padx=16, pady=16)

        self._linear = Joystick(container, "Linear velocity [m/s]", self._changed)
        self._linear._canvas.grid(row=0, column=0, padx=10)
        self._yaw = YawRateBar(container, self._changed)
        self._yaw._canvas.grid(row=0, column=1, padx=10)

        height_frame = tk.Frame(container, bg="#2c2c2c")
        height_frame.grid(row=0, column=2, padx=16, sticky="ns")
        height_frame.columnconfigure(0, weight=1)
        tk.Label(
            height_frame,
            text="Height [m]",
            bg="#2c2c2c",
            fg="white",
            font=TITLE_FONT,
        ).grid(row=0, column=0)
        self._height_value = tk.StringVar(value="z = 0.792")
        tk.Label(
            height_frame,
            textvariable=self._height_value,
            bg="#2c2c2c",
            fg="white",
            font=VALUE_FONT,
        ).grid(row=1, column=0, pady=(8, 4))
        tk.Label(
            height_frame,
            text="↑",
            bg="#2c2c2c",
            fg="white",
            font=("Helvetica", 12, "bold"),
        ).grid(row=2, column=0)
        self._height = HeightBar(height_frame, self._height_changed)
        self._height.set(0.7925)
        self._height._canvas.grid(row=3, column=0)
        tk.Label(
            height_frame,
            text="↓",
            bg="#2c2c2c",
            fg="white",
            font=("Helvetica", 12, "bold"),
        ).grid(row=4, column=0)

        controls = tk.Frame(self, bg="#2c2c2c")
        controls.pack(pady=(0, 16))
        tk.Button(controls, text="Center", command=self._reset, width=10).pack(side=tk.LEFT, padx=6)

        self.protocol("WM_DELETE_WINDOW", self._close)
        self._changed()
        self.after(20, self._publish_periodically)

    def _changed(self):
        velocity_x = self._linear.y * self._max_linear_velocity_x
        velocity_y = -self._linear.x * self._max_linear_velocity_y
        yaw_rate_degrees = math.degrees(self._yaw.value * self._max_yaw_rate)
        self._linear.set_readout(velocity_x, velocity_y)
        self._yaw.set_readout(yaw_rate_degrees)

    def _height_changed(self, value):
        self._height_value.set(f"z = {float(value):.3f}")

    def _reset(self):
        self._linear.reset()
        self._yaw.reset()
        self._height.set(0.7925)

    def _publish_periodically(self):
        # Joystick vertical axis = forward (vx); horizontal = lateral (ROS +y is left,
        # so a right-drag must command negative vy). The yaw bar is +1 on the
        # left and -1 on the right, making a right-drag command a right turn.
        self._publish(self._linear.y, -self._linear.x, self._height.get(), self._yaw.value)
        self.after(20, self._publish_periodically)

    def _close(self):
        self.destroy()


class PublisherNode(Node):
    def __init__(self):
        super().__init__("base_command_gui")
        self.declare_parameter("max_linear_velocity_x", 2.4)
        self.declare_parameter("max_linear_velocity_y", 1.2)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.max_linear_velocity_x = float(self.get_parameter("max_linear_velocity_x").value)
        self.max_linear_velocity_y = float(self.get_parameter("max_linear_velocity_y").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
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
    app = baseCommandGui(
        node.publish_command,
        node.max_linear_velocity_x,
        node.max_linear_velocity_y,
        node.max_yaw_rate,
    )
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
