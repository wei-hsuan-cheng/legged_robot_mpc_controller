#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from ocs2_msgs.msg import WalkingVelocityCommand
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


KNOB_IDLE_COLOR = "#4a90e2"
KNOB_ACTIVE_COLOR = "#ffffff"


class Joystick:
    def __init__(self, parent, label, on_change):
        self._on_change = on_change
        self._canvas = tk.Canvas(parent, width=220, height=240, bg="#303030", highlightthickness=0)
        self._cx = 110.0
        self._cy = 125.0
        self._radius = 85.0
        self._x = 0.0
        self._y = 0.0
        self._dragging = False
        self._canvas.create_text(110, 14, text=label, fill="white", font=("Helvetica", 11, "bold"))
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
            self._cx - 12,
            self._cy - 12,
            self._cx + 12,
            self._cy + 12,
            fill=KNOB_IDLE_COLOR,
            activefill=KNOB_ACTIVE_COLOR,
            outline="",
        )
        self._canvas.bind("<Button-1>", self._press)
        self._canvas.bind("<B1-Motion>", self._drag)
        self._canvas.bind("<ButtonRelease-1>", self._release)

    def _press(self, event):
        self._dragging = True
        self._canvas.itemconfigure(self._knob, fill=KNOB_ACTIVE_COLOR)
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
        self._canvas.coords(
            self._knob,
            self._cx + dx * scale - 12,
            self._cy + dy * scale - 12,
            self._cx + dx * scale + 12,
            self._cy + dy * scale + 12,
        )
        self._on_change()

    def _release(self, _event):
        self._dragging = False
        self._canvas.itemconfigure(self._knob, fill=KNOB_IDLE_COLOR)
        self._on_change()

    def reset(self):
        self._x = 0.0
        self._y = 0.0
        self._dragging = False
        self._canvas.coords(self._knob, self._cx - 12, self._cy - 12, self._cx + 12, self._cy + 12)
        self._canvas.itemconfigure(self._knob, fill=KNOB_IDLE_COLOR)
        self._on_change()

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
        self._cy = 125.0
        self._half_length = 85.0
        self._value = 0.0
        self._knob_x = self._cx
        self._dragging = False

        self._canvas.create_text(110, 14, text="Yaw rate", fill="white", font=("Helvetica", 11, "bold"))
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
            self._cx - 12,
            self._cy - 12,
            self._cx + 12,
            self._cy + 12,
            fill=KNOB_IDLE_COLOR,
            activefill=KNOB_ACTIVE_COLOR,
            outline="",
        )

        self._canvas.bind("<Button-1>", self._press)
        self._canvas.bind("<B1-Motion>", self._drag)
        self._canvas.bind("<ButtonRelease-1>", self._release)

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
        self._canvas.itemconfigure(self._knob, fill=KNOB_ACTIVE_COLOR)
        self._move(event)

    def _drag(self, event):
        if self._dragging:
            self._move(event)

    def _move(self, event):
        knob_x = max(self._cx - self._half_length, min(self._cx + self._half_length, float(event.x)))
        self._knob_x = knob_x
        self._value = -(knob_x - self._cx) / self._half_length
        self._canvas.coords(self._knob, knob_x - 12, self._cy - 12, knob_x + 12, self._cy + 12)
        self._on_change()

    def _release(self, _event):
        if not self._dragging:
            return
        self._dragging = False
        self._canvas.itemconfigure(self._knob, fill=KNOB_IDLE_COLOR)
        self._on_change()

    def reset(self):
        self._value = 0.0
        self._knob_x = self._cx
        self._dragging = False
        self._canvas.coords(self._knob, self._cx - 12, self._cy - 12, self._cx + 12, self._cy + 12)
        self._canvas.itemconfigure(self._knob, fill=KNOB_IDLE_COLOR)
        self._on_change()

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
            activefill=KNOB_ACTIVE_COLOR,
            outline="",
        )

        self._canvas.bind("<Button-1>", self._press)
        self._canvas.bind("<B1-Motion>", self._drag)
        self._canvas.bind("<ButtonRelease-1>", self._release)

    def _value_to_y(self, value):
        ratio = (self._maximum - value) / (self._maximum - self._minimum)
        return self._top + ratio * (self._bottom - self._top)

    def _y_to_value(self, y):
        ratio = (y - self._top) / (self._bottom - self._top)
        return self._maximum - ratio * (self._maximum - self._minimum)

    def _press(self, event):
        event_x = float(event.x)
        event_y = float(event.y)
        knob_y = self._value_to_y(self._value)
        knob_hit = abs(event_x - self._x) <= 14.0 and abs(event_y - knob_y) <= 12.0
        bar_hit = abs(event_x - self._x) <= 8.0 and self._top <= event_y <= self._bottom
        if not (knob_hit or bar_hit):
            return

        self._dragging = True
        self._canvas.itemconfigure(self._knob, fill=KNOB_ACTIVE_COLOR)
        self._move(event)

    def _drag(self, event):
        if self._dragging:
            self._move(event)

    def _move(self, event):
        knob_y = max(self._top, min(self._bottom, float(event.y)))
        self._value = round(self._y_to_value(knob_y), 3)
        knob_y = self._value_to_y(self._value)
        self._canvas.coords(
            self._knob,
            self._x - 10,
            knob_y - 8,
            self._x + 10,
            knob_y + 8,
        )
        self._on_change(self._value)

    def _release(self, _event):
        if not self._dragging:
            return
        self._dragging = False
        self._canvas.itemconfigure(self._knob, fill=KNOB_IDLE_COLOR)

    def set(self, value):
        self._value = max(self._minimum, min(self._maximum, float(value)))
        knob_y = self._value_to_y(self._value)
        self._canvas.coords(
            self._knob,
            self._x - 10,
            knob_y - 8,
            self._x + 10,
            knob_y + 8,
        )
        self._canvas.itemconfigure(self._knob, fill=KNOB_IDLE_COLOR)
        self._on_change(self._value)

    def get(self):
        return self._value


class VelocityCommandGui(tk.Tk):
    def __init__(self, publish):
        super().__init__()
        self.title("Humanoid MPC Base Twist Controller (Pelvis Frame)")
        self.configure(bg="#2c2c2c")
        self._publish = publish

        container = tk.Frame(self, bg="#2c2c2c")
        container.pack(padx=16, pady=16)

        self._linear = Joystick(container, "Linear velocity", self._changed)
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
        ).grid(row=0, column=0)
        self._height_value = tk.StringVar(value="0.793")
        tk.Label(
            height_frame,
            textvariable=self._height_value,
            bg="#2c2c2c",
            fg="white",
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
        status_text = tk.Text(
            controls,
            width=55,
            height=1,
            wrap=tk.NONE,
            bg="#2c2c2c",
            fg="#cccccc",
            borderwidth=0,
            highlightthickness=0,
            padx=0,
            pady=0,
            takefocus=False,
        )
        status_text.tag_configure("math", foreground="#cccccc", font=("Times", 11, "italic"))
        status_text.tag_configure(
            "subscript",
            foreground="#cccccc",
            font=("Times", 8, "italic"),
            offset=-3,
        )
        status_text.tag_configure("body", foreground="#cccccc")
        for base, subscript, suffix in (
            ("v", "x", ", "),
            ("v", "y", ", and "),
            ("ω", "z", " normalized to the configured MPC limits"),
        ):
            status_text.insert(tk.END, base, "math")
            status_text.insert(tk.END, subscript, "subscript")
            status_text.insert(tk.END, suffix, "body")
        status_text.configure(state=tk.DISABLED)
        status_text.pack(side=tk.LEFT, padx=6)

        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(20, self._publish_periodically)

    def _changed(self):
        return

    def _height_changed(self, value):
        self._height_value.set(f"{float(value):.3f}")

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
