"""ROS2 bag file viewer with Tkinter GUI."""

import io
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

import numpy as np
from PIL import Image, ImageTk

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py


BAG_PATH = '/home/branimir/Desktop/rosbag2_2026_03_24-13_11_16'


def load_bag(bag_path):
    """Read all messages from a bag and return sorted by timestamp."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    topic_types = {}
    for meta in reader.get_all_topics_and_types():
        topic_types[meta.name] = meta.type

    messages = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        messages.append((timestamp, topic, msg))

    messages.sort(key=lambda x: x[0])
    return messages


class BagViewerApp:
    def __init__(self, root, messages):
        self.root = root
        self.root.title('ROS2 Bag Viewer')
        self.root.geometry('1100x700')

        self.messages = messages
        self.playing = False
        self.current_index = 0
        self.playback_speed = 1.0

        # Separate indices per topic for efficient lookup
        self.image_msgs = [(i, ts, m) for i, (ts, t, m) in enumerate(messages)
                           if t == '/camera/camera/color/image_raw/compressed']
        self.odom_msgs = [(i, ts, m) for i, (ts, t, m) in enumerate(messages)
                          if t == '/odom']

        # Odometry path accumulator
        self.odom_x = []
        self.odom_y = []

        self._build_ui()
        self.root.after(100, lambda: self._update_display(0))

    # ------------------------------------------------------------------ UI --

    def _build_ui(self):
        # --- Top bar ---
        top = tk.Frame(self.root, bg='#2b2b2b')
        top.pack(fill=tk.X, padx=5, pady=5)

        tk.Label(top, text='ROS2 Bag Viewer', font=('Helvetica', 14, 'bold'),
                 fg='white', bg='#2b2b2b').pack(side=tk.LEFT, padx=10)

        tk.Button(top, text='Open Bag…', command=self._open_bag,
                  bg='#4a90d9', fg='white').pack(side=tk.RIGHT, padx=5)

        # Speed selector
        tk.Label(top, text='Speed:', fg='white', bg='#2b2b2b').pack(
            side=tk.RIGHT, padx=(10, 2))
        self.speed_var = tk.StringVar(value='1x')
        speed_cb = ttk.Combobox(top, textvariable=self.speed_var,
                                values=['0.25x', '0.5x', '1x', '2x', '4x'],
                                width=6, state='readonly')
        speed_cb.pack(side=tk.RIGHT)
        speed_cb.bind('<<ComboboxSelected>>', self._on_speed_change)

        # --- Main area ---
        main = tk.Frame(self.root, bg='#1e1e1e')
        main.pack(fill=tk.BOTH, expand=True, padx=5)

        # Left: camera
        left = tk.Frame(main, bg='#1e1e1e')
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tk.Label(left, text='Camera', fg='#aaaaaa', bg='#1e1e1e',
                 font=('Helvetica', 10)).pack(anchor='w', padx=5)
        self.img_label = tk.Label(left, bg='black')
        self.img_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Right: odometry canvas
        right = tk.Frame(main, bg='#1e1e1e', width=320)
        right.pack(side=tk.RIGHT, fill=tk.BOTH)
        right.pack_propagate(False)

        tk.Label(right, text='Odometry Path (X / Y)',
                 fg='#aaaaaa', bg='#1e1e1e',
                 font=('Helvetica', 10)).pack(anchor='w', padx=5)
        self.odom_canvas = tk.Canvas(right, bg='#111111',
                                     highlightthickness=0)
        self.odom_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Info panel
        self.info_var = tk.StringVar(value='')
        tk.Label(right, textvariable=self.info_var, fg='#cccccc',
                 bg='#1e1e1e', font=('Courier', 9),
                 justify=tk.LEFT, anchor='w').pack(fill=tk.X, padx=8, pady=4)

        # --- Timeline ---
        bottom = tk.Frame(self.root, bg='#2b2b2b')
        bottom.pack(fill=tk.X, padx=5, pady=5)

        self.play_btn = tk.Button(bottom, text='▶  Play', width=9,
                                  command=self._toggle_play,
                                  bg='#4caf50', fg='white', font=('Helvetica', 10))
        self.play_btn.pack(side=tk.LEFT, padx=5)

        tk.Button(bottom, text='⏮ Reset', command=self._reset,
                  bg='#666', fg='white').pack(side=tk.LEFT, padx=2)

        self.time_label = tk.Label(bottom, text='0.00 s / 0.00 s',
                                   fg='white', bg='#2b2b2b', width=18)
        self.time_label.pack(side=tk.RIGHT, padx=10)

        self.slider_var = tk.IntVar()
        self.slider = ttk.Scale(bottom, from_=0,
                                to=max(len(self.messages) - 1, 1),
                                orient=tk.HORIZONTAL,
                                variable=self.slider_var,
                                command=self._on_slider)
        self.slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

    # --------------------------------------------------------- callbacks ----

    def _open_bag(self):
        path = filedialog.askdirectory(title='Select ROS2 bag directory')
        if not path:
            return
        self.playing = False
        self.play_btn.config(text='▶  Play', bg='#4caf50')
        try:
            msgs = load_bag(path)
        except Exception as exc:
            tk.messagebox.showerror('Error', str(exc))
            return
        self.messages = msgs
        self.image_msgs = [(i, ts, m) for i, (ts, t, m) in enumerate(msgs)
                           if t == '/camera/camera/color/image_raw/compressed']
        self.odom_msgs = [(i, ts, m) for i, (ts, t, m) in enumerate(msgs)
                          if t == '/odom']
        self.odom_x.clear()
        self.odom_y.clear()
        self.current_index = 0
        self.slider.config(to=max(len(msgs) - 1, 1))
        self._update_display(0)

    def _toggle_play(self):
        self.playing = not self.playing
        if self.playing:
            self.play_btn.config(text='⏸ Pause', bg='#e67e22')
            threading.Thread(target=self._playback_thread, daemon=True).start()
        else:
            self.play_btn.config(text='▶  Play', bg='#4caf50')

    def _reset(self):
        self.playing = False
        self.play_btn.config(text='▶  Play', bg='#4caf50')
        self.odom_x.clear()
        self.odom_y.clear()
        self.current_index = 0
        self.slider_var.set(0)
        self._update_display(0)

    def _on_slider(self, val):
        idx = int(float(val))
        if idx != self.current_index:
            self.current_index = idx
            self._update_display(idx)

    def _on_speed_change(self, _event=None):
        text = self.speed_var.get().replace('x', '')
        try:
            self.playback_speed = float(text)
        except ValueError:
            self.playback_speed = 1.0

    # ------------------------------------------------------ playback -------

    def _playback_thread(self):
        while self.playing and self.current_index < len(self.messages) - 1:
            ts_now = self.messages[self.current_index][0]
            next_idx = self.current_index + 1
            ts_next = self.messages[next_idx][0]
            delay = (ts_next - ts_now) / 1e9 / self.playback_speed
            time.sleep(max(0.0, delay))
            if not self.playing:
                break
            self.current_index = next_idx
            self.root.after(0, self._update_display, next_idx)
        self.playing = False
        self.root.after(0, lambda: self.play_btn.config(
            text='▶  Play', bg='#4caf50'))

    # ---------------------------------------------------- display ----------

    def _update_display(self, idx):
        if not self.messages:
            return
        idx = max(0, min(idx, len(self.messages) - 1))
        self.current_index = idx
        self.slider_var.set(idx)

        ts0 = self.messages[0][0]
        ts_cur = self.messages[idx][0]
        ts_end = self.messages[-1][0]
        elapsed = (ts_cur - ts0) / 1e9
        total = (ts_end - ts0) / 1e9
        self.time_label.config(text=f'{elapsed:6.2f} s / {total:.2f} s')

        # Find latest image and odometry up to current index
        img_frame = self._latest_msg_before(self.image_msgs, idx)
        odom_frame = self._latest_msg_before(self.odom_msgs, idx)

        if img_frame is not None:
            self._show_image(img_frame)

        if odom_frame is not None:
            self._update_odom(odom_frame, idx)

    def _latest_msg_before(self, pool, idx):
        """Return the last message in pool whose global index <= idx."""
        result = None
        for (global_idx, _ts, msg) in pool:
            if global_idx <= idx:
                result = msg
            else:
                break
        return result

    def _show_image(self, msg):
        try:
            img = Image.open(io.BytesIO(bytes(msg.data)))
            # Fit inside label
            w = max(self.img_label.winfo_width(), 10)
            h = max(self.img_label.winfo_height(), 10)
            img.thumbnail((w, h), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.img_label.config(image=photo)
            self.img_label.image = photo  # keep reference
        except Exception:
            pass

    def _update_odom(self, msg, current_idx):
        # Accumulate all odom up to current_idx
        self.odom_x.clear()
        self.odom_y.clear()
        for (global_idx, _ts, m) in self.odom_msgs:
            if global_idx > current_idx:
                break
            self.odom_x.append(m.pose.pose.position.x)
            self.odom_y.append(m.pose.pose.position.y)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        self.info_var.set(
            f'Pos  x={x:+.3f}  y={y:+.3f}\n'
            f'Vel  vx={vx:+.3f}  vy={vy:+.3f}'
        )

        self._draw_odom_path()

    def _draw_odom_path(self):
        c = self.odom_canvas
        c.delete('all')
        if len(self.odom_x) < 2:
            return

        cw = max(c.winfo_width(), 10)
        ch = max(c.winfo_height(), 10)
        pad = 20

        xs = np.array(self.odom_x)
        ys = np.array(self.odom_y)

        x_min, x_max = xs.min(), xs.max()
        y_min, y_max = ys.min(), ys.max()
        x_range = max(x_max - x_min, 0.01)
        y_range = max(y_max - y_min, 0.01)

        def to_canvas(xi, yi):
            cx = pad + (xi - x_min) / x_range * (cw - 2 * pad)
            cy = ch - pad - (yi - y_min) / y_range * (ch - 2 * pad)
            return cx, cy

        # Grid lines
        for v in np.linspace(y_min, y_max, 5):
            _, cy = to_canvas(x_min, v)
            c.create_line(pad, cy, cw - pad, cy, fill='#333333')
        for v in np.linspace(x_min, x_max, 5):
            cx, _ = to_canvas(v, y_min)
            c.create_line(cx, pad, cx, ch - pad, fill='#333333')

        # Path
        pts = [to_canvas(xi, yi) for xi, yi in zip(xs, ys)]
        flat = [coord for pt in pts for coord in pt]
        if len(flat) >= 4:
            c.create_line(*flat, fill='#4a90d9', width=2, smooth=True)

        # Current position dot
        cx, cy = pts[-1]
        r = 5
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      fill='#ff6b6b', outline='white', width=1)

        # Axis labels
        c.create_text(cw // 2, ch - 5, text='X', fill='#888888', font=('Helvetica', 8))
        c.create_text(8, ch // 2, text='Y', fill='#888888', font=('Helvetica', 8))


# --------------------------------------------------------------------------

def main(args=None):
    bag_path = BAG_PATH

    # Allow overriding via command line: ros2 run bag_viewer bag_viewer /path/to/bag
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--ros-args'):
        bag_path = sys.argv[1]

    print(f'Loading bag: {bag_path}')
    rclpy.init(args=args)

    try:
        messages = load_bag(bag_path)
    except Exception as exc:
        print(f'Error loading bag: {exc}')
        rclpy.shutdown()
        return

    print(f'Loaded {len(messages)} messages.')
    rclpy.shutdown()  # done with ROS — release before entering Tkinter mainloop

    root = tk.Tk()
    app = BagViewerApp(root, messages)  # noqa: F841
    root.mainloop()


if __name__ == '__main__':
    main()
