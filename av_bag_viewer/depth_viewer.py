"""ROS2 bag 3D point cloud viewer — compressedDepth + compressed RGB."""

import struct
import sys
import threading
import time

import cv2
import numpy as np

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QSlider, QPushButton, QComboBox, QSplitter, QSizePolicy,
)
import pyqtgraph.opengl as gl

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# ---------------------------------------------------------------------------
# RealSense D435 defaults at 640×480 — override if you have camera_info
# ---------------------------------------------------------------------------
FX = 615.0
FY = 615.0
CX = 320.0
CY = 240.0

BAG_PATH = '/home/branimir/Desktop/rosbag2_2026_03_24-14_08_28'

DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth'
COLOR_TOPIC = '/camera/camera/color/image_raw/compressed'


# ---------------------------------------------------------------------------
# Bag loading helpers
# ---------------------------------------------------------------------------

def decode_compressed_depth(msg) -> np.ndarray:
    """Return float32 depth in metres (0 = invalid)."""
    raw = bytes(msg.data)
    # Skip 12-byte ConfigHeader: int32 format + float32[2] depthParam
    png_buf = np.frombuffer(raw[12:], dtype=np.uint8)
    depth_mm = cv2.imdecode(png_buf, cv2.IMREAD_UNCHANGED)   # uint16, mm
    return depth_mm.astype(np.float32) / 1000.0              # → metres


def decode_compressed_color(msg) -> np.ndarray:
    """Return uint8 RGB array (H×W×3)."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def load_frames(bag_path: str) -> list:
    """
    Return list of (timestamp_ns, depth_m, color_rgb) for every matched pair.
    Pairs are matched by nearest colour to each depth frame.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topic_types = {m.name: m.type for m in reader.get_all_topics_and_types()}

    depth_buf: list = []  # (ts, msg)
    color_buf: list = []  # (ts, msg)

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in (DEPTH_TOPIC, COLOR_TOPIC):
            continue
        msg = deserialize_message(data, get_message(topic_types[topic]))
        if topic == DEPTH_TOPIC:
            depth_buf.append((ts, msg))
        else:
            color_buf.append((ts, msg))

    # Nearest-neighbour matching
    frames = []
    color_ts = np.array([t for t, _ in color_buf], dtype=np.int64)
    for ts, dmsg in depth_buf:
        idx = int(np.argmin(np.abs(color_ts - ts)))
        frames.append((ts, dmsg, color_buf[idx][1]))

    frames.sort(key=lambda x: x[0])
    print(f'  Matched {len(frames)} depth+colour pairs.')
    return frames


# ---------------------------------------------------------------------------
# Point cloud builder
# ---------------------------------------------------------------------------

def make_pointcloud(depth_m: np.ndarray, color_rgb: np.ndarray,
                    max_depth: float = 5.0, stride: int = 2):
    """
    Returns (pos Nx3 float32, color Nx4 float32 RGBA).
    stride=2 → every other pixel for speed.
    """
    h, w = depth_m.shape
    u = np.arange(0, w, stride)
    v = np.arange(0, h, stride)
    uu, vv = np.meshgrid(u, v)

    z = depth_m[vv, uu]
    mask = (z > 0.1) & (z < max_depth)

    z = z[mask]
    x = (uu[mask] - CX) * z / FX
    y = (vv[mask] - CY) * z / FY

    # Camera frame: X right, Y down, Z forward
    # OpenGL view: X right, Y up, Z toward viewer
    # → keep X, flip Y (down→up), keep Z as depth (scene goes into +Z)
    pos = np.column_stack([x, -y, z]).astype(np.float32)

    rgb = color_rgb[vv, uu][mask].astype(np.float32) / 255.0
    alpha = np.ones((rgb.shape[0], 1), dtype=np.float32)
    col = np.hstack([rgb, alpha])

    return pos, col


# ---------------------------------------------------------------------------
# Signals bridge (worker → GUI thread)
# ---------------------------------------------------------------------------

class _Signals(QObject):
    frame_ready = pyqtSignal(int)   # emitted with frame index


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class DepthViewerWindow(QMainWindow):
    def __init__(self, frames: list):
        super().__init__()
        self.setWindowTitle('ROS2 Bag — 3D Depth Viewer')
        self.resize(1400, 800)

        self.frames = frames
        self.current = 0
        self.playing = False
        self.speed = 1.0
        self._signals = _Signals()
        self._signals.frame_ready.connect(self._on_frame_ready)

        self._build_ui()
        # Render first frame after window is shown
        QTimer.singleShot(200, lambda: self._show_frame(0))

    # ---------------------------------------------------------------- UI ---

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(4, 4, 4, 4)

        # ── top splitter: 3D view | side panel ──────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter, stretch=1)

        # 3D view — camera sits behind origin, looks toward +Z (scene depth)
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setMinimumWidth(700)
        self.gl_view.setCameraPosition(distance=5, elevation=15, azimuth=180)
        self.gl_view.opts['center'] = __import__('pyqtgraph').Vector(0, 0, 2)

        # Floor grid in XZ plane (Y is up)
        grid = gl.GLGridItem()
        grid.rotate(90, 1, 0, 0)      # tilt from XY to XZ
        grid.translate(0, -1.0, 2.5)  # shift to floor level, centered on scene
        grid.setSize(6, 6)
        grid.setSpacing(0.5, 0.5)
        self.gl_view.addItem(grid)

        # Axis indicator at origin
        ax = gl.GLAxisItem()
        ax.setSize(0.3, 0.3, 0.3)
        self.gl_view.addItem(ax)

        self.scatter = gl.GLScatterPlotItem(size=2, pxMode=True)
        self.gl_view.addItem(self.scatter)
        splitter.addWidget(self.gl_view)

        # Side panel: RGB + depth images
        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(4, 0, 0, 0)
        side.setFixedWidth(340)

        side_layout.addWidget(QLabel('<b>RGB</b>'))
        self.rgb_label = QLabel(alignment=Qt.AlignCenter)
        self.rgb_label.setFixedSize(320, 240)
        self.rgb_label.setStyleSheet('background: #111;')
        side_layout.addWidget(self.rgb_label)

        side_layout.addWidget(QLabel('<b>Depth (colormap)</b>'))
        self.depth_label = QLabel(alignment=Qt.AlignCenter)
        self.depth_label.setFixedSize(320, 240)
        self.depth_label.setStyleSheet('background: #111;')
        side_layout.addWidget(self.depth_label)

        self.info_label = QLabel()
        self.info_label.setStyleSheet('font-family: monospace; font-size: 11px;')
        side_layout.addWidget(self.info_label)
        side_layout.addStretch()
        splitter.addWidget(side)

        # ── controls bar ────────────────────────────────────────────────
        ctrl = QWidget()
        ctrl_layout = QHBoxLayout(ctrl)
        ctrl_layout.setContentsMargins(4, 2, 4, 2)
        root_layout.addWidget(ctrl)

        self.play_btn = QPushButton('▶  Play')
        self.play_btn.setFixedWidth(90)
        self.play_btn.setStyleSheet('background:#4caf50;color:white;font-size:13px;')
        self.play_btn.clicked.connect(self._toggle_play)
        ctrl_layout.addWidget(self.play_btn)

        reset_btn = QPushButton('⏮')
        reset_btn.setFixedWidth(40)
        reset_btn.clicked.connect(self._reset)
        ctrl_layout.addWidget(reset_btn)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, max(len(self.frames) - 1, 1))
        self.slider.valueChanged.connect(self._on_slider)
        ctrl_layout.addWidget(self.slider, stretch=1)

        ctrl_layout.addWidget(QLabel('Speed:'))
        self.speed_cb = QComboBox()
        self.speed_cb.addItems(['0.25x', '0.5x', '1x', '2x', '4x'])
        self.speed_cb.setCurrentText('1x')
        self.speed_cb.currentTextChanged.connect(self._on_speed)
        self.speed_cb.setFixedWidth(70)
        ctrl_layout.addWidget(self.speed_cb)

        ctrl_layout.addWidget(QLabel('Max depth (m):'))
        self.maxdepth_cb = QComboBox()
        self.maxdepth_cb.addItems(['1', '2', '3', '5', '8'])
        self.maxdepth_cb.setCurrentText('5')
        self.maxdepth_cb.currentTextChanged.connect(lambda _: self._show_frame(self.current))
        self.maxdepth_cb.setFixedWidth(55)
        ctrl_layout.addWidget(self.maxdepth_cb)

        self.time_label = QLabel('0.00 s / 0.00 s')
        self.time_label.setFixedWidth(130)
        ctrl_layout.addWidget(self.time_label)

    # ---------------------------------------------------------- slots ----

    def _toggle_play(self):
        self.playing = not self.playing
        if self.playing:
            self.play_btn.setText('⏸ Pause')
            self.play_btn.setStyleSheet('background:#e67e22;color:white;font-size:13px;')
            threading.Thread(target=self._playback_thread, daemon=True).start()
        else:
            self.play_btn.setText('▶  Play')
            self.play_btn.setStyleSheet('background:#4caf50;color:white;font-size:13px;')

    def _reset(self):
        self.playing = False
        self.play_btn.setText('▶  Play')
        self.play_btn.setStyleSheet('background:#4caf50;color:white;font-size:13px;')
        self.slider.setValue(0)

    def _on_slider(self, val):
        if val != self.current:
            self._show_frame(val)

    def _on_speed(self, text):
        try:
            self.speed = float(text.replace('x', ''))
        except ValueError:
            self.speed = 1.0

    def _on_frame_ready(self, idx):
        self._show_frame(idx)

    # --------------------------------------------------- playback -------

    def _playback_thread(self):
        while self.playing and self.current < len(self.frames) - 1:
            ts_now = self.frames[self.current][0]
            next_idx = self.current + 1
            ts_next = self.frames[next_idx][0]
            delay = (ts_next - ts_now) / 1e9 / self.speed
            time.sleep(max(0.001, delay))
            if not self.playing:
                break
            self._signals.frame_ready.emit(next_idx)
        self.playing = False
        self.play_btn.setText('▶  Play')
        self.play_btn.setStyleSheet('background:#4caf50;color:white;font-size:13px;')

    # --------------------------------------------------- rendering ------

    def _show_frame(self, idx: int):
        if not self.frames:
            return
        idx = max(0, min(idx, len(self.frames) - 1))
        self.current = idx
        self.slider.blockSignals(True)
        self.slider.setValue(idx)
        self.slider.blockSignals(False)

        ts, dmsg, cmsg = self.frames[idx]
        ts0 = self.frames[0][0]
        ts_end = self.frames[-1][0]
        elapsed = (ts - ts0) / 1e9
        total = (ts_end - ts0) / 1e9
        self.time_label.setText(f'{elapsed:6.2f} s / {total:.2f} s')

        depth_m = decode_compressed_depth(dmsg)
        color_rgb = decode_compressed_color(cmsg)

        max_d = float(self.maxdepth_cb.currentText())

        # --- 3D scatter ---
        pos, col = make_pointcloud(depth_m, color_rgb, max_depth=max_d)
        self.scatter.setData(pos=pos, color=col)

        # --- RGB thumbnail ---
        self._set_image_label(self.rgb_label, color_rgb, is_depth=False)

        # --- Depth colormap thumbnail ---
        depth_vis = np.clip(depth_m / max_d, 0, 1)
        depth_u8 = (depth_vis * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        depth_rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)
        self._set_image_label(self.depth_label, depth_rgb, is_depth=False)

        # --- Info ---
        valid = (depth_m > 0.1) & (depth_m < max_d)
        self.info_label.setText(
            f'Frame {idx + 1}/{len(self.frames)}\n'
            f'Depth range: {depth_m[valid].min():.2f}–{depth_m[valid].max():.2f} m\n'
            f'Points: {valid.sum():,}'
        )

    @staticmethod
    def _set_image_label(label: QLabel, rgb: np.ndarray, is_depth=False):
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            label.width(), label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        label.setPixmap(pix)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    bag_path = BAG_PATH
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--ros-args'):
        bag_path = sys.argv[1]

    print(f'Loading bag: {bag_path}')
    rclpy.init(args=args)
    try:
        frames = load_frames(bag_path)
    except Exception as exc:
        print(f'Error: {exc}')
        rclpy.shutdown()
        return
    rclpy.shutdown()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = DepthViewerWindow(frames)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
