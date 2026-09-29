import queue
import socket
import struct
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import matplotlib
try:
    matplotlib.use("TkAgg")
except ImportError:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

HOST = "127.0.0.1"
PORT = 12345
IMG_WIDTH = 1020
IMG_HEIGHT = 1020
RAW_IMAGE_BYTES = IMG_WIDTH * IMG_HEIGHT
CAMERA_EXTENT = (0, IMG_WIDTH, IMG_HEIGHT, 0)

# The firmware uses this fixed L4 block size for progressive full-frame transfer.
L4_BLOCK_SIZE = 16
L4_REDRAW_EVERY = 8
L3_REDRAW_EVERY = 8
L2_REDRAW_EVERY = 32

MODE_COMMANDS = {
    "l0": b"AURA L0\n",
    "l1": b"AURA L1\n",
    "l2": b"AURA L2\n",
    "l3": b"AURA L3\n",
    "l4": b"AURA L4\n",
}

MODE_NAMES = {
    "l0": "L0 — Top-100 Landmark Cloud + Score",
    "l1": "L1 — Top-1000 Landmark Map (X,Y only)",
    "l2": "L2 — Full Adaptive Entropy Map",
    "l3": "L3 — Sparse ROI Image",
    "l4": "L4 — Full Image / Block Stream",
}

MARKER_ACK = 0xAC
MARKER_L0_HEADER = 0x5B
MARKER_L0_POINT = 0xBD
MARKER_L1_HEADER = 0xB1
MARKER_L2_HEADER = 0xD0
MARKER_L2_ENTROPY = 0xD1
MARKER_L2_COUNT = 0xD2
MARKER_L3_HEADER = 0xD3
MARKER_L3_META = 0xD4
MARKER_L3_BLOCK = 0xD5
MARKER_L3_COUNT = 0xD6
MARKER_FULL_HEADER = 0xC0
MARKER_FULL_BLOCK = 0xC3
MARKER_END = 0xFE

ACK_NAMES = {
    10: "L0 TOP-100",
    11: "L1 TOP-1000 (X,Y)",
    12: "L2 FULL ENTROPY",
    13: "L3 SPARSE ROI",
    14: "L4 FULL FRAME STREAM",
}


class AuraGroundUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("AURA V0.3 — Ground Segment / Interactive Telemetry Console")
        self.root.geometry("1280x1280")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.receiver_thread = None
        self.stop_event = threading.Event()
        self.events = queue.Queue()

        self.connected = False
        self.busy = False
        self.current_mode = None

        # Cloud state.
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_score = []
        self.cloud_requested = 0
        self.cloud_step = 0
        self.cloud_threshold = 0
        self.cloud_is_scored = True

        # L1 packed point stream state.
        self.l1_expected_points = 0
        self.l1_expected_bytes = 0
        self.l1_point_buffer = bytearray()

        # L2 full entropy map state.
        self.heatmap = None
        self.heatmap_rows = 0
        self.heatmap_cols = 0
        self.block_size = 0
        self.heatmap_block_count = 0
        self.heatmap_expected_blocks = 0

        # L3/L4 image reconstruction state.
        self.roi_frame = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
        self.l4_frame = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
        self.roi_block_size = 0
        self.roi_cols = 0
        self.roi_rows = 0
        self.roi_threshold = None
        self.roi_expected_blocks = 0
        self.roi_received_blocks = 0
        self.l4_block_size = 0
        self.l4_cols = 0
        self.l4_rows = 0
        self.l4_expected_blocks = 0
        self.l4_received_blocks = 0

        # Exact raw block receive state.
        self.raw_expected = 0
        self.raw_buffer = bytearray()
        self.raw_block_col = 0
        self.raw_block_row = 0
        self.raw_block_w = 0
        self.raw_block_h = 0
        self.raw_block_mode = None

        self.parser_buffer = bytearray()
        self.parser_state = "packet"

        self.rx_bytes_total = 0
        self.frame_started_bytes = 0

        # Matplotlib artists. Created once per mode and then updated in-place.
        self.cbar = None
        self.cloud_scatter = None
        self.im_data = None

        self._build_ui()
        self._configure_styles()
        self.root.after(50, self._process_events)
        self.root.after(100, self.toggle_connection)

    def _configure_styles(self):
        self.style = ttk.Style()
        self.style.configure("Connect.TButton", font=("Helvetica", 10, "bold"))
        self.style.configure("Disconnect.TButton", font=("Helvetica", 10, "bold"))

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(
            top,
            text="AURA Ground Segment Console V0.3",
            font=("Helvetica", 16, "bold"),
        ).pack(side="left", padx=(0, 18))

        self.status_var = tk.StringVar(value="Initializing connection to Renode LEON3 core...")
        ttk.Label(top, textvariable=self.status_var, font=("Helvetica", 10, "italic")).pack(side="left")

        self.connect_button = ttk.Button(
            top,
            text="Connect",
            command=self.toggle_connection,
            style="Connect.TButton",
        )
        self.connect_button.pack(side="right")

        controls = ttk.LabelFrame(self.root, text="Interactive Telemetry Request Panel", padding=10)
        controls.pack(fill="x", padx=10, pady=(0, 10))

        self.buttons = {}
        button_specs = [
            ("l0", "L0 — Top-100 + score"),
            ("l1", "L1 — Top-1000 X/Y"),
            ("l2", "L2 — Full entropy"),
            ("l3", "L3 — ROI blocks"),
            ("l4", "L4 — Full block stream"),
        ]
        for mode, label in button_specs:
            btn = ttk.Button(controls, text=label, command=lambda m=mode: self.request_mode(m))
            btn.pack(side="left", padx=4, fill="x", expand=True)
            self.buttons[mode] = btn

        self.info_var = tk.StringVar(value="System Idle. Awaiting connection.")
        ttk.Label(
            self.root,
            textvariable=self.info_var,
            anchor="w",
            font=("Courier New", 10, "bold"),
            padding=(10, 0, 10, 8),
        ).pack(fill="x")

        fig_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        fig_frame.pack(fill="both", expand=True)

        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=fig_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_empty()

    def _remove_colorbar(self):
        if self.cbar is not None:
            try:
                self.cbar.remove()
            except Exception:
                pass
            self.cbar = None

    def _clear_extra_axes(self):
        self._remove_colorbar()
        for extra_ax in self.fig.axes[:]:
            if extra_ax is not self.ax:
                self.fig.delaxes(extra_ax)
        self.im_data = None

    def _fix_camera_axes(self, title, xlabel="Pixel X", ylabel="Pixel Y"):
        self.ax.set_xlim(0, IMG_WIDTH)
        self.ax.set_ylim(IMG_HEIGHT, 0)
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title(title)

    def _draw_empty(self):
        self.ax.clear()
        self._clear_extra_axes()
        self.cloud_scatter = None
        self._fix_camera_axes("AURA V0.3 — Select Telemetry Level")
        self.canvas.draw_idle()

    def set_buttons_state(self, enabled: bool):
        state = "normal" if enabled and self.connected and not self.busy else "disabled"
        for btn in self.buttons.values():
            btn.configure(state=state)

    def toggle_connection(self):
        if self.connected:
            self.stop_event.set()
            with self.sock_lock:
                if self.sock:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
            self.events.put(("disconnected", None))
            return

        self.status_var.set(f"Connecting to LEON3 target at {HOST}:{PORT}...")
        self.connect_button.configure(state="disabled")

        def worker():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
                sock.connect((HOST, PORT))
                self.events.put(("connected", sock))
                self._recv_loop(sock)
            except ConnectionRefusedError:
                self.events.put(("error", "Connection refused. Start Renode script.resc first."))
            except OSError as exc:
                self.events.put(("error", f"Socket transport error: {exc}"))

        self.stop_event.clear()
        self.receiver_thread = threading.Thread(target=worker, daemon=True)
        self.receiver_thread.start()

    def _recv_loop(self, sock):
        while not self.stop_event.is_set():
            try:
                chunk = sock.recv(16384)
                if not chunk:
                    break
                self.parser_buffer.extend(chunk)
                self.rx_bytes_total += len(chunk)
                self._parse_buffer()
            except OSError:
                break
        self.events.put(("disconnected", None))

    @staticmethod
    def _block_shape(block_size: int, col: int, row: int):
        x0 = col * block_size
        y0 = row * block_size
        bw = min(block_size, IMG_WIDTH - x0)
        bh = min(block_size, IMG_HEIGHT - y0)
        return x0, y0, bw, bh

    def _decode_l1_points(self, payload: bytes, count: int):
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_score = [0] * count
        for i in range(count):
            base = i * 3
            b0 = payload[base]
            b1 = payload[base + 1]
            b2 = payload[base + 2]
            x = (b0 << 2) | (b1 >> 6)
            y = ((b1 & 0x3F) << 4) | (b2 >> 4)
            self.cloud_x.append(x)
            self.cloud_y.append(y)

    def _parse_buffer(self):
        while True:
            # Raw payload modes are length-delimited and consume bytes without
            # interpreting protocol markers inside the image data.
            if self.parser_state in ("l1_points", "image_block"):
                if self.raw_expected <= 0:
                    self.parser_state = "packet"
                    continue

                take = min(len(self.parser_buffer), self.raw_expected - len(self.raw_buffer))
                if take <= 0:
                    return

                self.raw_buffer.extend(self.parser_buffer[:take])
                del self.parser_buffer[:take]

                if len(self.raw_buffer) != self.raw_expected:
                    return

                payload = bytes(self.raw_buffer)
                if self.parser_state == "l1_points":
                    if len(payload) == self.l1_expected_bytes:
                        self._decode_l1_points(payload, self.l1_expected_points)
                        self.events.put(("l1_points_complete", None))
                else:
                    block = np.frombuffer(payload, dtype=np.uint8).copy().reshape(
                        (self.raw_block_h, self.raw_block_w)
                    )
                    self.events.put((
                        "image_block_complete",
                        (
                            self.raw_block_mode,
                            self.raw_block_col,
                            self.raw_block_row,
                            block,
                        ),
                    ))

                self.raw_expected = 0
                self.raw_buffer.clear()
                self.parser_state = "packet"
                continue

            if len(self.parser_buffer) < 4:
                return

            word = struct.unpack(">I", self.parser_buffer[:4])[0]
            marker = (word >> 24) & 0xFF

            if marker == MARKER_END:
                del self.parser_buffer[:4]
                self.events.put(("frame_complete", self.current_mode))
                continue

            if marker == MARKER_ACK:
                del self.parser_buffer[:4]
                self.events.put(("ack", word & 0xFF))
                continue

            if marker == MARKER_L0_HEADER:
                del self.parser_buffer[:4]
                count = (word >> 12) & 0x0FFF
                step = (word >> 8) & 0x0F
                threshold = word & 0xFF
                self.current_mode = "l0"
                self.cloud_requested = count
                self.cloud_step = step
                self.cloud_threshold = threshold
                self.cloud_is_scored = True
                self.cloud_x = []
                self.cloud_y = []
                self.cloud_score = []
                self.events.put(("cloud_header", ("l0", count, step, threshold)))
                continue

            if marker == MARKER_L0_POINT:
                del self.parser_buffer[:4]
                x = (word >> 14) & 0x03FF
                y = (word >> 4) & 0x03FF
                score = word & 0x0F
                self.cloud_x.append(x)
                self.cloud_y.append(y)
                self.cloud_score.append(score)
                if len(self.cloud_x) % 16 == 0:
                    self.events.put(("cloud_update", None))
                continue

            if marker == MARKER_L1_HEADER:
                del self.parser_buffer[:4]
                count = (word >> 12) & 0x0FFF
                step = (word >> 8) & 0x0F
                threshold = word & 0xFF
                self.current_mode = "l1"
                self.cloud_requested = count
                self.cloud_step = step
                self.cloud_threshold = threshold
                self.cloud_is_scored = False
                self.l1_expected_points = count
                self.l1_expected_bytes = count * 3
                self.l1_point_buffer.clear()
                self.cloud_x = []
                self.cloud_y = []
                self.cloud_score = []
                if self.l1_expected_bytes == 0:
                    self.events.put(("l1_points_complete", None))
                else:
                    self.raw_expected = self.l1_expected_bytes
                    self.parser_state = "l1_points"
                self.events.put(("l1_header", (count, step, threshold)))
                continue

            if marker == MARKER_L2_HEADER:
                del self.parser_buffer[:4]
                block = (word >> 16) & 0xFF
                cols = (word >> 8) & 0xFF
                rows = word & 0xFF
                self.current_mode = "l2"
                self.block_size = block
                self.heatmap_cols = cols
                self.heatmap_rows = rows
                self.heatmap = np.zeros((rows, cols), dtype=np.float32)
                self.heatmap_block_count = 0
                self.heatmap_expected_blocks = 0
                self.events.put(("l2_header", (rows, cols, block)))
                continue

            if marker == MARKER_L2_ENTROPY:
                del self.parser_buffer[:4]
                col = (word >> 16) & 0xFF
                row = (word >> 8) & 0xFF
                entropy = (word & 0xFF) * 0.1
                if self.heatmap is not None and row < self.heatmap_rows and col < self.heatmap_cols:
                    self.heatmap[row, col] = entropy
                    self.heatmap_block_count += 1
                    if self.heatmap_block_count % L2_REDRAW_EVERY == 0:
                        self.events.put(("l2_update", None))
                continue

            if marker == MARKER_L2_COUNT:
                del self.parser_buffer[:4]
                self.heatmap_expected_blocks = word & 0xFFFF
                self.events.put(("l2_count", self.heatmap_expected_blocks))
                continue

            if marker == MARKER_L3_HEADER:
                del self.parser_buffer[:4]
                block = (word >> 16) & 0xFF
                cols = (word >> 8) & 0xFF
                rows = word & 0xFF
                self.current_mode = "l3"
                self.roi_block_size = block
                self.roi_cols = cols
                self.roi_rows = rows
                self.roi_frame.fill(0)
                self.roi_expected_blocks = 0
                self.roi_received_blocks = 0
                self.events.put(("l3_header", (rows, cols, block)))
                continue

            if marker == MARKER_L3_META:
                del self.parser_buffer[:4]
                self.roi_threshold = (word & 0xFFFF) * 0.01
                self.events.put(("l3_meta", self.roi_threshold))
                continue

            if marker == MARKER_L3_COUNT:
                del self.parser_buffer[:4]
                self.roi_expected_blocks = word & 0xFFFF
                self.events.put(("l3_count", self.roi_expected_blocks))
                continue

            if marker == MARKER_L3_BLOCK:
                del self.parser_buffer[:4]
                col = (word >> 16) & 0xFF
                row = (word >> 8) & 0xFF
                if self.roi_block_size <= 0 or col >= self.roi_cols or row >= self.roi_rows:
                    continue
                x0, y0, bw, bh = self._block_shape(self.roi_block_size, col, row)
                if bw <= 0 or bh <= 0:
                    continue
                self.raw_block_mode = "l3"
                self.raw_block_col = col
                self.raw_block_row = row
                self.raw_block_w = bw
                self.raw_block_h = bh
                self.raw_expected = bw * bh
                self.raw_buffer.clear()
                self.parser_state = "image_block"
                continue

            if marker == MARKER_FULL_HEADER:
                del self.parser_buffer[:4]
                block = (word >> 16) & 0xFF
                cols = (word >> 8) & 0xFF
                rows = word & 0xFF
                self.current_mode = "l4"
                self.l4_block_size = block
                self.l4_cols = cols
                self.l4_rows = rows
                self.l4_expected_blocks = rows * cols
                self.l4_received_blocks = 0
                self.l4_frame.fill(0)
                self.events.put(("l4_header", (block, cols, rows)))
                continue

            if marker == MARKER_FULL_BLOCK:
                del self.parser_buffer[:4]
                col = (word >> 16) & 0xFF
                row = (word >> 8) & 0xFF
                if self.l4_block_size <= 0 or col >= self.l4_cols or row >= self.l4_rows:
                    continue
                x0, y0, bw, bh = self._block_shape(self.l4_block_size, col, row)
                if bw <= 0 or bh <= 0:
                    continue
                self.raw_block_mode = "l4"
                self.raw_block_col = col
                self.raw_block_row = row
                self.raw_block_w = bw
                self.raw_block_h = bh
                self.raw_expected = bw * bh
                self.raw_buffer.clear()
                self.parser_state = "image_block"
                continue

            # Resynchronize against stray ASCII startup text or unknown bytes.
            del self.parser_buffer[0]

    def request_mode(self, mode: str):
        if not self.connected or self.busy:
            return

        with self.sock_lock:
            try:
                self.sock.sendall(MODE_COMMANDS[mode])
            except OSError as exc:
                self.status_var.set(f"Tx Error: {exc}")
                return

        self.busy = True
        self.current_mode = mode
        self.frame_started_bytes = self.rx_bytes_total
        self.current_frame = None

        self.cloud_x = []
        self.cloud_y = []
        self.cloud_score = []
        self.cloud_requested = 0
        self.cloud_is_scored = mode == "l0"

        self.heatmap = None
        self.heatmap_rows = 0
        self.heatmap_cols = 0
        self.block_size = 0
        self.heatmap_block_count = 0
        self.heatmap_expected_blocks = 0

        self.roi_frame.fill(0)
        self.l4_frame.fill(0)
        self.roi_received_blocks = 0
        self.roi_expected_blocks = 0
        self.l4_received_blocks = 0
        self.l4_expected_blocks = 0
        self.roi_threshold = None
        self.raw_expected = 0
        self.raw_buffer.clear()
        self.parser_state = "packet"
        self.raw_block_mode = None

        self._prepare_mode_view(mode)
        self.info_var.set(f"Request sent: {MODE_NAMES[mode]} — awaiting payload...")
        self.set_buttons_state(False)
        self.canvas.draw_idle()

    def _prepare_mode_view(self, mode):
        self.ax.clear()
        self._clear_extra_axes()
        self.cloud_scatter = None

        if mode in ("l0", "l1"):
            self.ax.set_facecolor("#0b0c10")
            self._fix_camera_axes(MODE_NAMES[mode])
            return

        if mode == "l2":
            self.heatmap = None
            self._fix_camera_axes("AURA — Full Adaptive Entropy Map", "Pixel X", "Pixel Y")
            return

        if mode == "l3":
            self._create_image_artist(self.roi_frame, "AURA — Sparse ROI Reconstruction")
            return

        if mode == "l4":
            self._create_image_artist(self.l4_frame, "AURA — Full Frame Streaming (top → bottom, left → right)")

    def _create_image_artist(self, image, title):
        self.im_data = self.ax.imshow(
            image,
            cmap="gray",
            vmin=0,
            vmax=255,
            interpolation="nearest",
            extent=CAMERA_EXTENT,
            origin="upper",
            aspect="equal",
        )
        self._fix_camera_axes(title)

    def _render_cloud(self):
        self.ax.clear()
        self._clear_extra_axes()
        self.cloud_scatter = None
        self.ax.set_facecolor("#0b0c10")

        if self.cloud_x:
            if self.current_mode == "l0":
                self.cloud_scatter = self.ax.scatter(
                    self.cloud_x,
                    self.cloud_y,
                    c=self.cloud_score,
                    cmap="plasma",
                    vmin=0,
                    vmax=15,
                    s=24,
                    marker="+",
                )
                title = f"AURA — L0 Top-100 Contrast Cloud ({len(self.cloud_x)} features, score shown)"
            else:
                # L1 deliberately has no value channel: only X/Y are transmitted.
                self.cloud_scatter = self.ax.scatter(
                    self.cloud_x,
                    self.cloud_y,
                    s=24,
                    marker="+",
                )
                title = f"AURA — L1 Top-1000 Landmark Map ({len(self.cloud_x)} points, X/Y only)"
        else:
            title = MODE_NAMES.get(self.current_mode, "AURA — Landmark Map")

        self._fix_camera_axes(title)
        self.canvas.draw_idle()

    def _render_entropy(self):
        if self.heatmap is None:
            return

        if self.im_data is None or self.current_mode != "l2":
            self.ax.clear()
            self._clear_extra_axes()
            self.im_data = self.ax.imshow(
                self.heatmap,
                cmap="jet",
                interpolation="nearest",
                vmin=0,
                vmax=8,
                extent=CAMERA_EXTENT,
                origin="upper",
                aspect="equal",
            )
            # Create exactly one colorbar for the L2 view. Never recreate it on updates.
            cax = inset_axes(
                self.ax,
                width="3%",
                height="80%",
                loc="center right",
                borderpad=1.5
            )

            self.cbar = self.fig.colorbar(self.im_data, cax=cax)
            self.cbar.set_label(
                "Shannon entropy (bits/pixel)",
                rotation=270,
                labelpad=15
            )

        else:
            self.im_data.set_data(self.heatmap)

        self._fix_camera_axes(
            f"AURA — FULL Entropy Map ({self.heatmap_cols}×{self.heatmap_rows}, block={self.block_size}px)",
            "Camera Pixel X",
            "Camera Pixel Y",
        )
        self.canvas.draw_idle()

    def _render_roi(self):
        if self.im_data is None or self.current_mode != "l3":
            self.ax.clear()
            self._clear_extra_axes()
            self._create_image_artist(
                self.roi_frame,
                f"AURA — Sparse ROI Reconstruction ({self.roi_received_blocks}/{self.roi_expected_blocks or '?'})",
            )
        else:
            self.im_data.set_data(self.roi_frame)
            self._fix_camera_axes(
                f"AURA — Sparse ROI Reconstruction ({self.roi_received_blocks}/{self.roi_expected_blocks or '?'})"
            )
        self.canvas.draw_idle()

    def _render_l4(self):
        if self.im_data is None or self.current_mode != "l4":
            self.ax.clear()
            self._clear_extra_axes()
            self._create_image_artist(
                self.l4_frame,
                f"AURA — Full Frame Streaming ({self.l4_received_blocks}/{self.l4_expected_blocks or '?'})",
            )
        else:
            self.im_data.set_data(self.l4_frame)
            self._fix_camera_axes(
                f"AURA — Full Frame Streaming ({self.l4_received_blocks}/{self.l4_expected_blocks or '?'})"
            )
        self.canvas.draw_idle()

    def _process_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()

                if kind == "connected":
                    self.sock = payload
                    self.connected = True
                    self.busy = False
                    self.status_var.set(f"Active link to LEON3 simulator at {HOST}:{PORT}")
                    self.info_var.set("Link established. Select AURA telemetry level.")
                    self.connect_button.configure(text="Disconnect", style="Disconnect.TButton", state="normal")
                    self.set_buttons_state(True)

                elif kind == "error":
                    self.connected = False
                    self.busy = False
                    self.sock = None
                    self.status_var.set(payload)
                    self.info_var.set("Transport link failure.")
                    self.connect_button.configure(text="Connect", style="Connect.TButton", state="normal")
                    self.set_buttons_state(False)

                elif kind == "disconnected":
                    self.connected = False
                    self.busy = False
                    self.sock = None
                    self.status_var.set("Link to spacecraft core closed.")
                    self.info_var.set("Click Connect to re-establish the link.")
                    self.connect_button.configure(text="Connect", style="Connect.TButton", state="normal")
                    self.set_buttons_state(False)
                    self._draw_empty()

                elif kind == "ack":
                    self.status_var.set(f"LEON3 ACK: {ACK_NAMES.get(payload, 'UNKNOWN')}")

                elif kind == "cloud_header":
                    mode, count, step, threshold = payload
                    self.cloud_requested = count
                    self.cloud_step = step
                    self.cloud_threshold = threshold
                    self.info_var.set(
                        f"{MODE_NAMES[mode]}: count={count}, sampling={step}px, threshold={threshold}"
                    )
                    self._render_cloud()

                elif kind == "cloud_update":
                    self._render_cloud()

                elif kind == "l1_header":
                    count, step, threshold = payload
                    self.info_var.set(
                        f"L1 coordinate stream: {count} points, {count * 3} payload bytes, no score channel"
                    )

                elif kind == "l1_points_complete":
                    self._render_cloud()
                    self.info_var.set(f"L1 map reconstructed: {len(self.cloud_x)} X/Y points")

                elif kind == "l2_header":
                    rows, cols, block = payload
                    self.heatmap_rows = rows
                    self.heatmap_cols = cols
                    self.block_size = block
                    self.heatmap = np.zeros((rows, cols), dtype=np.float32)
                    self.heatmap_block_count = 0
                    self._render_entropy()
                    self.info_var.set(
                        f"L2 FULL map: {cols}×{rows} blocks at {block}px; camera frame fixed at {IMG_WIDTH}×{IMG_HEIGHT}"
                    )

                elif kind == "l2_update":
                    self._render_entropy()

                elif kind == "l2_count":
                    self.heatmap_expected_blocks = payload
                    self.info_var.set(f"L2 FULL entropy blocks: {payload}")

                elif kind == "l3_header":
                    rows, cols, block = payload
                    self.roi_rows = rows
                    self.roi_cols = cols
                    self.roi_block_size = block
                    self._render_roi()
                    self.info_var.set(f"L3 layout: {cols}×{rows}, adaptive block={block}px")

                elif kind == "l3_meta":
                    self.roi_threshold = payload
                    self.info_var.set(
                        f"L3 entropy gate: retain blocks where entropy ≥ {payload:.2f} bits/pixel"
                    )

                elif kind == "l3_count":
                    self.roi_expected_blocks = payload
                    self.info_var.set(f"L3 selected blocks: {payload}")

                elif kind == "l4_header":
                    block, cols, rows = payload
                    self._render_l4()
                    self.info_var.set(
                        f"L4 progressive stream: {cols}×{rows} blocks, block={block}px, raster order"
                    )

                elif kind == "image_block_complete":
                    mode, col, row, block = payload
                    if mode == "l3":
                        x0, y0, bw, bh = self._block_shape(self.roi_block_size, col, row)
                        self.roi_frame[y0:y0 + bh, x0:x0 + bw] = block
                        self.roi_received_blocks += 1
                        if self.roi_received_blocks % L3_REDRAW_EVERY == 0:
                            self._render_roi()
                    elif mode == "l4":
                        x0, y0, bw, bh = self._block_shape(self.l4_block_size, col, row)
                        self.l4_frame[y0:y0 + bh, x0:x0 + bw] = block
                        self.l4_received_blocks += 1
                        if self.l4_received_blocks % L4_REDRAW_EVERY == 0:
                            self._render_l4()

                elif kind == "frame_complete":
                    mode = self.current_mode
                    frame_bytes = self.rx_bytes_total - self.frame_started_bytes
                    raw_equivalent = RAW_IMAGE_BYTES
                    saving = max(0.0, 100.0 * (1.0 - frame_bytes / raw_equivalent))
                    ratio = raw_equivalent / frame_bytes if frame_bytes else 0.0

                    # Always display the final state after the last packet/block.
                    if mode == "l0" or mode == "l1":
                        self._render_cloud()
                        detail = f"{len(self.cloud_x)} landmarks"
                    elif mode == "l2":
                        self._render_entropy()
                        detail = (
                            f"{self.heatmap_block_count}/{self.heatmap_expected_blocks or '?'} entropy blocks, "
                            f"block={self.block_size}px"
                        )
                    elif mode == "l3":
                        self._render_roi()
                        detail = (
                            f"{self.roi_received_blocks}/{self.roi_expected_blocks or '?'} ROI blocks, "
                            f"block={self.roi_block_size}px"
                        )
                    else:
                        self._render_l4()
                        detail = (
                            f"{self.l4_received_blocks}/{self.l4_expected_blocks or '?'} image blocks, "
                            f"block={self.l4_block_size}px"
                        )

                    self.busy = False
                    self.set_buttons_state(True)
                    self.status_var.set(f"Telemetry frame complete: {MODE_NAMES.get(mode, mode)}")
                    self.info_var.set(
                        f"{MODE_NAMES.get(mode, mode)}: {detail} | "
                        f"wire={frame_bytes} B ({frame_bytes / 1024:.2f} KiB) | "
                        f"saving={saving:.2f}% | raw/wire={ratio:.2f}×"
                    )

        except queue.Empty:
            pass

        self.root.after(50, self._process_events)

    def on_close(self):
        self.stop_event.set()
        self.busy = False
        self.connected = False
        with self.sock_lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except OSError:
                    pass
        plt.close(self.fig)
        self.root.destroy()


def main():
    root = tk.Tk()
    AuraGroundUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
