"""
========================================================================================
AURA Ground Segment Telemetry Decoder & Mission Console
Repository: https://github.com/techn0man1ac/AURA
Author:     Serhii Trush, 2026 (MIT License)
========================================================================================

ARCHITECTURAL SPECIFICATION & PROTOCOL LAYOUT:
----------------------------------------------
All structured telemetry frames arrive big-endian, encapsulated into 32-bit registers 
(uint32) and flushed across the physical or emulated loopback socket byte-by-byte. 
This serial transmission pattern prevents unaligned bus memory faults (SPARC Trap 0x07) 
on radiation-hardened spaceborne processors like the Cobham Gaisler GR712RC (LEON3).

1. L0 / L1 Landmark Subsystem (Sync Markers 0x5B, 0xB1):
   - L0 Layer (Ultra-Sparse Anchor Cloud):
     Transmits 10-bit X and 10-bit Y coordinate pairs packed into contiguous 3-byte
     records (20 bits effective, 4 bits padding). Bypasses score/intensity overhead 
     to provide critical silhouette anchors during razor-thin DSN radio passes (99.98% savings).
   - L1 Layer (Directional Topological Descriptors):
     Transmits 32-bit registers per landmark containing:
     [Bits 27:18] -> 10-bit X Coordinate (0..1019 px).
     [Bits 17:8]  -> 10-bit Y Coordinate (0..1019 px).
     [Bits 7:4]   -> 4-bit Edge-Tangent Vector Orientation (16 sectors, 22.5 deg each).
     [Bits 3:0]   -> 4-bit Quantized Luminance Intensity (0..15).
     Renders as high-contrast anchor dots with directional tangent line segments (whiskers).

2. L2 Shannon Entropy Topography (Sync Markers 0xD0, 0xD1):
   - Streams an adaptive 64x64 block information density map (typically 16x16 pixel tiles).
     Each telemetry word packs: [0xD1] | [Col: 8-bit] | [Row: 8-bit] | [Entropy_x10: 8-bit].
   - Concludes with onboard cross-validation diagnostic benchmarks:
     * MARKER_L2_COMPARE_BLOCKS (0xD7) - Top-10% highest entropy block volume.
     * MARKER_L2_COMPARE_POINTS (0xD8) - Landmark-to-block geometric overlaps.
     * MARKER_L2_COMPARE_HITS   (0xD9) - Physical landmarks residing in top-entropy blocks.

3. L3 / L4 Raster Graphics Engine (Sync Markers 0xD3, 0xC0):
   - L3 Gated Sparse ROI: Progressively reconstructs surface segments isolated via the 
     logical union (A ∪ B) of dilated landmarks and high-entropy regions. Transmitted in 
     linear Bucket-Sorted order (highest entropy first) so science data arrives even if 
     the transmission window terminates abruptly.
   - L4 Full Image Scan: Uncompressed baseline verification raster. Progressively streams 
     paired columns converging from the frame margins inward to the center (Bilateral 
     Opposing Scan), allowing Ground Optical Navigation to stabilize the target silhouette instantly.

4. Closed-Loop Telecommand Array (TTC Uplink):
   - "AURA L0\n" .. "AURA L4\n" -> Commands spacecraft to switch active downlink stream.
   - "AURA GATE <val_x100>\n"   -> Overrides the active onboard Shannon entropy threshold.
========================================================================================
"""

import queue
import socket
import struct
import threading
import time
import csv
import os
from tkinter import ttk, filedialog, messagebox
import tkinter as tk

import numpy as np
import matplotlib
try:
    matplotlib.use("TkAgg")
except ImportError:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.collections import LineCollection
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

# ======================================================================================
# NETWORK & SENSOR CONSTRAINTS
# ======================================================================================
HOST = "127.0.0.1"                     # Localhost TCP port for Renode / QEMU emulator
PORT = 12345                           # UART forwarded loopback socket port
IMG_WIDTH = 1020                       # Sensor matrix width (Hera AFC camera standard)
IMG_HEIGHT = 1020                      # Sensor matrix height
RAW_IMAGE_BYTES = IMG_WIDTH * IMG_HEIGHT  # Uncompressed 8-bit monochrome baseline (1,040,400 B)
CAMERA_EXTENT = (0, IMG_WIDTH, IMG_HEIGHT, 0)
CSV_LOG_FILE = os.path.join("logs", "aura_telemetry_log.csv")

# ======================================================================================
# TELECOMMAND (TTC) STRING SEQUENCES
# ======================================================================================
MODE_COMMANDS = {
    "l0": b"AURA L0\n",
    "l1": b"AURA L1\n",
    "l2": b"AURA L2\n",
    "l3": b"AURA L3\n",
    "l4": b"AURA L4\n",
}

MODE_NAMES = {
    "l0": "L0 — Top-100 Landmark Cloud (X,Y only)",
    "l1": "L1 — Top-1000 Directional Map (X,Y + Tangent Vectors)",
    "l2": "L2 — Full Adaptive Shannon Entropy Map",
    "l3": "L3 — Sparse ROI Image (Union A ∪ B)",
    "l4": "L4 — Full Image / Bilateral Raster Stream",
}

# ======================================================================================
# DOWNLINK HARDWARE PROTOCOL MARKERS (MATCHING Hello_AURA.c)
# ======================================================================================
MARKER_ACK               = 0xAC  # Telecommand acknowledgment token
MARKER_L0_HEADER         = 0x5B  # L0 Ultra-Sparse Landmark Cloud start token
MARKER_L0_POINT          = 0xBD  # L0 Individual coordinate point record
MARKER_L1_HEADER         = 0xB1  # L1 Directional Landmark Map start token
MARKER_L2_HEADER         = 0xD0  # L2 Shannon Entropy Grid configuration header
MARKER_L2_ENTROPY        = 0xD1  # L2 Cell entropy payload packet
MARKER_L2_COUNT          = 0xD2  # L2 Total grid cell count metadata
MARKER_L2_COMPARE_BLOCKS = 0xD7  # Diagnostic: Top-10% entropy block volume
MARKER_L2_COMPARE_POINTS = 0xD8  # Diagnostic: Landmark-to-block geometric overlaps
MARKER_L2_COMPARE_HITS   = 0xD9  # Diagnostic: Physical landmarks inside top entropy
MARKER_L3_HEADER         = 0xD3  # L3 Sparse ROI image transmission header
MARKER_L3_META           = 0xD4  # L3 Active gating threshold and priority metadata
MARKER_L3_BLOCK          = 0xD5  # L3 ROI sub-block pixel packet header
MARKER_L3_COUNT          = 0xD6  # L3 Total transmitted ROI block volume
MARKER_FULL_HEADER       = 0xC0  # L4 Full Raster verification stream header
MARKER_FULL_BLOCK        = 0xC3  # L4 Raster block pixel array header
MARKER_END               = 0xFE  # Frame transmission termination trailer

ACK_NAMES = {
    10: "L0 TOP-100 CLOUD",
    11: "L1 TOP-1000 VECTOR MAP",
    12: "L2 FULL ENTROPY GRID",
    13: "L3 SPARSE ROI FRAME",
    14: "L4 FULL RASTER STREAM",
}


class AuraGroundUI:
    """
    Mission Ground Control Console GUI.
    Manages socket reception, stream parsing, live vector plotting, and TTC commanding.
    """
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("AURA Ground Segment — Real-Time Telemetry & Telecommand Console")
        self.root.geometry("1500x950")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Threading and network attributes
        self.sock = None
        self.sock_lock = threading.Lock()
        self.receiver_thread = None
        self.stop_event = threading.Event()
        self.events = queue.Queue()

        self.connected = False
        self.busy = False
        self.current_mode = None

        # --- Active Landmark Geometry & Vector Structures ---
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_angles = []      # 4-bit edge-tangent orientation angle (0..15 sectors)
        self.cloud_intensity = []   # 4-bit pixel luminance (0..255 scaled)
        self.vector_collection = None  # Matplotlib LineCollection handle for fast vector rendering

        # --- L2 Shannon Entropy Matrices ---
        self.heatmap = np.zeros((64, 64), dtype=np.float32)
        self.heatmap_rows = 64
        self.heatmap_cols = 64
        self.block_size = 16
        self.heatmap_expected_blocks = 0
        self.heatmap_received_blocks = 0
        self.l2_top10_count = 0
        self.l2_overlap_blocks = 0
        self.l2_landmark_hits = 0

        # --- L3/L4 Surface Reconstruction Buffers ---
        self.roi_frame = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
        self.l4_frame = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
        self.roi_block_size = 16
        self.roi_expected_blocks = 0
        self.roi_received_blocks = 0
        self.roi_priority_mode = 0
        self.l4_expected_blocks = 0
        self.l4_received_blocks = 0
        self.roi_threshold = None

        # --- Low-Level Stream Parser State Machine ---
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

        # --- Matplotlib Plot Artist Handles ---
        self.scatter_cloud = None
        self.im_entropy = None
        self.im_image = None
        self.cbar_entropy = None

        self._build_ui()
        self._configure_styles()
        self.root.after(10, self._process_events)
        self.root.after(100, self.toggle_connection)

    def _configure_styles(self):
        self.style = ttk.Style()
        self.style.configure("Connect.TButton", font=("Helvetica", 10, "bold"))
        self.style.configure("Disconnect.TButton", font=("Helvetica", 10, "bold"))

    def _build_ui(self):
        """Constructs layout panels, TTC command buttons, interactive sliders, and plots."""
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="AURA Ground Segment Console", font=("Helvetica", 16, "bold")).pack(side="left", padx=(0, 18))
        self.status_var = tk.StringVar(value="Initializing connection to Renode LEON3 core...")
        ttk.Label(top, textvariable=self.status_var, font=("Helvetica", 10, "italic")).pack(side="left")

        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection, style="Connect.TButton")
        self.connect_button.pack(side="right")

        # Telemetry Level Selection Buttons
        controls = ttk.LabelFrame(self.root, text="Interactive Telemetry Request Panel", padding=10)
        controls.pack(fill="x", padx=10, pady=(0, 5))

        self.buttons = {}
        for mode, label in [("l0", "L0 — Cloud"), ("l1", "L1 — Map (Vectors)"), ("l2", "L2 — Entropy"), ("l3", "L3 — ROI"), ("l4", "L4 — Full")]:
            btn = ttk.Button(controls, text=label, command=lambda m=mode: self.request_mode(m))
            btn.pack(side="left", padx=4, fill="x", expand=True)
            self.buttons[mode] = btn

        # Dynamic Science Gating Array (Uplink Telecommands)
        gate_panel = ttk.LabelFrame(self.root, text="On-Board Science Gate Controller (TTC Command Array)", padding=10)
        gate_panel.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(gate_panel, text="Shannon Entropy Compression Threshold:", font=("Helvetica", 10)).pack(side="left", padx=(5, 10))
        
        self.gate_slider = tk.Scale(gate_panel, from_=0.0, to=8.0, resolution=0.1, orient="horizontal", length=350, showvalue=True, font=("Helvetica", 9))
        self.gate_slider.set(2.5)  # Default baseline threshold
        self.gate_slider.pack(side="left", padx=5)

        self.send_gate_button = ttk.Button(gate_panel, text="⟪ Transmit Gate Command", command=self.send_gate_command)
        self.send_gate_button.pack(side="left", padx=15)
        self.send_gate_button.configure(state="disabled")

        self.info_var = tk.StringVar(value="System Idle. Awaiting connection.")
        ttk.Label(self.root, textvariable=self.info_var, font=("Courier New", 10, "bold"), padding=(10, 0, 10, 8)).pack(fill="x")

        # Multi-Panel Plot Windows
        main_paned = ttk.PanedWindow(self.root, orient="horizontal")
        main_paned.pack(fill="both", expand=True, padx=10, pady=10)

        left_side = ttk.Frame(main_paned)
        right_side = ttk.Frame(main_paned)
        main_paned.add(left_side, weight=1)
        main_paned.add(right_side, weight=1)

        # Top-Left Landmark Cloud Viewport
        self.fig_cloud, self.ax_cloud = plt.subplots(figsize=(5, 4))
        self.canvas_cloud = FigureCanvasTkAgg(self.fig_cloud, master=left_side)
        self.canvas_cloud.get_tk_widget().pack(fill="both", expand=True, side="top", pady=(0, 5))

        # Bottom-Left Shannon Entropy Topography Viewport
        self.fig_entropy, self.ax_entropy = plt.subplots(figsize=(5, 4))
        self.canvas_entropy = FigureCanvasTkAgg(self.fig_entropy, master=left_side)
        self.canvas_entropy.get_tk_widget().pack(fill="both", expand=True, side="bottom")

        # Right-Wing Main Science Frame Stream Viewport
        self.fig_image, self.ax_image = plt.subplots(figsize=(6, 8))
        self.canvas_image = FigureCanvasTkAgg(self.fig_image, master=right_side)
        self.canvas_image.get_tk_widget().pack(fill="both", expand=True)

        self._init_plots()
        self._bind_save_contexts()

    def _init_plots(self):
        """Initializes empty plot styles, aspect ratios, and color palettes."""
        self.ax_cloud.set_facecolor("#0b0c10")
        self.ax_cloud.set_xlim(0, IMG_WIDTH)
        self.ax_cloud.set_ylim(IMG_HEIGHT, 0)
        self.ax_cloud.set_title("AURA — L0/L1 Landmark & Vector View")
        self.ax_cloud.set_aspect("equal", adjustable="box")

        self.im_entropy = self.ax_entropy.imshow(self.heatmap, cmap="jet", interpolation="nearest", vmin=0, vmax=8, extent=CAMERA_EXTENT, origin="upper")
        self.ax_entropy.set_title("AURA — L2 Full Entropy Map")
        self.ax_entropy.set_xlim(0, IMG_WIDTH)
        self.ax_entropy.set_ylim(IMG_HEIGHT, 0)
        self.ax_entropy.set_aspect("equal", adjustable="box")
        
        # Colorbar synchronized inside axes via inset_axes
        cax = inset_axes(self.ax_entropy, width="3%", height="70%", loc="center right", borderpad=-3.5)
        self.cbar_entropy = self.fig_entropy.colorbar(self.im_entropy, cax=cax)
        self.cbar_entropy.set_label("Shannon Entropy (bits/pixel)", rotation=270, labelpad=15)

        self.im_image = self.ax_image.imshow(self.roi_frame, cmap="gray", vmin=0, vmax=255, interpolation="nearest", extent=CAMERA_EXTENT, origin="upper")
        self.ax_image.set_title("AURA — Main Right-Wing Science Frame Stream")
        self.ax_image.set_xlim(0, IMG_WIDTH)
        self.ax_image.set_ylim(IMG_HEIGHT, 0)
        self.ax_image.set_aspect("equal", adjustable="box")

        self.canvas_cloud.draw()
        self.canvas_entropy.draw()
        self.canvas_image.draw()

    def _bind_save_contexts(self):
        """Binds Mouse Button-3 (Right Click) across viewports for native 1020x1020 export."""
        self.canvas_cloud.get_tk_widget().bind("<Button-3>", lambda e: self._trigger_save_menu("cloud"))
        self.canvas_entropy.get_tk_widget().bind("<Button-3>", lambda e: self._trigger_save_menu("entropy"))
        self.canvas_image.get_tk_widget().bind("<Button-3>", lambda e: self._trigger_save_menu("image"))

    def set_buttons_state(self, enabled: bool):
        state = "normal" if enabled and self.connected and not self.busy else "disabled"
        for btn in self.buttons.values():
            btn.configure(state=state)
        self.send_gate_button.configure(state=state)

    def toggle_connection(self):
        """Establishes or disconnects the TCP link to the spacecraft simulator."""
        if self.connected:
            self.stop_event.set()
            with self.sock_lock:
                if self.sock:
                    try: self.sock.close()
                    except OSError: pass
            self.events.put(("disconnected", None))
            return

        self.status_var.set(f"Connecting to LEON3 target at {HOST}:{PORT}...")
        self.connect_button.configure(state="disabled")

        def worker():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)
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
        """Asynchronous background socket listener collecting incoming raw bytes."""
        while not self.stop_event.is_set():
            try:
                chunk = sock.recv(16384)
                if not chunk: break
                self.parser_buffer.extend(chunk)
                self.rx_bytes_total += len(chunk)
                self._parse_buffer()
            except OSError:
                break
        self.events.put(("disconnected", None))

    def _parse_buffer(self):
        """
        State-machine stream deserializer.
        Unpacks big-endian telemetry words into coordinates, 4-bit angles, and raw pixel tiles.
        """
        while True:
            if self.parser_state in ("l0_points", "l1_points", "image_block"):
                if self.raw_expected <= 0:
                    self.parser_state = "packet"
                    continue

                take = min(len(self.parser_buffer), self.raw_expected - len(self.raw_buffer))
                if take <= 0: return

                self.raw_buffer.extend(self.parser_buffer[:take])
                del self.parser_buffer[:take]

                if len(self.raw_buffer) != self.raw_expected: return

                payload = bytes(self.raw_buffer)
                if self.parser_state == "l0_points":
                    local_x = []
                    local_y = []
                    record_size = 3
                    # L0: Sequential 20-bit coordinate unpacking (3 bytes per landmark)
                    for i in range(len(payload) // record_size):
                        base = i * record_size
                        x = (payload[base] << 2) | (payload[base + 1] >> 6)
                        y = ((payload[base + 1] & 0x3F) << 4) | (payload[base + 2] >> 4)
                        local_x.append(x)
                        local_y.append(y)
                    self.events.put(("l0_points_complete", (local_x, local_y)))

                elif self.parser_state == "l1_points":
                    local_x = []
                    local_y = []
                    local_angles = []
                    local_intensities = []
                    record_size = 4
                    # L1: 32-bit register unpacking (X, Y, Edge-Tangent Angle, Intensity)
                    for i in range(len(payload) // record_size):
                        base = i * record_size
                        word = struct.unpack(">I", payload[base:base + record_size])[0]
                        
                        # Bit allocation alignment matching Hello_AURA.c flight target:
                        # [Bits 27:18] -> 10 bits X
                        # [Bits 17:8]  -> 10 bits Y
                        # [Bits 7:4]   -> 4 bits local anisotropic tangent vector (0..15 sectors)
                        # [Bits 3:0]   -> 4 bits intensity
                        x = (word >> 18) & 0x03FF
                        y = (word >> 8) & 0x03FF
                        angle = (word >> 4) & 0x0F
                        intensity = (word & 0x0F) * 17  # Scale 0..15 range to 0..255
                        
                        local_x.append(x)
                        local_y.append(y)
                        local_angles.append(angle)
                        local_intensities.append(intensity)
                    self.events.put(("l1_points_complete", (local_x, local_y, local_angles, local_intensities)))

                else:
                    block = np.frombuffer(payload, dtype=np.uint8).copy().reshape((self.raw_block_h, self.raw_block_w))
                    self.events.put(("image_block_complete", (self.raw_block_mode, self.raw_block_col, self.raw_block_row, block)))

                self.raw_expected = 0
                self.raw_buffer.clear()
                self.parser_state = "packet"
                continue

            if len(self.parser_buffer) < 4: return
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
                self.current_mode = "l0"
                count = (word >> 12) & 0x0FFF
                self.raw_expected = count * 3
                self.parser_state = "l0_points"
                self.events.put(("header_init", "l0"))
                continue

            if marker == MARKER_L1_HEADER:
                del self.parser_buffer[:4]
                self.current_mode = "l1"
                count = (word >> 12) & 0x0FFF
                self.raw_expected = count * 4
                self.parser_state = "l1_points"
                self.events.put(("header_init", "l1"))
                continue

            if marker == MARKER_L2_HEADER:
                del self.parser_buffer[:4]
                self.current_mode = "l2"
                self.block_size = (word >> 16) & 0xFF
                self.heatmap_cols = (word >> 8) & 0xFF
                self.heatmap_rows = word & 0xFF
                self.events.put(("header_init", "l2"))
                continue

            if marker == MARKER_L2_ENTROPY:
                del self.parser_buffer[:4]
                col = (word >> 16) & 0xFF
                row = (word >> 8) & 0xFF
                entropy = (word & 0xFF) * 0.1
                self.events.put(("l2_block", (row, col, entropy)))
                continue

            if marker == MARKER_L2_COUNT:
                del self.parser_buffer[:4]
                self.heatmap_expected_blocks = word & 0xFFFF
                continue

            if marker == MARKER_L2_COMPARE_BLOCKS:
                del self.parser_buffer[:4]
                self.l2_top10_count = word & 0xFFFF
                continue

            if marker == MARKER_L2_COMPARE_POINTS:
                del self.parser_buffer[:4]
                self.l2_overlap_blocks = word & 0xFFFF
                continue

            if marker == MARKER_L2_COMPARE_HITS:
                del self.parser_buffer[:4]
                self.l2_landmark_hits = word & 0xFFFF
                self.events.put(("l2_compare", None))
                continue

            if marker == MARKER_L3_HEADER:
                del self.parser_buffer[:4]
                self.current_mode = "l3"
                self.roi_block_size = (word >> 16) & 0xFF
                self.roi_cols = (word >> 8) & 0xFF
                self.roi_rows = word & 0xFF
                self.roi_received_blocks = 0
                self.roi_frame.fill(0)
                self.events.put(("header_init", "l3"))
                continue

            if marker == MARKER_L3_META:
                del self.parser_buffer[:4]
                self.roi_priority_mode = (word >> 16) & 0xFF
                self.roi_threshold = (word & 0xFFFF) * 0.01
                continue

            if marker == MARKER_L3_COUNT:
                del self.parser_buffer[:4]
                self.roi_expected_blocks = word & 0xFFFF
                continue

            if marker == MARKER_L3_BLOCK:
                del self.parser_buffer[:4]
                col = (word >> 16) & 0xFF
                row = (word >> 8) & 0xFF
                x0 = col * self.roi_block_size
                y0 = row * self.roi_block_size
                bw = min(self.roi_block_size, IMG_WIDTH - x0)
                bh = min(self.roi_block_size, IMG_HEIGHT - y0)
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
                self.current_mode = "l4"
                self.l4_block_size = (word >> 16) & 0xFF
                self.l4_cols = (word >> 8) & 0xFF
                self.l4_rows = word & 0xFF
                self.l4_received_blocks = 0
                self.l4_expected_blocks = self.l4_rows * self.l4_cols
                self.l4_frame.fill(0)
                self.events.put(("header_init", "l4"))
                continue

            if marker == MARKER_FULL_BLOCK:
                del self.parser_buffer[:4]
                col = (word >> 16) & 0xFF
                row = (word >> 8) & 0xFF
                x0 = col * self.l4_block_size
                y0 = row * self.l4_block_size
                bw = min(self.l4_block_size, IMG_WIDTH - x0)
                bh = min(self.l4_block_size, IMG_HEIGHT - y0)
                self.raw_block_mode = "l4"
                self.raw_block_col = col
                self.raw_block_row = row
                self.raw_block_w = bw
                self.raw_block_h = bh
                self.raw_expected = bw * bh
                self.raw_buffer.clear()
                self.parser_state = "image_block"
                continue

            del self.parser_buffer[0]

    def request_mode(self, mode: str):
        """Transmits an uplink telecommand switching the spacecraft's telemetry level."""
        if not self.connected or self.busy: return
        self.frame_started_bytes = self.rx_bytes_total
        with self.sock_lock:
            try: self.sock.sendall(MODE_COMMANDS[mode])
            except OSError as exc:
                self.status_var.set(f"Tx Error: {exc}")
                return

        self.busy = True
        self.current_mode = mode
        self.info_var.set(f"Request sent: {MODE_NAMES[mode]} — downlinking architecture...")
        self.set_buttons_state(False)

    def send_gate_command(self):
        """Uplinks a parameterized telecommand (AURA GATE <val>) to adjust onboard entropy gating."""
        if not self.connected: return
        gate_val_x100 = int(round(self.gate_slider.get() * 100))
        command = f"AURA GATE {gate_val_x100}\n".encode('ascii')
        
        with self.sock_lock:
            try:
                self.sock.sendall(command)
                self.info_var.set(f"TTC Telecommand Uplinked: Setting On-Board Entropy Gate to {self.gate_slider.get():.2f} bits/px")
            except OSError as exc:
                self.status_var.set(f"Tx Error: {exc}")

    def _draw_l0_live(self):
        """Renders Level 0: Pure (X, Y) coordinate points cloud on black space background."""
        if self.scatter_cloud:
            try: self.scatter_cloud.remove()
            except Exception: pass
            self.scatter_cloud = None

        if self.vector_collection:
            try: self.vector_collection.remove()
            except Exception: pass
            self.vector_collection = None

        if self.cloud_x and self.cloud_y:
            # s=5 makes the points look like neat, sharp spikes instead of blurry spheres
            self.scatter_cloud = self.ax_cloud.scatter(
                self.cloud_x, self.cloud_y, c="#ffffff", s=5, marker="o", zorder=3
            )
        self.ax_cloud.set_title(f"AURA — L0 Coordinate Cloud ({len(self.cloud_x)} features)")
        self.canvas_cloud.draw_idle()

    def _draw_l1_live(self):
        """
        Renders Level 1: Topological Feature Map.
        Draws crisp pinpoint anchor dots with high-contrast tangent whiskers.
        """
        if self.scatter_cloud:
            try: self.scatter_cloud.remove()
            except Exception: pass
            self.scatter_cloud = None

        if self.vector_collection:
            try: self.vector_collection.remove()
            except Exception: pass
            self.vector_collection = None

        if self.cloud_x and self.cloud_y:
            # 1. An antenna length of 24.0 in the coordinate system corresponds to exactly ~8 pixels in the UI widget
            vec_len = 24.0
            segments = []
            for x, y, ang in zip(self.cloud_x, self.cloud_y, self.cloud_angles):
                rad = ang * (22.5 * np.pi / 180.0)
                x2 = x + vec_len * np.cos(rad)
                y2 = y + vec_len * np.sin(rad)
                segments.append([(x, y), (x2, y2)])

            # 2. Draw bright neon-yellow tangent lines
            self.vector_collection = LineCollection(
                segments, colors="#ffe600", linewidths=1.5, zorder=2
            )
            self.ax_cloud.add_collection(self.vector_collection)

            # 3. Small, compact red center points (s=4 does not overlap the line)
            self.scatter_cloud = self.ax_cloud.scatter(
                self.cloud_x, self.cloud_y, c="#ff2233", s=4, zorder=3
            )

        self.ax_cloud.set_title(f"AURA — L1 Landmarks & Vectors ({len(self.cloud_x)} points)")
        self.canvas_cloud.draw_idle()

    def _draw_l2_live(self):
        """High-frequency blitting update of the 64x64 Shannon entropy heatmap."""
        self.im_entropy.set_data(self.heatmap)
        self.ax_entropy.set_title(f"AURA — L2 Adaptive Grid ({self.heatmap_received_blocks} blocks)")
        
        self.ax_entropy.draw_artist(self.im_entropy)
        self.fig_entropy.canvas.blit(self.ax_entropy.bbox)

    def _draw_l3_live(self):
        """Progressive visual reconstruction of the gated Sparse ROI frame."""
        self.im_image.set_data(self.roi_frame)
        priority_name = "entropy" if getattr(self, "roi_priority_mode", 0) == 0 else "brightness"
        self.ax_image.set_title(f"AURA — L3 ROI Gated Reconstruction ({self.roi_received_blocks} blocks, {priority_name} priority)")
        self.canvas_image.draw_idle()

    def _draw_l4_live(self):
        """Visual rendering of the uncompressed progressive bilateral vertical scan."""
        self.im_image.set_data(self.l4_frame)
        self.ax_image.set_title(f"AURA — L4 Full Progressive Raster ({self.l4_received_blocks} blocks)")
        self.canvas_image.draw_idle()

    def _trigger_save_menu(self, channel_type):
        """Native 1020x1020 high-fidelity export without borders or UI decorations."""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG Image", "*.png"), ("JPEG Image", "*.jpg;*.jpeg")],
            title=f"Export {channel_type.upper()} Data Channel (1020×1020)"
        )
        if not file_path:
            return

        try:
            from PIL import Image

            ext = os.path.splitext(file_path)[1].lower()
            if channel_type == "image":
                frame_data = self.roi_frame if self.current_mode == "l3" else self.l4_frame
                image = Image.fromarray(frame_data.astype(np.uint8), mode="L")
                if ext in (".jpg", ".jpeg"):
                    image.save(file_path, format="JPEG", quality=95)
                else:
                    image.save(file_path, format="PNG")
                return

            export_fig = plt.figure(figsize=(10.2, 10.2), dpi=100, facecolor="#0b0c10")
            export_ax = export_fig.add_axes([0, 0, 1, 1], facecolor="#0b0c10")
            export_ax.set_xlim(0, IMG_WIDTH)
            export_ax.set_ylim(IMG_HEIGHT, 0)
            export_ax.axis("off")

            if channel_type == "cloud":
                if self.current_mode == "l0":
                    export_ax.scatter(self.cloud_x, self.cloud_y, c="white", s=25, marker="o")
                else:
                    # Export L1 with matching tangent whiskers and landmark centers
                    vec_len = 14.0
                    segments = []
                    for x, y, ang in zip(self.cloud_x, self.cloud_y, self.cloud_angles):
                        rad = ang * (22.5 * np.pi / 180.0)
                        x2 = x + vec_len * np.cos(rad)
                        y2 = y + vec_len * np.sin(rad)
                        segments.append([(x, y), (x2, y2)])
                    
                    lc = LineCollection(segments, colors="#ffd700", linewidths=1.5, zorder=2)
                    export_ax.add_collection(lc)
                    export_ax.scatter(self.cloud_x, self.cloud_y, c="#ff3344", s=14, zorder=3)

            elif channel_type == "entropy":
                export_ax.imshow(
                    self.heatmap,
                    cmap="jet",
                    interpolation="nearest",
                    vmin=0,
                    vmax=8,
                    extent=CAMERA_EXTENT,
                    origin="upper",
                    aspect="auto",
                )

            export_fig.savefig(file_path, format="png", dpi=100, pad_inches=0, facecolor=export_fig.get_facecolor(), edgecolor='none')
            plt.close(export_fig)
        except Exception as exc:
            messagebox.showerror("AURA export error", f"Could not save {file_path}: {exc}")

    def _log_to_csv(self, current_mode_str, wire_bytes, saving_pct, ratio):
        """Appends verified wire transmission metrics into the CSV mission audit log."""
        os.makedirs(os.path.dirname(CSV_LOG_FILE), exist_ok=True)
        file_exists = os.path.isfile(CSV_LOG_FILE)
        try:
            with open(CSV_LOG_FILE, mode="a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow([
                        "Timestamp", 
                        "Telemetry_Level", 
                        "Entropy_Gate_Threshold", 
                        "Transmitted_Bytes_Wire", 
                        "Bandwidth_Savings_Pct", 
                        "Compression_Ratio_Factor", 
                        "Detail_Metrics"
                    ])
                
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                if current_mode_str in ("l0", "l1"):
                    detail = f"{len(self.cloud_x)} landmarks"
                elif current_mode_str == "l2":
                    detail = f"{self.heatmap_received_blocks} entropy blocks"
                elif current_mode_str == "l3":
                    detail = f"{self.roi_received_blocks}/{self.roi_expected_blocks} ROI blocks"
                else:
                    detail = f"{self.l4_received_blocks} sequential blocks"

                current_gate_val = f"{self.gate_slider.get():.2f}"

                writer.writerow([
                    timestamp, 
                    current_mode_str.upper(), 
                    current_gate_val, 
                    wire_bytes, 
                    f"{saving_pct:.2f}%", 
                    f"{ratio:.2f}x", 
                    detail
                ])
        except Exception as e:
            print(f"Mission Log Write Failure: {e}")

    def _process_events(self):
        """Main UI event loop dispatcher draining the queue without blocking."""
        try:
            burst_counter = 0
            while burst_counter < 2000:
                try:
                    kind, payload = self.events.get_nowait()
                    burst_counter += 1
                except queue.Empty:
                    break

                if kind == "connected":
                    self.sock = payload
                    self.connected = True
                    self.busy = False
                    self.status_var.set(f"Active link to LEON3 simulator at {HOST}:{PORT}")
                    self.info_var.set("Link established. Select AURA telemetry level.")
                    self.connect_button.configure(text="Disconnect", style="Disconnect.TButton", state="normal")
                    self.set_buttons_state(True)

                elif kind in ("error", "disconnected"):
                    self.connected = False
                    self.busy = False
                    self.sock = None
                    self.status_var.set("Link inactive.")
                    self.connect_button.configure(text="Connect", style="Connect.TButton", state="normal")
                    self.set_buttons_state(False)

                elif kind == "ack":
                    if payload == 255:
                        self.status_var.set("LEON3 Dynamic Registry Update: Entropy Gate Locked Successfully!")
                    else:
                        self.status_var.set(f"LEON3 Target ACK: Hardware Mode {ACK_NAMES.get(payload, payload)} active")

                elif kind == "header_init":
                    self.info_var.set(f"Streaming package for channel {payload.upper()} incoming...")
                    if payload in ("l0", "l1"):
                        self.cloud_x, self.cloud_y, self.cloud_angles, self.cloud_intensity = [], [], [], []
                        if self.vector_collection:
                            try: self.vector_collection.remove()
                            except Exception: pass
                            self.vector_collection = None
                        if self.scatter_cloud:
                            try: self.scatter_cloud.remove()
                            except Exception: pass
                            self.scatter_cloud = None
                        self.ax_cloud.set_facecolor("#0b0c10")
                        self.ax_cloud.set_xlim(0, IMG_WIDTH)
                        self.ax_cloud.set_ylim(IMG_HEIGHT, 0)
                        self.canvas_cloud.draw_idle()
                    elif payload == "l2":
                        self.heatmap = np.zeros((self.heatmap_rows, self.heatmap_cols), dtype=np.float32)
                        self.heatmap_received_blocks = 0
                        self._draw_l2_live()

                elif kind == "l0_points_complete":
                    self.cloud_x, self.cloud_y = payload
                    self._draw_l0_live()

                elif kind == "l1_points_complete":
                    self.cloud_x, self.cloud_y, self.cloud_angles, self.cloud_intensity = payload
                    self._draw_l1_live()

                elif kind == "l2_block":
                    r, c, val = payload
                    if r < self.heatmap_rows and c < self.heatmap_cols:
                        self.heatmap[r, c] = val
                        self.heatmap_received_blocks += 1
                        if self.heatmap_received_blocks % 16 == 0: 
                            self._draw_l2_live()

                elif kind == "l2_compare":
                    self.status_var.set(
                        f"L2 comparison: Top-10% entropy blocks={self.l2_top10_count} | "
                        f"landmark-block overlap={self.l2_overlap_blocks} | "
                        f"landmarks inside Top-10%={self.l2_landmark_hits}/{len(self.cloud_x) or 1000}"
                    )

                elif kind == "image_block_complete":
                    mode, col, row, block = payload
                    if mode == "l3":
                        x0 = col * self.roi_block_size
                        y0 = row * self.roi_block_size
                        bh, bw = block.shape
                        self.roi_frame[y0:y0+bh, x0:x0+bw] = block
                        self.roi_received_blocks += 1
                        self._draw_l3_live()
                    elif mode == "l4":
                        x0 = col * self.l4_block_size
                        y0 = row * self.l4_block_size
                        bh, bw = block.shape
                        self.l4_frame[y0:y0+bh, x0:x0+bw] = block
                        self.l4_received_blocks += 1
                        self._draw_l4_live()

                elif kind == "frame_complete":
                    self.busy = False
                    self.set_buttons_state(True)
                    frame_bytes = self.rx_bytes_total - self.frame_started_bytes
                    saving = max(0.0, 100.0 * (1.0 - frame_bytes / RAW_IMAGE_BYTES))
                    ratio = RAW_IMAGE_BYTES / frame_bytes if frame_bytes else 0.0

                    if self.current_mode == "l0": self._draw_l0_live()
                    elif self.current_mode == "l1": self._draw_l1_live()
                    elif self.current_mode == "l2": 
                        self.im_entropy.set_data(self.heatmap)
                        self.ax_entropy.set_title(f"AURA — L2 Adaptive Grid ({self.heatmap_received_blocks} blocks)")
                        self.canvas_entropy.draw_idle()
                    elif self.current_mode == "l3": self._draw_l3_live()
                    elif self.current_mode == "l4": self._draw_l4_live()

                    priority_detail = ""
                    if self.current_mode == "l3":
                        priority_detail = " | priority=entropy" if self.roi_priority_mode == 0 else " | priority=brightness"
                    self.info_var.set(
                        f"Downlink Finished | wire={frame_bytes} B ({frame_bytes / 1024:.2f} KiB) | "
                        f"bandwidth_saved={saving:.2f}% | raw/wire={ratio:.2f}×{priority_detail}"
                    )
                    
                    self._log_to_csv(self.current_mode, frame_bytes, saving, ratio)

        except Exception as e:
            print(f"UI Thread Recovery Exception: {e}")

        self.root.after(10, self._process_events)

    def on_close(self):
        self.stop_event.set()
        with self.sock_lock:
            if self.sock:
                try: self.sock.close()
                except OSError: pass
        plt.close('all')
        self.root.destroy()


def main():
    root = tk.Tk()
    AuraGroundUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
