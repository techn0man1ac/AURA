import queue
import socket
import struct
import threading
import time
import csv
import os
from tkinter import ttk, filedialog
import tkinter as tk

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
CSV_LOG_FILE = os.path.join("logs", "aura_telemetry_log.csv")

MODE_COMMANDS = {
    "l0": b"AURA L0\n",
    "l1": b"AURA L1\n",
    "l2": b"AURA L2\n",
    "l3": b"AURA L3\n",
    "l4": b"AURA L4\n",
}

MODE_NAMES = {
    "l0": "L0 — Top-100 Landmark Cloud (X,Y only)",
    "l1": "L1 — Top-1000 Landmark Map (X,Y + intensity)",
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
MARKER_L2_COMPARE_BLOCKS = 0xD7
MARKER_L2_COMPARE_POINTS = 0xD8
MARKER_L2_COMPARE_HITS = 0xD9
MARKER_L3_HEADER = 0xD3
MARKER_L3_META = 0xD4
MARKER_L3_BLOCK = 0xD5
MARKER_L3_COUNT = 0xD6
MARKER_FULL_HEADER = 0xC0
MARKER_FULL_BLOCK = 0xC3
MARKER_END = 0xFE

ACK_NAMES = {
    10: "L0 TOP-100",
    11: "L1 TOP-1000 (X,Y + intensity)",
    12: "L2 FULL ENTROPY",
    13: "L3 SPARSE ROI",
    14: "L4 FULL FRAME STREAM",
}

class AuraGroundUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("AURA V0.3 — Ground Segment / Information-First Telemetry Console")
        self.root.geometry("1500x950")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.receiver_thread = None
        self.stop_event = threading.Event()
        self.events = queue.Queue()

        self.connected = False
        self.busy = False
        self.current_mode = None

        # Стан каналів зв'язку
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_score = []
        self.cloud_is_scored = True

        self.heatmap = np.zeros((64, 64), dtype=np.float32)
        self.heatmap_rows = 64
        self.heatmap_cols = 64
        self.block_size = 16
        self.heatmap_expected_blocks = 0
        self.heatmap_received_blocks = 0
        self.l2_top10_count = 0
        self.l2_overlap_blocks = 0
        self.l2_landmark_hits = 0

        self.roi_frame = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
        self.l4_frame = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
        self.roi_block_size = 16
        self.roi_expected_blocks = 0
        self.roi_received_blocks = 0
        self.roi_priority_mode = 0
        self.l4_expected_blocks = 0
        self.l4_received_blocks = 0
        self.roi_threshold = None
        self.roi_priority_mode = 0

        # Низькорівневий парсер
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

        # Посилання на графічні об'єкти Matplotlib
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
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="AURA Ground Segment Console V0.3", font=("Helvetica", 16, "bold")).pack(side="left", padx=(0, 18))
        self.status_var = tk.StringVar(value="Initializing connection to Renode LEON3 core...")
        ttk.Label(top, textvariable=self.status_var, font=("Helvetica", 10, "italic")).pack(side="left")

        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection, style="Connect.TButton")
        self.connect_button.pack(side="right")

        # Основна панель команд
        controls = ttk.LabelFrame(self.root, text="Interactive Telemetry Request Panel", padding=10)
        controls.pack(fill="x", padx=10, pady=(0, 5))

        self.buttons = {}
        for mode, label in [("l0", "L0 — Cloud"), ("l1", "L1 — Map"), ("l2", "L2 — Entropy"), ("l3", "L3 — ROI"), ("l4", "L4 — Full")]:
            btn = ttk.Button(controls, text=label, command=lambda m=mode: self.request_mode(m))
            btn.pack(side="left", padx=4, fill="x", expand=True)
            self.buttons[mode] = btn

        # НОВА ПАНЕЛЬ: Динамічне керування порогом компресії (TTC Entropy Gate)
        gate_panel = ttk.LabelFrame(self.root, text="On-Board Science Gate Controller (TTC Command Array)", padding=10)
        gate_panel.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(gate_panel, text="Shannon Entropy Compression Threshold:", font=("Helvetica", 10)).pack(side="left", padx=(5, 10))
        
        self.gate_slider = tk.Scale(gate_panel, from_=0.0, to=8.0, resolution=0.1, orient="horizontal", length=350, showvalue=True, font=("Helvetica", 9))
        self.gate_slider.set(2.5) # Значення за замовчуванням
        self.gate_slider.pack(side="left", padx=5)

        self.send_gate_button = ttk.Button(gate_panel, text="⟪ Transmit Gate Command", command=self.send_gate_command)
        self.send_gate_button.pack(side="left", padx=15)
        self.send_gate_button.configure(state="disabled")

        self.info_var = tk.StringVar(value="System Idle. Awaiting connection.")
        ttk.Label(self.root, textvariable=self.info_var, font=("Courier New", 10, "bold"), padding=(10, 0, 10, 8)).pack(fill="x")

        main_paned = ttk.PanedWindow(self.root, orient="horizontal")
        main_paned.pack(fill="both", expand=True, padx=10, pady=10)

        left_side = ttk.Frame(main_paned)
        right_side = ttk.Frame(main_paned)
        main_paned.add(left_side, weight=1)
        main_paned.add(right_side, weight=1)

        self.fig_cloud, self.ax_cloud = plt.subplots(figsize=(5, 4))
        self.canvas_cloud = FigureCanvasTkAgg(self.fig_cloud, master=left_side)
        self.canvas_cloud.get_tk_widget().pack(fill="both", expand=True, side="top", pady=(0, 5))

        self.fig_entropy, self.ax_entropy = plt.subplots(figsize=(5, 4))
        self.canvas_entropy = FigureCanvasTkAgg(self.fig_entropy, master=left_side)
        self.canvas_entropy.get_tk_widget().pack(fill="both", expand=True, side="bottom")

        self.fig_image, self.ax_image = plt.subplots(figsize=(6, 8))
        self.canvas_image = FigureCanvasTkAgg(self.fig_image, master=right_side)
        self.canvas_image.get_tk_widget().pack(fill="both", expand=True)

        self._init_plots()
        self._bind_save_contexts()

    def _init_plots(self):
        # Ініціалізація L0/L1 (Top Left)
        self.ax_cloud.set_facecolor("#0b0c10")
        self.ax_cloud.set_xlim(0, IMG_WIDTH)
        self.ax_cloud.set_ylim(IMG_HEIGHT, 0)
        self.ax_cloud.set_title("AURA — L0/L1 Landmark Cloud View")
        self.ax_cloud.set_aspect("equal", adjustable="box")

        # Ініціалізація L2 Entropy (Bottom Left)
        self.im_entropy = self.ax_entropy.imshow(self.heatmap, cmap="jet", interpolation="nearest", vmin=0, vmax=8, extent=CAMERA_EXTENT, origin="upper")
        self.ax_entropy.set_title("AURA — L2 Full Entropy Map")
        self.ax_entropy.set_xlim(0, IMG_WIDTH)
        self.ax_entropy.set_ylim(IMG_HEIGHT, 0)
        self.ax_entropy.set_aspect("equal", adjustable="box")
        
        # ФІКС ЗМІЩЕННЯ: Створюємо внутрішню вісь для colorbar без зсуву осей самого графіка
        cax = inset_axes(self.ax_entropy, width="3%", height="70%", loc="center right", borderpad=-3.5)
        self.cbar_entropy = self.fig_entropy.colorbar(self.im_entropy, cax=cax)
        self.cbar_entropy.set_label("Shannon Entropy (bits/pixel)", rotation=270, labelpad=15)

        # Ініціалізація L3/L4 Image Display (Right Wing)
        self.im_image = self.ax_image.imshow(self.roi_frame, cmap="gray", vmin=0, vmax=255, interpolation="nearest", extent=CAMERA_EXTENT, origin="upper")
        self.ax_image.set_title("AURA — Main Right-Wing Science Frame Stream")
        self.ax_image.set_xlim(0, IMG_WIDTH)
        self.ax_image.set_ylim(IMG_HEIGHT, 0)
        self.ax_image.set_aspect("equal", adjustable="box")

        self.canvas_cloud.draw()
        self.canvas_entropy.draw()
        self.canvas_image.draw()

    def _bind_save_contexts(self):
        # Прив'язка натискання правої кнопки миші (Button-3) для контекстного експорту
        self.canvas_cloud.get_tk_widget().bind("<Button-3>", lambda e: self._trigger_save_menu("cloud"))
        self.canvas_entropy.get_tk_widget().bind("<Button-3>", lambda e: self._trigger_save_menu("entropy"))
        self.canvas_image.get_tk_widget().bind("<Button-3>", lambda e: self._trigger_save_menu("image"))

    def set_buttons_state(self, enabled: bool):
        state = "normal" if enabled and self.connected and not self.busy else "disabled"
        for btn in self.buttons.values():
            btn.configure(state=state)

    def toggle_connection(self):
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
                if self.parser_state in ("l0_points", "l1_points"):
                    self.cloud_x, self.cloud_y, self.cloud_score = [], [], []

                    if self.parser_state == "l0_points":
                        record_size = 3
                        for i in range(len(payload) // record_size):
                            base = i * record_size
                            x = (payload[base] << 2) | (payload[base + 1] >> 6)
                            y = ((payload[base + 1] & 0x3F) << 4) | (payload[base + 2] >> 4)
                            self.cloud_x.append(x)
                            self.cloud_y.append(y)
                            self.cloud_score.append(0)
                        self.events.put(("l0_points_complete", None))
                    else:
                        # ФІКС L1: reserved[4] | X[10] | Y[10] | vector_angle[4] | intensity[4]
                        record_size = 4
                        for i in range(len(payload) // record_size):
                            base = i * record_size
                            word = struct.unpack(">I", payload[base:base + record_size])[0]
                            x = (word >> 18) & 0x03FF
                            y = (word >> 8) & 0x03FF
                            angle = (word >> 4) & 0x0F  # Дістаємо вектор напрямку рельєфу
                            self.cloud_x.append(x)
                            self.cloud_y.append(y)
                            self.cloud_score.append(angle * 17) # Масштабуємо 0..15 до 0..255 під палітру
                        self.events.put(("l1_points_complete", None))
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
                self.cloud_x, self.cloud_y, self.cloud_score = [], [], []
                count = (word >> 12) & 0x0FFF
                self.raw_expected = count * 3
                self.parser_state = "l0_points"
                self.events.put(("header_init", "l0"))
                continue

            if marker == MARKER_L1_HEADER:
                del self.parser_buffer[:4]
                self.current_mode = "l1"
                self.cloud_x, self.cloud_y, self.cloud_score = [], [], []
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
                self.heatmap = np.zeros((self.heatmap_rows, self.heatmap_cols), dtype=np.float32)
                self.heatmap_received_blocks = 0
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
        if not self.connected: return
        # Переводимо float (наприклад 2.5) у ціле число x100 (250) для фіксованої точки LEON3
        gate_val_x100 = int(round(self.gate_slider.get() * 100))
        command = f"AURA GATE {gate_val_x100}\n".encode('ascii')
        
        with self.sock_lock:
            try:
                self.sock.sendall(command)
                self.info_var.set(f"TTC Telecommand Uplinked: Setting On-Board Entropy Gate to {self.gate_slider.get():.2f} bits/px")
            except OSError as exc:
                self.status_var.set(f"Tx Error: {exc}")

    def set_buttons_state(self, enabled: bool):
        state = "normal" if enabled and self.connected and not self.busy else "disabled"
        for btn in self.buttons.values():
            btn.configure(state=state)
        # Керуємо доступністю кнопки відправки команди динамічного порогу
        self.send_gate_button.configure(state=state)

    def _draw_l0_live(self):
        if self.scatter_cloud: self.scatter_cloud.remove()
        self.scatter_cloud = self.ax_cloud.scatter(self.cloud_x, self.cloud_y, c="white", s=22, marker="+")
        self.ax_cloud.set_title(f"AURA — L0 Coordinate-Only Cloud ({len(self.cloud_x)} features)")
        self.canvas_cloud.draw_idle()

    def _draw_l1_live(self):
        if self.scatter_cloud: self.scatter_cloud.remove()
        self.scatter_cloud = self.ax_cloud.scatter(
            self.cloud_x, self.cloud_y, c=self.cloud_score, cmap="plasma", vmin=0, vmax=255, s=18, marker="+"
        )
        self.ax_cloud.set_title(f"AURA — L1 Landmark + Intensity Stream ({len(self.cloud_x)} points)")
        self.canvas_cloud.draw_idle()

    def _draw_l2_live(self):
        self.im_entropy.set_data(self.heatmap)
        self.ax_entropy.set_title(f"AURA — L2 Adaptive Grid ({self.heatmap_received_blocks} blocks)")
        self.canvas_entropy.draw_idle()

    def _draw_l3_live(self):
        self.im_image.set_data(self.roi_frame)
        priority_name = "entropy" if getattr(self, "roi_priority_mode", 0) == 0 else "brightness"
        self.ax_image.set_title(f"AURA — L3 ROI Gated Reconstruction ({self.roi_received_blocks} blocks, {priority_name} priority)")
        self.canvas_image.draw_idle()

    def _draw_l4_live(self):
        self.im_image.set_data(self.l4_frame)
        self.ax_image.set_title(f"AURA — L4 Full Progressive Raster ({self.l4_received_blocks} blocks)")
        self.canvas_image.draw_idle()

    def _trigger_save_menu(self, channel_type):
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

            export_fig = plt.figure(figsize=(10.2, 10.2), dpi=100)
            export_ax = export_fig.add_axes([0, 0, 1, 1])
            export_ax.set_xlim(0, IMG_WIDTH)
            export_ax.set_ylim(IMG_HEIGHT, 0)
            export_ax.axis("off")

            if channel_type == "cloud":
                export_ax.set_facecolor("#0b0c10")
                if self.current_mode == "l0":
                    export_ax.scatter(self.cloud_x, self.cloud_y, c="white", s=24, marker="+")
                else:
                    export_ax.scatter(self.cloud_x, self.cloud_y, c="cyan", s=20, marker="+")
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

            export_fig.savefig(file_path, format="png", dpi=100, pad_inches=0)
            plt.close(export_fig)
        except Exception as exc:
            messagebox.showerror("AURA export error", f"Could not save {file_path}: {exc}")

    def _log_to_csv(self, current_mode_str, wire_bytes, saving_pct, ratio):
        # Автоматичне створення каталогу logs за потреби
        os.makedirs(os.path.dirname(CSV_LOG_FILE), exist_ok=True)
        file_exists = os.path.isfile(CSV_LOG_FILE)
        try:
            with open(CSV_LOG_FILE, mode="a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                if not file_exists:
                    # ФІКС: Додано колонку Entropy_Gate_Threshold строго коло поля Telemetry_Level
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
                if current_mode_str == "l0" or current_mode_str == "l1":
                    detail = f"{len(self.cloud_x)} landmarks"
                elif current_mode_str == "l2":
                    detail = f"{self.heatmap_received_blocks} entropy blocks"
                elif current_mode_str == "l3":
                    detail = f"{self.roi_received_blocks}/{self.roi_expected_blocks} ROI blocks"
                else:
                    detail = f"{self.l4_received_blocks} sequential blocks"

                # Витягуємо поточне значення порогу зі слайдера Ground UI
                current_gate_val = f"{self.gate_slider.get():.2f}"

                # Запис повного інформаційного рядка телеметрії місії на диск
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
                    if payload == "l0" or payload == "l1":
                        self.ax_cloud.clear()
                        self.ax_cloud.set_facecolor("#0b0c10")
                        self.ax_cloud.set_xlim(0, IMG_WIDTH)
                        self.ax_cloud.set_ylim(IMG_HEIGHT, 0)
                        self.scatter_cloud = None
                    elif payload == "l2":
                        self.heatmap = np.zeros((self.heatmap_rows, self.heatmap_cols), dtype=np.float32)
                        self._draw_l2_live()

                elif kind == "l0_point":
                    x, y = payload
                    self.cloud_x.append(x)
                    self.cloud_y.append(y)
                    self.cloud_score.append(0)

                elif kind == "l0_points_complete":
                    self._draw_l0_live()

                elif kind == "l1_points_complete":
                    self._draw_l1_live()

                elif kind == "l2_block":
                    r, c, val = payload
                    if r < self.heatmap_rows and c < self.heatmap_cols:
                        self.heatmap[r, c] = val
                        self.heatmap_received_blocks += 1
                        if self.heatmap_received_blocks % 4 == 0: self._draw_l2_live()

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
                    elif self.current_mode == "l2": self._draw_l2_live()
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
