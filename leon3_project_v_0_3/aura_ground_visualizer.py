import queue
import socket
import struct
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

HOST = "127.0.0.1"
PORT = 12345
IMG_WIDTH = 1020
IMG_HEIGHT = 1020
RAW_IMAGE_BYTES = IMG_WIDTH * IMG_HEIGHT

MODE_COMMANDS = {
    "cloud": b"AURA CLOUD\n",
    "entropy": b"AURA ENTROPY\n",
    "cut": b"AURA CUT\n",
    "full": b"AURA FULL\n",
}

MODE_NAMES = {
    "cloud": "Landmark Cloud",
    "entropy": "Entropy Map",
    "cut": "Zero-Low-Entropy Cut Image",
    "full": "Full Raw Image",
}

MARKER_ACK = 0xAC
MARKER_CLOUD_HEADER = 0x5B
MARKER_CLOUD_POINT = 0xBD
MARKER_ENTROPY_HEADER = 0x5A
MARKER_ENTROPY_BLOCK = 0xA5
MARKER_FULL_HEADER = 0xC0
MARKER_CUT_HEADER = 0xC1
MARKER_CUT_META = 0xC2
MARKER_END = 0xFE

class AuraGroundUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("AURA — Ground Segment / Interactive Telemetry Console")
        self.root.geometry("1250x900")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.receiver_thread = None
        self.stop_event = threading.Event()
        self.events = queue.Queue()
        self.busy = False
        self.connected = False

        self.current_mode = None
        self.current_frame = None
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_score = []
        self.heatmap = None
        self.heatmap_rows = 0
        self.heatmap_cols = 0
        self.block_size = 0
        self.cut_threshold = None
        
        # Потоковий локальний облік телеметричних байт
        self.rx_bytes_total = 0
        self.frame_started_bytes = 0
        
        self.raw_expected = 0
        self.raw_buffer = bytearray()
        self.parser_buffer = bytearray()
        self.parser_state = "packet"
        self.points_since_draw = 0
        self.blocks_since_draw = 0
        
        # Об'єкт колірної шкали для запобігання бага нашарування
        self.cbar = None

        self._build_ui()
        self._configure_styles()
        self.root.after(50, self._process_events)
        self.root.after(100, self.toggle_connection)

    def _configure_styles(self):
        self.style = ttk.Style()
        self.style.configure("Connect.TButton", foreground="white", background="#28a745", font=("Helvetica", 10, "bold"))
        self.style.configure("Disconnect.TButton", foreground="white", background="#dc3545", font=("Helvetica", 10, "bold"))
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(
            top,
            text="AURA Ground Segment Console",
            font=("Helvetica", 16, "bold"),
        ).pack(side="left", padx=(0, 18))

        self.status_var = tk.StringVar(value="Initializing connection to Renode LEON3 core...")
        ttk.Label(top, textvariable=self.status_var, font=("Helvetica", 10, "italic")).pack(side="left")

        # Interactive connection status controller
        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection, style="Connect.TButton")
        self.connect_button.pack(side="right")

        controls = ttk.LabelFrame(self.root, text="Telemetry Request Panel", padding=10)
        controls.pack(fill="x", padx=10, pady=(0, 10))

        self.buttons = {}
        button_specs = [
            ("cloud", "☁ Landmark Cloud"),
            ("entropy", "▦ Entropy Heatmap"),
            ("cut", "✂ Cut Image (Gated)"),
            ("full", "▣ Full Raw Frame"),
        ]

        for mode, label in button_specs:
            btn = ttk.Button(
                controls,
                text=label,
                command=lambda m=mode: self.request_mode(m),
            )
            btn.pack(side="left", padx=5, fill="x", expand=True)
            self.buttons[mode] = btn

        self.info_var = tk.StringVar(value="System Idle. Awaiting connection.")
        ttk.Label(
            self.root,
            textvariable=self.info_var,
            anchor="w",
            font=("Courier New", 10, "bold"),
            foreground="#0056b3",
            padding=(10, 0, 10, 8),
        ).pack(fill="x")

        fig_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        fig_frame.pack(fill="both", expand=True)

        # Стабільна архітектура: Створюємо фігуру один раз
        self.fig = plt.figure(figsize=(8, 7))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=fig_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_empty()

    def _draw_empty(self):
        self.ax.clear()
        # Замість cbar.remove() безпечно очищаємо фігуру через повне видалення додаткових осей колірних шкал
        for ax in self.fig.axes[:]:
            if ax != self.ax:
                self.fig.delaxes(ax)
        self.cbar = None
        self.ax.set_title("AURA Downlink — Select Telemetry Mode")
        self.ax.set_xlim(0, IMG_WIDTH)
        self.ax.set_ylim(IMG_HEIGHT, 0)
        self.ax.set_xlabel("Spacecraft Frame X")
        self.ax.set_ylabel("Spacecraft Frame Y")
        self.canvas.draw_idle()

    def set_buttons_state(self, enabled: bool):
        state = "normal" if enabled and self.connected else "disabled"
        for btn in self.buttons.values():
            btn.configure(state=state)

    def toggle_connection(self):
        if self.connected:
            self.stop_event.set()
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
            self.events.put(("disconnected", None))
        else:
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
                    self.events.put(("error", f"Connection refused. Ensure Renode script.resc is executing."))
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
                self.rx_bytes_total += len(chunk)  # Точний локальний лічильник байт у додатку
                self._parse_buffer(sock)
            except OSError:
                break
        self.events.put(("disconnected", None))

    def _parse_buffer(self, sock):
        while True:
            if self.parser_state == "raw":
                if self.raw_expected <= 0:
                    self.parser_state = "packet"
                    continue

                take = min(len(self.parser_buffer), self.raw_expected - len(self.raw_buffer))
                if take == 0:
                    return

                self.raw_buffer.extend(self.parser_buffer[:take])
                del self.parser_buffer[:take]

                if len(self.raw_buffer) == self.raw_expected:
                    image = np.frombuffer(self.raw_buffer, dtype=np.uint8).copy()
                    if image.size == RAW_IMAGE_BYTES:
                        self.current_frame = image.reshape((IMG_HEIGHT, IMG_WIDTH))
                        mode = self.current_mode or "full"
                        self.events.put(("raw_complete", (mode, self.current_frame.copy())))
                    self.raw_expected = 0
                    self.raw_buffer.clear()
                    self.parser_state = "packet"
                    continue
                return

            if len(self.parser_buffer) < 4:
                return

            raw = struct.unpack(">I", self.parser_buffer[:4])[0]
            marker = (raw >> 24) & 0xFF

            if marker == MARKER_END:
                del self.parser_buffer[:4]
                self.events.put(("frame_complete", self.current_mode))
                continue

            if marker == MARKER_ACK:
                del self.parser_buffer[:4]
                mode_id = raw & 0xFF
                self.events.put(("ack", mode_id))
                continue

            if marker == MARKER_CLOUD_HEADER:
                del self.parser_buffer[:4]
                max_landmarks = (raw >> 12) & 0x0FFF
                step = (raw >> 8) & 0x0F
                threshold = raw & 0xFF
                self.current_mode = "cloud"
                self.cloud_x = []
                self.cloud_y = []
                self.cloud_score = []
                self.events.put(("cloud_header", (max_landmarks, step, threshold)))
                continue

            if marker == MARKER_CLOUD_POINT:
                del self.parser_buffer[:4]
                x = (raw >> 14) & 0x03FF
                y = (raw >> 4) & 0x03FF
                score = raw & 0x0F
                self.cloud_x.append(x)
                self.cloud_y.append(y)
                self.cloud_score.append(score)
                
                # МИТТЄВИЙ АПДЕЙТ: Відправляємо кожну точку в реальному часі без очікування
                self.events.put((
                    "cloud_update",
                    (list(self.cloud_x), list(self.cloud_y), list(self.cloud_score)),
                ))
                continue

            if marker == MARKER_ENTROPY_HEADER:
                del self.parser_buffer[:4]
                cols = (raw >> 16) & 0xFF
                rows = (raw >> 8) & 0xFF
                block = raw & 0xFF
                self.current_mode = "entropy"
                self.heatmap_cols = cols
                self.heatmap_rows = rows
                self.block_size = block
                self.heatmap = np.zeros((rows, cols), dtype=np.float32)
                self.events.put(("entropy_header", (rows, cols, block)))
                continue

            if marker == MARKER_ENTROPY_BLOCK:
                del self.parser_buffer[:4]
                col = (raw >> 17) & 0x7F
                row = (raw >> 10) & 0x7F
                entropy = (raw & 0x3FF) * 0.01
                if self.heatmap is not None and row < self.heatmap_rows and col < self.heatmap_cols:
                    self.heatmap[row, col] = entropy
                    
                    # МИТТЄВИЙ АПДЕЙТ: Візуалізуємо кожен завантажений блок одразу
                    self.events.put(("entropy_update", self.heatmap.copy()))
                continue

            if marker == MARKER_FULL_HEADER or marker == MARKER_CUT_HEADER:
                del self.parser_buffer[:4]
                self.current_mode = "full" if marker == MARKER_FULL_HEADER else "cut"
                self.raw_expected = raw & 0x00FFFFFF
                self.raw_buffer = bytearray()
                self.parser_state = "raw"
                self.events.put(("raw_header", (self.current_mode, self.raw_expected)))
                continue

            if marker == MARKER_CUT_META:
                del self.parser_buffer[:4]
                block = (raw >> 16) & 0xFF
                threshold = raw & 0xFFFF
                self.block_size = block
                self.cut_threshold = threshold * 0.01
                self.events.put(("cut_meta", (block, self.cut_threshold)))
                continue

            del self.parser_buffer[0]

    def request_mode(self, mode: str):
        if not self.connected or self.busy:
            return

        command = MODE_COMMANDS[mode]
        with self.sock_lock:
            try:
                self.sock.sendall(command)
            except OSError as exc:
                self.status_var.set(f"Tx Error: {exc}")
                return

        self.busy = True
        self.current_mode = mode
        self.current_frame = None
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_score = []
        self.heatmap = None
        self.cut_threshold = None
        self.raw_expected = 0
        self.raw_buffer = bytearray()
        self.parser_state = "packet"
        self.points_since_draw = 0
        self.blocks_since_draw = 0
        
        # Точка відліку для локального розрахунку байт кадру
        self.frame_started_bytes = self.rx_bytes_total
        
        self.info_var.set(f"Request Sent: {MODE_NAMES[mode]} ... Awaiting Payload ...")
        self.set_buttons_state(False)

        self.ax.clear()
        # Безпечний скид додаткових осей без використання cbar.remove()
        for ax in self.fig.axes[:]:
            if ax != self.ax:
                self.fig.delaxes(ax)
        self.cbar = None

        if mode in ("full", "cut"):
            self.ax.imshow(np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8), cmap="gray", vmin=0, vmax=255)
            self.ax.set_xlim(0, IMG_WIDTH)
            self.ax.set_ylim(IMG_HEIGHT, 0)
        self.canvas.draw_idle()

    def _render_cloud(self, payload):
        x, y, score = payload
        
        # Очищуємо поле повністю лише при старті (порожній масив), далі просто міняємо точки
        if not x and not y:
            self.ax.clear()
            for ax in self.fig.axes[:]:
                if ax != self.ax:
                    self.fig.delaxes(ax)
            self.cbar = None
            self.im_entropy = None
            self.ax.set_facecolor("#0b0c10")
            self.ax.set_xlim(0, IMG_WIDTH)
            self.ax.set_ylim(IMG_HEIGHT, 0)
            self.ax.set_aspect("equal", adjustable="box")
            self.ax.set_title("AURA — Landmark Feature Cloud (Streaming...)")
            self.ax.set_xlabel("Spacecraft Matrix X")
            self.ax.set_ylabel("Spacecraft Matrix Y")
            self.cloud_scatter = None
        
        if x and y:
            if hasattr(self, 'cloud_scatter') and self.cloud_scatter is not None:
                self.cloud_scatter.remove()
            self.cloud_scatter = self.ax.scatter(x, y, c=score, cmap="plasma", vmin=0, vmax=15, s=15, marker="+")
            self.ax.set_title(f"AURA — Landmark Feature Cloud ({len(x)} features tracked)")
            
        self.canvas.draw_idle()

    def _render_entropy(self, matrix):
        # Оновлення даних у реальному часі без повного очищення (set_data), як у твоєму коді
        if self.im_entropy is not None and self.current_mode == "entropy":
            self.im_entropy.set_data(matrix)
        else:
            self.ax.clear()
            for ax in self.fig.axes[:]:
                if ax != self.ax:
                    self.fig.delaxes(ax)
            
            # Еталонний рендеринг: суто за індексами матриці, без спотворень
            self.im_entropy = self.ax.imshow(
                matrix,
                cmap="jet",
                interpolation="nearest",
                vmin=0,
                vmax=8
            )
            self.ax.set_title(f"AURA Live Feed: Grayscale Verification ({self.heatmap_rows}x{self.heatmap_cols} Blocks)")
            self.ax.set_xlabel("Block Column Index")
            self.ax.set_ylabel("Block Row Index")
            
            self.cbar = self.fig.colorbar(self.im_entropy, ax=self.ax)
            self.cbar.set_label("Shannon Entropy (bits/pixel)", rotation=270, labelpad=15)
                
        self.canvas.draw_idle()

    def _render_image(self, image, mode):
        self.ax.clear()
        for ax in self.fig.axes[:]:
            if ax != self.ax:
                self.fig.delaxes(ax)
        self.cbar = None
        self.im_entropy = None
        
        self.ax.imshow(image, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        title = "AURA — Full Science Image Frame" if mode == "full" else "AURA — Dynamic Zero-Gated Low-Entropy Compressed Frame"
        self.ax.set_title(title)
        self.ax.set_xlim(0, IMG_WIDTH)
        self.ax.set_ylim(IMG_HEIGHT, 0)
        self.ax.set_xlabel("Sensor Pixel X")
        self.ax.set_ylabel("Sensor Pixel Y")
        self.canvas.draw_idle()

    def _process_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()

                if kind == "connected":
                    self.sock = payload
                    self.connected = True
                    self.busy = False
                    self.status_var.set(f"Active Link to LEON3 Simulator Core at {HOST}:{PORT}")
                    self.info_var.set("Link Established. Select telemetry processing architecture.")
                    self.connect_button.configure(text="Disconnect", style="Disconnect.TButton", state="normal")
                    self.set_buttons_state(True)

                elif kind == "error":
                    self.connected = False
                    self.busy = False
                    self.sock = None
                    self.status_var.set(payload)
                    self.info_var.set("Transport link failure.")
                    self.set_buttons_state(False)
                    self.connect_button.configure(text="Connect", style="Connect.TButton", state="normal")

                elif kind == "disconnected":
                    self.connected = False
                    self.busy = False
                    self.sock = None
                    self.status_var.set("Link to Spacecraft Core Closed.")
                    self.info_var.set("Click 'Connect' to re-establish synchronous link.")
                    self.set_buttons_state(False)
                    self.connect_button.configure(text="Connect", style="Connect.TButton", state="normal")
                    self._draw_empty()

                elif kind == "ack":
                    modes = {1: "CLOUD", 2: "ENTROPY", 3: "CUT IMAGE", 4: "FULL IMAGE"}
                    self.status_var.set(f"LEON3 On-Board Execution ACK: Processing {modes.get(payload, 'UNKNOWN')}")

                elif kind == "cloud_header":
                    max_landmarks, step, threshold = payload
                    self.info_var.set(f"Landmark Stream: limit={max_landmarks}, step={step}px, noise_threshold={threshold}")
                    self._render_cloud(([], [], []))

                elif kind == "cloud_update":
                    # Потокова отрисовка точок
                    self._render_cloud(payload)

                elif kind == "entropy_header":
                    rows, cols, block = payload
                    self.im_entropy = None  # Скидаємо попередній рендер перед новою сіткою
                    self.info_var.set(f"Telemetry Layout Unpacked: {cols * block}x{rows * block} | Block: {block}px ({rows}x{cols} grid)")
                    self._render_entropy(np.zeros((rows, cols), dtype=np.float32))

                elif kind == "entropy_update":
                    # Потокова отрисовка блоків матриці ентропії в реальному часі
                    self._render_entropy(payload)

                elif kind == "cut_meta":
                    block, threshold = payload
                    self.info_var.set(f"Zero-Gating Parameter: kernel={block}px, zeroing arrays where entropy < {threshold:.2f} bits/px")

                elif kind == "raw_header":
                    mode, length = payload
                    self.info_var.set(f"Downloading {MODE_NAMES[mode]}: Streaming {length / 1024:.2f} KiB raw memory packet...")

                elif kind == "raw_complete":
                    mode, image = payload
                    self.current_frame = image
                    self._render_image(image, mode)
                    if mode == "cut":
                        zero_pct = float(np.mean(image == 0) * 100.0)
                        self.info_var.set(f"Gated image unpacked. Hardware link bandwidth saved: {zero_pct:.2f}% of matrix nulled.")
                    else:
                        self.info_var.set("Full raw science data saved to ground database.")

                elif kind == "frame_complete":
                    mode = self.current_mode
                    frame_bytes = self.rx_bytes_total - self.frame_started_bytes
                    name = MODE_NAMES.get(mode, "Frame")
                    self.busy = False
                    self.set_buttons_state(True)
                    self.status_var.set(f"Telemetry Packet Complete: {name}")

                    if mode == "cloud":
                        info = f"{name}: {len(self.cloud_x)} features, Real Telemetry Streamed = {frame_bytes} bytes ({frame_bytes / 1024:.2f} KiB)"
                    elif mode == "entropy":
                        info = f"{name}: {self.heatmap_rows * self.heatmap_cols} block metrics, Real Telemetry Streamed = {frame_bytes} bytes ({frame_bytes / 1024:.2f} KiB)"
                    else:
                        info = f"{name}: {RAW_IMAGE_BYTES} pixels matrix, Real Telemetry Streamed = {frame_bytes} bytes ({frame_bytes / 1024:.2f} KiB)"
                    self.info_var.set(info)

                    if mode == "cloud":
                        self._render_cloud((self.cloud_x, self.cloud_y, self.cloud_score))
                    elif mode == "entropy" and self.heatmap is not None:
                        self._render_entropy(self.heatmap)

        except queue.Empty:
            pass

        self.root.after(50, self._process_events)

    def on_close(self):
        self.stop_event.set()
        self.busy = False
        self.connected = False
        try:
            if self.sock is not None:
                self.sock.close()
        except OSError:
            pass
        plt.close(self.fig)
        self.root.destroy()


def main():
    root = tk.Tk()
    app = AuraGroundUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
