import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

HOST = "127.0.0.1"
PORT = 12345

def start_live_receiver():
    print(f"Connecting to AURA Telemetry Link on {HOST}:{PORT}...")
    
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    try:
        client_socket.connect((HOST, PORT))
        print("Connected successfully! Decoding aerospace binary stream...")
    except ConnectionRefusedError:
        print("Error: Renode server not running. Launch script.resc first.")
        return

    # Налаштування вікна графіків
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Створюємо базову підкладку 1020x1020 для хмари точок та теплової карти
    base_image = np.zeros((1020, 1020))
    im = ax.imshow(base_image, cmap="gray", vmin=0, vmax=255)
    cbar = None
    
    # Шар для малювання хмари точок-орієнтирів (Зелені хрестики)
    landmark_dots, = ax.plot([], [], 'g+', markersize=10, markeredgewidth=1.5, label="Detected Landmarks")
    
    ax.set_xlim(0, 1020)
    ax.set_ylim(1020, 0)
    ax.set_title("AURA Ground Segment: Live Stream Analysis")
    plt.ion()
    plt.show(block=False)

    # Буфери та змінні стану
    heatmap_matrix = None
    img_w, img_h, block_size = 0, 0, 0
    cols, rows = 0, 0
    x_coords, y_coords = [], []

    total_binary_bytes_received = 0
    equivalent_text_bytes = 0
    packet_counter = 0

    data_buffer = b""
    packet_size = 4  # 32 біти = 4 байти

    try:
        while True:
            packet = client_socket.recv(16384)
            if not packet:
                break
            
            data_buffer += packet
            
            while len(data_buffer) >= packet_size:
                raw_uint32 = struct.unpack(">I", data_buffer[:packet_size])[0]
                sync_byte = (raw_uint32 >> 24) & 0xFF

                # --- РЕЖИМ 1: ТЕПЛОВА КАРТА ЕНТРОПІЇ ---
                if sync_byte == 0x5A:
                    cols = (raw_uint32 >> 16) & 0xFF
                    rows = (raw_uint32 >> 8) & 0xFF
                    block_size = raw_uint32 & 0xFF
                    img_w = cols * block_size
                    img_h = rows * block_size
                    
                    heatmap_matrix = np.zeros((rows, cols))
                    total_binary_bytes_received += 4
                    
                    print(f"[Mode: Heatmap] Layout Unpacked: {img_w}x{img_h} ({rows}x{cols} grid)")
                    
                    ax.clear()
                    im = ax.imshow(heatmap_matrix, cmap="jet", interpolation="nearest", vmin=0, vmax=8)
                    if not cbar:
                        cbar = fig.colorbar(im, ax=ax)
                        cbar.set_label("Shannon Entropy (bits/pixel)", rotation=270, labelpad=15)
                    ax.set_title(f"AURA Live Feed: Entropy Heatmap ({rows}x{cols} Blocks)")
                    
                    data_buffer = data_buffer[packet_size:]
                    continue

                elif sync_byte == 0xA5:
                    col_idx = (raw_uint32 >> 17) & 0x7F
                    row_idx = (raw_uint32 >> 10) & 0x7F
                    entropy = (raw_uint32 & 0x3FF) * 0.01

                    if heatmap_matrix is not None and row_idx < rows and col_idx < cols:
                        heatmap_matrix[row_idx, col_idx] = entropy
                        total_binary_bytes_received += 4
                        equivalent_text_bytes += 27 
                        packet_counter += 1

                        if packet_counter % 100 == 0:
                            im.set_data(heatmap_matrix)
                            fig.canvas.draw_idle()
                            fig.canvas.flush_events()

                    data_buffer = data_buffer[packet_size:]
                    continue

                # --- РЕЖИМ 2: ХМАРА ТОЧОК-ОРІЄНТИРІВ (МАРКЕР 0xBD) ---
                elif sync_byte == 0xBD:
                    x_idx = (raw_uint32 >> 14) & 0x03FF   # [23:14] X (10 біт)
                    y_idx = (raw_uint32 >> 4) & 0x03FF    # [13:4]  Y (10 біт)
                    score = raw_uint32 & 0x0F             # [3:0]   Якість

                    x_coords.append(x_idx)
                    y_coords.append(y_idx)
                    
                    total_binary_bytes_received += 4
                    equivalent_text_bytes += 27 
                    packet_counter += 1

                    if packet_counter % 10 == 0:
                        landmark_dots.set_data(x_coords, y_coords)
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()

                    data_buffer = data_buffer[packet_size:]
                    continue

                # --- ФІНАЛ КАДРУ ---
                elif sync_byte == 0xFE:
                    total_binary_bytes_received += 4
                    print("\n--- TELEMETRY DOWNLOAD COMPLETE ---")
                    data_buffer = data_buffer[packet_size:]
                    raise GeneratorExit

                else:
                    data_buffer = data_buffer[1:]

    except (KeyboardInterrupt, GeneratorExit):
        pass
    finally:
        client_socket.close()
        
        # ЗАХИСТ ВІД ZERO DIVISION ПРИ РОЗРАХУНКУ СТАТИСТИКИ
        if total_binary_bytes_received > 0 and equivalent_text_bytes > 0:
            compression_ratio = (equivalent_text_bytes / total_binary_bytes_received)
            data_savings_pct = (1.0 - (total_binary_bytes_received / equivalent_text_bytes)) * 100.0
            
            print("\n=======================================================")
            print("🚀 AURA LINK TELEMETRY REPORT")
            print("=======================================================")
            print(f"• Raw Text Logs Volume: {equivalent_text_bytes / 1024:.2f} KB")
            print(f"• Bit-Packed Volume: {total_binary_bytes_received / 1024:.2f} KB")
            print(f"• Downlink Radio Traffic Savings: {data_savings_pct:.1f}%")
            print(f"• Bandwidth Optimization Efficiency: {compression_ratio:.2f}x Faster")
            print("=======================================================\n")
        else:
            print("\n=======================================================")
            print("🛰️ AURA LINK: NO ACTIVE TELEMETRY PACKETS RECEIVED")
            print("=======================================================\n")

        # Фінальне фіксування графіків залежно від того, що прийшло
        if len(x_coords) > 0:
            landmark_dots.set_data(x_coords, y_coords)
            ax.set_title(f"AURA Static Render: Cloud of {len(x_coords)} Landmarks")
            ax.legend(loc="upper right")
        elif heatmap_matrix is not None and im is not None:
            im.set_data(heatmap_matrix)
            ax.set_title("AURA Interactive Heatmap (Lossless Packed Render)")
            
        fig.canvas.draw()
        plt.ioff()
        print("UI unblocked. You can now zoom and explore the telemetry freely.")
        plt.show()

if __name__ == "__main__":
    start_live_receiver()
