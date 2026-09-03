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
        print("Connected successfully! Decompressing aerospace binary stream...")
    except ConnectionRefusedError:
        print("Error: Renode server not running. Launch script.resc first.")
        return

    fig, ax = plt.subplots(figsize=(10, 8))
    im = None
    cbar = None
    heatmap_matrix = None
    img_w, img_h, block_size = 0, 0, 0
    cols, rows = 0, 0

    total_binary_bytes_received = 0
    equivalent_text_bytes = 0
    block_draw_counter = 0

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

                if sync_byte == 0x5A:  # Конфігурація розкладки
                    cols = (raw_uint32 >> 16) & 0xFF
                    rows = (raw_uint32 >> 8) & 0xFF
                    block_size = raw_uint32 & 0xFF
                    img_w = cols * block_size
                    img_h = rows * block_size
                    
                    heatmap_matrix = np.zeros((rows, cols))
                    total_binary_bytes_received += 4
                    
                    print(f"Telemetry Layout Unpacked: {img_w}x{img_h} | Block: {block_size}px ({rows}x{cols} grid)")
                    
                    ax.clear()
                    im = ax.imshow(heatmap_matrix, cmap="jet", interpolation="nearest", vmin=0, vmax=8)
                    if not cbar:
                        cbar = fig.colorbar(im)
                        cbar.set_label("Shannon Entropy (bits/pixel)", rotation=270, labelpad=15)
                    ax.set_title(f"AURA Live Feed: Grayscale Verification ({rows}x{cols} Blocks)")
                    plt.ion()
                    plt.show(block=False)
                    
                    data_buffer = data_buffer[packet_size:]
                    continue

                elif sync_byte == 0xA5:  # Блок даних
                    col_idx = (raw_uint32 >> 17) & 0x7F
                    row_idx = (raw_uint32 >> 10) & 0x7F
                    entropy_scaled = raw_uint32 & 0x3FF
                    entropy = entropy_scaled * 0.01

                    if row_idx < rows and col_idx < cols:
                        heatmap_matrix[row_idx, col_idx] = entropy
                        
                        total_binary_bytes_received += 4
                        equivalent_text_bytes += 27 
                        block_draw_counter += 1

                        if block_draw_counter % 100 == 0:
                            im.set_data(heatmap_matrix)
                            fig.canvas.draw_idle()
                            fig.canvas.flush_events()

                    data_buffer = data_buffer[packet_size:]
                    continue

                elif sync_byte == 0xFE:  # Фінал
                    total_binary_bytes_received += 4
                    print("\n--- TELEMETRY DOWNLOAD COMPLETE ---")
                    raise GeneratorExit

                else:
                    data_buffer = data_buffer[1:]

    except (KeyboardInterrupt, GeneratorExit):
        pass
    finally:
        client_socket.close()
        
        if total_binary_bytes_received > 0:
            compression_ratio = (equivalent_text_bytes / total_binary_bytes_received)
            data_savings_pct = (1.0 - (total_binary_bytes_received / equivalent_text_bytes)) * 100.0
            
            print("\n=======================================================")
            print("🚀 AURA LINK DATA COMPRESSION REPORT (LOSSLESS)")
            print("=======================================================")
            print(f"• Baseline Telemetry Volume (Text Code): {equivalent_text_bytes / 1024:.2f} KB")
            print(f"• Bit-Packed Telemetry Volume (New Code): {total_binary_bytes_received / 1024:.2f} KB")
            print(f"• Absolute Radio Traffic Downlink Savings: {data_savings_pct:.1f}%")
            print(f"• Bandwidth Optimization Factor: {compression_ratio:.2f}x Faster Downlink")
            print("=======================================================\n")

        if heatmap_matrix is not None and im is not None:
            im.set_data(heatmap_matrix)
            ax.set_title("AURA Interactive Heatmap (Lossless Packed Render)")
            fig.canvas.draw()
        
        plt.ioff()
        print("UI unblocked. You can now zoom and explore the compressed telemetry freely.")
        plt.show()

if __name__ == "__main__":
    start_live_receiver()
