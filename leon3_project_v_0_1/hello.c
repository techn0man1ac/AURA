#define UART_BASE 0x80000100
#define UART_RBR_THR ((volatile unsigned char *)(UART_BASE + 0))
#define UART_LSR     ((volatile unsigned char *)(UART_BASE + 5))
#define LSR_THRE 0x20

#define IMAGE_ADDRESS 0x40600000
#define IMG_WIDTH     1020
#define IMG_HEIGHT    1020
#define BLOCK_SIZE    15  

/* Статичні масиви (ECSS Category D Compliance) */
unsigned short color_histogram[256]; 
unsigned char  used_bins[BLOCK_SIZE * BLOCK_SIZE];

void uart_putc(char c) {
    while (!(*UART_LSR & LSR_THRE));
    *UART_RBR_THR = c;
}

void uart_print(const char *str) {
    while (*str) {
        uart_putc(*str);
        str++;
    }
}

unsigned int integer_log2(unsigned int val) {
    unsigned int res = 0;
    while (val >>= 1) { res++; }
    return res;
}

/* ФІКС: Коректний лінійний розрахунок координати Grayscale без зсуву Stride */
unsigned int calculate_block_joint_rgb_entropy(int x0, int y0, int block_w, int block_h) {
    unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;
    int num_pixels = block_w * block_h;
    int used_bins_count = 0;

    for (int y = 0; y < block_h; y++) {
        // Абсолютна адреса рядка в одновимірному масиві Grayscale кадру
        int global_y_idx = (y0 + y) * IMG_WIDTH;
        for (int x = 0; x < block_w; x++) {
            int global_x_idx = x0 + x;
            int px_idx = global_y_idx + global_x_idx;
            
            unsigned char intensity = grayscale_pixels[px_idx];
            
            if (color_histogram[intensity] == 0) {
                used_bins[used_bins_count++] = intensity;
            }
            color_histogram[intensity]++;
        }
    }

    unsigned int total_entropy = 0;
    unsigned int log2_num_pixels = integer_log2(num_pixels);

    for (int i = 0; i < used_bins_count; i++) {
        unsigned char bin_idx = used_bins[i];
        unsigned int count = color_histogram[bin_idx];
        unsigned int p_log_p = count * (log2_num_pixels - integer_log2(count));
        total_entropy += p_log_p;
        color_histogram[bin_idx] = 0; // Швидке точкове очищення
    }

    return (total_entropy * 100) / num_pixels;
}

void uart_send_uint32(unsigned int packet) {
    for (int i = 3; i >= 0; i--) {
        while (!(*UART_LSR & LSR_THRE));
        *UART_RBR_THR = (unsigned char)((packet >> (i * 8)) & 0xFF);
    }
}

void process_imageGrid(void) {
    unsigned int packed_packet = 0;

    // Конфігураційний маркер сітки (0x5A) -> 68x68 блоків
    packed_packet = (0x5A << 24) | ((IMG_WIDTH / BLOCK_SIZE) << 16) | ((IMG_HEIGHT / BLOCK_SIZE) << 8) | BLOCK_SIZE;
    uart_send_uint32(packed_packet);

    for (int y = 0; y < IMG_HEIGHT; y += BLOCK_SIZE) {
        for (int x = 0; x < IMG_WIDTH; x += BLOCK_SIZE) {
            
            int bw = (x + BLOCK_SIZE > IMG_WIDTH) ? (IMG_WIDTH - x) : BLOCK_SIZE;
            int bh = (y + BLOCK_SIZE > IMG_HEIGHT) ? (IMG_HEIGHT - y) : BLOCK_SIZE;

            unsigned int entropy = calculate_block_joint_rgb_entropy(x, y, bw, bh);

            int col_idx = x / BLOCK_SIZE;
            int row_idx = y / BLOCK_SIZE;
            
            packed_packet = (0xA5 << 24) | ((col_idx & 0x7F) << 17) | ((row_idx & 0x7F) << 10) | (entropy & 0x3FF);
            uart_send_uint32(packed_packet);
        }
    }

    packed_packet = (0xFE << 24);
    uart_send_uint32(packed_packet);
}

void _start(void) {
    for (volatile int i = 0; i < 100000; i++);

    uart_print("\n===================================\n");
    uart_print("AURA Firmware: Compliant Grayscale Fixed Link\n");
    uart_print("===================================\n\n");

    process_imageGrid();

    while (1);
}
