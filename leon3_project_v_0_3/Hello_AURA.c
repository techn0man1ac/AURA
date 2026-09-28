/*
   AURA LEON3 firmware

   Interactive telemetry modes requested from the Ground Segment:
     CLOUD   - sparse landmark cloud (X, Y, 4-bit contrast score)
     ENTROPY - block entropy map
     CUT     - full-size grayscale image, low-entropy blocks replaced by 0
     FULL    - complete raw grayscale image

   UART / Renode command protocol (ASCII, host -> LEON3):
     AURA CLOUD\n
     AURA ENTROPY\n
     AURA CUT\n
     AURA FULL\n

   Telemetry protocol (LEON3 -> Ground Segment): every structured control/data
   packet is 32-bit big-endian. Raw image payloads follow a 32-bit header and
   are then sent as raw bytes for the exact number of IMAGE_BYTES bytes.

   Packet markers:
     0xACxxxxxx  command accepted; low byte = mode id
     0x5Bxxxxxx  landmark-cloud header
     0xBDxxxxxx  landmark record: X(10) | Y(10) | score(4)
     0x5Axxxxxx  entropy-grid header: cols | rows | block_size
     0xA5xxxxxx  entropy block: col(7) | row(7) | entropy_x100(10)
     0xC0xxxxxx  FULL image header; low 24 bits = raw payload length
     0xC1xxxxxx  CUT image header;  low 24 bits = raw payload length
     0xC2xxxxxx  CUT metadata: block_size(8) | threshold_x100(16)
     0xFE000000  end of frame

   Notes:
   - The entropy routine intentionally preserves the integer log2 approximation
     used in the original AURA prototype for deterministic LEON3 execution.
   - Image geometry is fixed at 1020x1020, 8-bit grayscale, so X/Y need 10 bits.
   - The image is addressed directly at IMAGE_ADDRESS for Renode simulation.
*/

/*
 * AURA LEON3 Firmware V0.3 - High-Performance Bare-Metal Pipeline
 * 
 * Aligned Architecture for Full-Grid Shannon Entropy & Feature Tracking.
 * Hardware Memory Mapping designed strictly for Renode Space Telemetry.
 */

#define UART_BASE 0x80000100
#define UART_RBR_THR ((volatile unsigned char *)(UART_BASE + 0))
#define UART_LSR     ((volatile unsigned char *)(UART_BASE + 5))
#define LSR_DR       0x01
#define LSR_THRE     0x20

#define IMAGE_ADDRESS 0x40600000
#define IMG_WIDTH     1020
#define IMG_HEIGHT    1020
#define IMAGE_BYTES   (IMAGE_BYTES_VAL)
#define IMAGE_BYTES_VAL (IMG_WIDTH * IMG_HEIGHT)

/* Конфігураційні параметри фільтрації */
#define CLOUD_THRESHOLD 45
#define CLOUD_STEP      12
#define CLOUD_MARGIN    15
#define MAX_LANDMARKS   1024

#define BLOCK_SIZE      15
#define CUT_ENTROPY_THRESHOLD_X100 250

#define MAX_BLOCK_COLS ((IMG_WIDTH  + BLOCK_SIZE - 1) / BLOCK_SIZE)
#define MAX_BLOCK_ROWS ((IMG_HEIGHT + BLOCK_SIZE - 1) / BLOCK_SIZE)

/* Статичні регістри пам'яті для уникнення динамічного виділення (No Dynamic Allocation) */
unsigned short color_histogram[256];
unsigned short used_bins[BLOCK_SIZE * BLOCK_SIZE];
unsigned char block_keep[MAX_BLOCK_COLS * MAX_BLOCK_ROWS];

static void uart_putc(char c) {
    while (!(*UART_LSR & LSR_THRE));
    *UART_RBR_THR = (unsigned char)c;
}

static int uart_getc_nonblocking(char *out) {
    if (*UART_LSR & LSR_DR) {
        *out = (char)(*UART_RBR_THR);
        return 1;
    }
    return 0;
}

static void print_str(const char *s) {
    while (*s) {
        uart_putc(*s);
        s++;
    }
}

static void uart_send_uint32(unsigned int packet) {
    for (int i = 3; i >= 0; i--) {
        while (!(*UART_LSR & LSR_THRE));
        *UART_RBR_THR = (unsigned char)((packet >> (i * 8)) & 0xFF);
    }
}

static void uart_send_bytes(const unsigned char *data, unsigned int length) {
    for (unsigned int i = 0; i < length; i++) {
        while (!(*UART_LSR & LSR_THRE));
        *UART_RBR_THR = data[i];
    }
}

static unsigned int integer_log2(unsigned int val) {
    unsigned int res = 0;
    while (val >>= 1) { res++; }
    return res;
}

static unsigned int calculate_block_entropy_x100(int x0, int y0, int block_w, int block_h) {
    unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;
    int num_pixels = block_w * block_h;
    int used_bins_count = 0;
    unsigned int total_entropy = 0;
    unsigned int log2_num_pixels = integer_log2((unsigned int)num_pixels);

    for (int y = 0; y < block_h; y++) {
        int global_y_idx = (y0 + y) * IMG_WIDTH;
        for (int x = 0; x < block_w; x++) {
            unsigned int px_idx = (unsigned int)(global_y_idx + x0 + x);
            unsigned char intensity = grayscale_pixels[px_idx];

            if (color_histogram[intensity] == 0) {
                used_bins[used_bins_count++] = intensity;
            }
            color_histogram[intensity]++;
        }
    }

    for (int i = 0; i < used_bins_count; i++) {
        unsigned short bin_idx = used_bins[i];
        unsigned int count = color_histogram[bin_idx];
        unsigned int p_log_p = count * (log2_num_pixels - integer_log2(count));
        total_entropy += p_log_p;
        color_histogram[bin_idx] = 0; 
    }

    return (total_entropy * 100U) / (unsigned int)num_pixels;
}

static void process_landmark_cloud(void) {
    unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;
    unsigned int landmarks_found = 0;

    uart_send_uint32((0x5BU << 24) | ((MAX_LANDMARKS & 0x0FFFU) << 12) | ((CLOUD_STEP & 0x0FU) << 8) | (CLOUD_THRESHOLD & 0xFFU));

    for (int y = CLOUD_MARGIN; y < IMG_HEIGHT - CLOUD_MARGIN; y += CLOUD_STEP) {
        for (int x = CLOUD_MARGIN; x < IMG_WIDTH - CLOUD_MARGIN; x += CLOUD_STEP) {
            if (landmarks_found >= MAX_LANDMARKS) break;

            unsigned char p_top   = grayscale_pixels[(y - 8) * IMG_WIDTH + x];
            unsigned char p_down  = grayscale_pixels[(y + 8) * IMG_WIDTH + x];
            unsigned char p_left  = grayscale_pixels[y * IMG_WIDTH + (x - 8)];
            unsigned char p_right = grayscale_pixels[y * IMG_WIDTH + (x + 8)];

            int diff_v = p_top - p_down;
            int diff_h = p_left - p_right;
            if (diff_v < 0) diff_v = -diff_v;
            if (diff_h < 0) diff_h = -diff_h;

            int total_score = diff_v + diff_h;

            if (total_score > CLOUD_THRESHOLD) {
                unsigned int packed_score = (unsigned int)(total_score >> 4);
                if (packed_score > 15U) packed_score = 15U;

                unsigned int packed_packet = (0xBDU << 24) | (((unsigned int)x & 0x03FFU) << 14) | (((unsigned int)y & 0x03FFU) << 4) | (packed_score & 0x0FU);
                uart_send_uint32(packed_packet);
                landmarks_found++;
            }
        }
        if (landmarks_found >= MAX_LANDMARKS) break;
    }
    uart_send_uint32(0xFE000000U);
}

static void process_entropy_grid(void) {
    int cols = MAX_BLOCK_COLS;
    int rows = MAX_BLOCK_ROWS;

    uart_send_uint32((0x5AU << 24) | (((unsigned int)cols & 0xFFU) << 16) | (((unsigned int)rows & 0xFFU) << 8) | (BLOCK_SIZE & 0xFFU));

    for (int y = 0; y < IMG_HEIGHT; y += BLOCK_SIZE) {
        for (int x = 0; x < IMG_WIDTH; x += BLOCK_SIZE) {
            int col_idx = x / BLOCK_SIZE;
            int row_idx = y / BLOCK_SIZE;

            int bw = (x + BLOCK_SIZE > IMG_WIDTH) ? (IMG_WIDTH - x) : BLOCK_SIZE;
            int bh = (y + BLOCK_SIZE > IMG_HEIGHT) ? (IMG_HEIGHT - y) : BLOCK_SIZE;
            unsigned int entropy = calculate_block_entropy_x100(x, y, bw, bh);

            unsigned int packed_packet = (0xA5U << 24) | (((unsigned int)col_idx & 0x7FU) << 17) | (((unsigned int)row_idx & 0x7FU) << 10) | (entropy & 0x3FFU);
            uart_send_uint32(packed_packet);
        }
    }
    uart_send_uint32(0xFE000000U);
}

static void process_cut_image(void) {
    unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;

    uart_send_uint32((0xC1U << 24) | (IMAGE_BYTES_VAL & 0x00FFFFFFU));
    uart_send_uint32((0xC2U << 24) | ((BLOCK_SIZE & 0xFFU) << 16) | (CUT_ENTROPY_THRESHOLD_X100 & 0xFFFFU));

    for (int y = 0; y < IMG_HEIGHT; y += BLOCK_SIZE) {
        int row_idx = y / BLOCK_SIZE;
        for (int x = 0; x < IMG_WIDTH; x += BLOCK_SIZE) {
            int col_idx = x / BLOCK_SIZE;
            
            int bw = (x + BLOCK_SIZE > IMG_WIDTH) ? (IMG_WIDTH - x) : BLOCK_SIZE;
            int bh = (y + BLOCK_SIZE > IMG_HEIGHT) ? (IMG_HEIGHT - y) : BLOCK_SIZE;
            unsigned int entropy = calculate_block_entropy_x100(x, y, bw, bh);
            block_keep[row_idx * MAX_BLOCK_COLS + col_idx] = (entropy >= CUT_ENTROPY_THRESHOLD_X100) ? 1 : 0;
        }
    }

    for (int y = 0; y < IMG_HEIGHT; y++) {
        unsigned int block_row = y / BLOCK_SIZE;
        for (int x = 0; x < IMG_WIDTH; x++) {
            unsigned int block_col = x / BLOCK_SIZE;
            unsigned char out_px = 0;
            
            if (block_keep[block_row * MAX_BLOCK_COLS + block_col]) {
                out_px = grayscale_pixels[y * IMG_WIDTH + x];
            }
            uart_putc((char)out_px);
        }
    }
    uart_send_uint32(0xFE000000U);
}

static void process_full_image(void) {
    unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;
    uart_send_uint32((0xC0U << 24) | (IMAGE_BYTES_VAL & 0x00FFFFFFU));
    uart_send_bytes(grayscale_pixels, IMAGE_BYTES_VAL);
    uart_send_uint32(0xFE000000U);
}

static int text_equals(const char *a, const char *b) {
    while (*a && *b) {
        if (*a != *b) return 0;
        a++; b++;
    }
    return (*a == '\0' && *b == '\0');
}

static int decode_command(const char *cmd) {
    if (text_equals(cmd, "AURA CLOUD"))   return 1;
    if (text_equals(cmd, "AURA ENTROPY")) return 2;
    if (text_equals(cmd, "AURA CUT"))     return 3;
    if (text_equals(cmd, "AURA FULL"))    return 4;
    return 0;
}

static int uart_poll_command(char *cmd_buffer, int *cmd_pos) {
    char c;
    while (uart_getc_nonblocking(&c)) {
        if (c == '\r' || c == '\n') {
            cmd_buffer[*cmd_pos] = '\0';
            int mode = decode_command(cmd_buffer);
            *cmd_pos = 0;
            return mode;
        }
        if (*cmd_pos < 31) {
            cmd_buffer[*cmd_pos] = c;
            (*cmd_pos)++;
        } else {
            *cmd_pos = 0;
        }
    }
    return 0;
}

int main(void) {
    char cmd_buffer[32];
    int cmd_pos = 0;

    print_str("\n===================================\n");
    print_str("AURA Firmware V0.3: Active Unified Core\n");
    print_str("Commands: AURA CLOUD | AURA ENTROPY | AURA CUT | AURA FULL\n");
    print_str("===================================\n\n");

    while (1) {
        int mode = uart_poll_command(cmd_buffer, &cmd_pos);
        if (mode == 1) {
            uart_send_uint32(0xAC000001U);
            process_landmark_cloud();
        } else if (mode == 2) {
            uart_send_uint32(0xAC000002U);
            process_entropy_grid();
        } else if (mode == 3) {
            uart_send_uint32(0xAC000003U);
            process_cut_image();
        } else if (mode == 4) {
            uart_send_uint32(0xAC000004U);
            process_full_image();
        }
    }
    return 0;
}