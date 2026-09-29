/*
 * AURA LEON3 Firmware V0.4.1
 *
 * Fixed 1020x1020, 8-bit grayscale image.
 *
 * Telemetry levels:
 *   L0 - Top-100 local-contrast maxima, X/Y + 4-bit score.
 *   L1 - Top-1000 local-contrast maxima, X/Y only (packed 20-bit points).
 *   L2 - FULL adaptive Shannon entropy map for the whole image.
 *   L3 - Sparse ROI image blocks selected by entropy.
 *   L4 - FULL image streamed block-by-block in raster order.
 *
 * Ground commands:
 *   AURA L0\n
 *   AURA L1\n
 *   AURA L2\n
 *   AURA L3\n
 *   AURA L4\n
 *
 * V0.3 compatibility aliases are preserved:
 *   AURA CLOUD   -> L1
 *   AURA ENTROPY -> L2
 *   AURA CUT     -> L3
 *   AURA FULL    -> L4
 *
 * No dynamic allocation. No floating-point operations in the flight path.
 * Structured words are transmitted big-endian, 32-bit, byte-by-byte.
 */

#define UART_BASE 0x80000100
#define UART_RBR_THR ((volatile unsigned char *)(UART_BASE + 0))
#define UART_LSR     ((volatile unsigned char *)(UART_BASE + 5))
#define LSR_DR       0x01
#define LSR_THRE     0x20

#define IMAGE_ADDRESS 0x40600000
#define IMG_WIDTH     1020
#define IMG_HEIGHT    1020
#define IMAGE_BYTES   (IMG_WIDTH * IMG_HEIGHT)

/* ---------------- Feature cloud ---------------- */
#define CLOUD_THRESHOLD        45
#define CLOUD_STEP             8
#define CLOUD_MARGIN           8
#define CLOUD_MAX_GRID         128
#define CLOUD_SCORE_MAX        510
#define TOP_N_L0               100
#define TOP_N_L1               1000

/* ---------------- Adaptive ROI / entropy -------- */
#define ROI_TARGET_BLOCKS        512
#define ROI_RADIUS_PIXELS        8
#define BLOCK_16                 16
#define BLOCK_8                   8
#define BLOCK_4                   4
#define CUT_ENTROPY_THRESHOLD_X100 250
#define MAX_ROI_GRID             256
#define MAX_ROI_CELLS            (MAX_ROI_GRID * MAX_ROI_GRID)

/* Full-frame L4 stream uses a fixed raster block size. */
#define L4_BLOCK_SIZE            16

/* ---------------- Protocol markers --------------- */
#define MARKER_ACK              0xAC
#define MARKER_L0_HEADER        0x5B
#define MARKER_L0_POINT         0xBD
#define MARKER_L1_HEADER        0xB1
#define MARKER_L2_HEADER        0xD0
#define MARKER_L2_ENTROPY       0xD1
#define MARKER_L2_COUNT         0xD2
#define MARKER_L3_HEADER        0xD3
#define MARKER_L3_META          0xD4
#define MARKER_L3_BLOCK         0xD5
#define MARKER_L3_COUNT         0xD6
#define MARKER_FULL_HEADER      0xC0
#define MARKER_FULL_BLOCK       0xC3
#define MARKER_END              0xFE

/* ---------------- Static memory ------------------ */
/* 128x128 x uint16_t = 32 KiB. Enough for CLOUD_STEP=8. */
static unsigned short cloud_score_grid[CLOUD_MAX_GRID * CLOUD_MAX_GRID];
static unsigned short score_histogram[CLOUD_SCORE_MAX + 1];

typedef struct {
    unsigned short x;
    unsigned short y;
    unsigned short score_full;
} Landmark;

static Landmark landmarks[TOP_N_L1];
static unsigned int landmark_count = 0;

/* 256x256 byte grids = 64 KiB each; fixed maximum grid for 4x4 blocks. */
static unsigned char roi_mask[MAX_ROI_CELLS];
static unsigned char roi_keep[MAX_ROI_CELLS];
static unsigned char block_seen[MAX_ROI_CELLS];

static unsigned short entropy_histogram[256];
static unsigned char entropy_used_bins[256];

/* log2(n) in Q8 for n=0..256. */
static const unsigned short log2_q8[257] = {
    0,0,256,406,512,594,662,719,768,812,850,886,918,947,975,1000,
    1024,1046,1068,1087,1106,1124,1142,1158,1174,1189,1203,1217,1231,1244,1256,1268,
    1280,1291,1302,1313,1324,1334,1343,1353,1362,1372,1380,1389,1398,1406,1414,1422,
    1430,1437,1445,1452,1459,1466,1473,1480,1487,1493,1500,1506,1512,1518,1524,1530,
    1536,1542,1547,1553,1558,1564,1569,1574,1580,1585,1590,1595,1599,1604,1609,1614,
    1618,1623,1628,1632,1636,1641,1645,1649,1654,1658,1662,1666,1670,1674,1678,1682,
    1686,1690,1693,1697,1701,1705,1708,1712,1715,1719,1722,1726,1729,1733,1736,1739,
    1743,1746,1749,1752,1756,1759,1762,1765,1768,1771,1774,1777,1780,1783,1786,1789,
    1792,1795,1798,1801,1803,1806,1809,1812,1814,1817,1820,1822,1825,1828,1830,1833,
    1836,1838,1841,1843,1846,1848,1851,1853,1855,1858,1860,1863,1865,1867,1870,1872,
    1874,1877,1879,1881,1884,1886,1888,1890,1892,1895,1897,1899,1901,1903,1905,1908,
    1910,1912,1914,1916,1918,1920,1922,1924,1926,1928,1930,1932,1934,1936,1938,1940,
    1942,1944,1946,1947,1949,1951,1953,1955,1957,1959,1961,1962,1964,1966,1968,1970,
    1971,1973,1975,1977,1978,1980,1982,1984,1985,1987,1989,1990,1992,1994,1995,1997,
    1999,2000,2002,2004,2005,2007,2008,2010,2012,2013,2015,2016,2018,2020,2021,2023,
    2024,2026,2027,2029,2030,2032,2033,2035,2036,2038,2039,2041,2042,2044,2045,2047,
    2048,
};

static void uart_putc(char c)
{
    while (!(*UART_LSR & LSR_THRE)) {}
    *UART_RBR_THR = (unsigned char)c;
}

static int uart_getc_nonblocking(char *out)
{
    if (*UART_LSR & LSR_DR) {
        *out = (char)(*UART_RBR_THR);
        return 1;
    }
    return 0;
}

static void print_str(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_send_u32(unsigned int packet)
{
    int i;
    for (i = 3; i >= 0; --i) {
        while (!(*UART_LSR & LSR_THRE)) {}
        *UART_RBR_THR = (unsigned char)((packet >> (i * 8)) & 0xFFU);
    }
}

static void uart_send_bytes(const unsigned char *data, unsigned int length)
{
    unsigned int i;
    for (i = 0; i < length; ++i) {
        while (!(*UART_LSR & LSR_THRE)) {}
        *UART_RBR_THR = data[i];
    }
}

static unsigned int abs_int(int v)
{
    return (unsigned int)(v < 0 ? -v : v);
}

static int ceil_div_int(int a, int b)
{
    return (a + b - 1) / b;
}

static unsigned int contrast_score_at(int x, int y)
{
    unsigned char *img = (unsigned char *)IMAGE_ADDRESS;
    int p_top = img[(y - 8) * IMG_WIDTH + x];
    int p_down = img[(y + 8) * IMG_WIDTH + x];
    int p_left = img[y * IMG_WIDTH + (x - 8)];
    int p_right = img[y * IMG_WIDTH + (x + 8)];
    unsigned int dv = abs_int(p_top - p_down);
    unsigned int dh = abs_int(p_left - p_right);
    return dv + dh;
}

static void build_cloud_score_grid(void)
{
    int gx, gy, x, y;
    int grid_w = 0, grid_h = 0;

    for (gx = 0; gx < CLOUD_MAX_GRID * CLOUD_MAX_GRID; ++gx)
        cloud_score_grid[gx] = 0;
    for (gx = 0; gx <= CLOUD_SCORE_MAX; ++gx)
        score_histogram[gx] = 0;

    gy = 0;
    for (y = CLOUD_MARGIN; y < IMG_HEIGHT - CLOUD_MARGIN; y += CLOUD_STEP) {
        if (gy >= CLOUD_MAX_GRID) break;
        gx = 0;
        for (x = CLOUD_MARGIN; x < IMG_WIDTH - CLOUD_MARGIN; x += CLOUD_STEP) {
            unsigned int score;
            if (gx >= CLOUD_MAX_GRID) break;
            score = contrast_score_at(x, y);
            cloud_score_grid[gy * CLOUD_MAX_GRID + gx] = (unsigned short)score;
            ++gx;
        }
        grid_w = gx;
        ++gy;
    }
    grid_h = gy;

    for (gy = 0; gy < grid_h; ++gy) {
        for (gx = 0; gx < grid_w; ++gx) {
            unsigned int score = cloud_score_grid[gy * CLOUD_MAX_GRID + gx];
            int nx, ny;
            int is_max = 1;
            int has_lower = 0;
            if (score <= CLOUD_THRESHOLD) continue;

            for (ny = gy - 1; ny <= gy + 1; ++ny) {
                for (nx = gx - 1; nx <= gx + 1; ++nx) {
                    unsigned int nscore;
                    if (nx < 0 || ny < 0 || nx >= grid_w || ny >= grid_h) continue;
                    if (nx == gx && ny == gy) continue;
                    nscore = cloud_score_grid[ny * CLOUD_MAX_GRID + nx];
                    if (nscore > score) is_max = 0;
                    else if (nscore < score) has_lower = 1;
                }
            }
            if (is_max && has_lower && score <= CLOUD_SCORE_MAX)
                ++score_histogram[score];
        }
    }
}

static int cloud_grid_cell_is_local_max(int gx, int gy, int grid_w, int grid_h)
{
    unsigned int score = cloud_score_grid[gy * CLOUD_MAX_GRID + gx];
    int nx, ny;
    int is_max = 1;
    int has_lower = 0;

    if (score <= CLOUD_THRESHOLD) return 0;
    for (ny = gy - 1; ny <= gy + 1; ++ny) {
        for (nx = gx - 1; nx <= gx + 1; ++nx) {
            unsigned int nscore;
            if (nx < 0 || ny < 0 || nx >= grid_w || ny >= grid_h) continue;
            if (nx == gx && ny == gy) continue;
            nscore = cloud_score_grid[ny * CLOUD_MAX_GRID + nx];
            if (nscore > score) is_max = 0;
            else if (nscore < score) has_lower = 1;
        }
    }
    return is_max && has_lower;
}

static unsigned int collect_top_landmarks(unsigned int requested)
{
    int gx, gy, x, y;
    int grid_w = 0, grid_h = 0;
    unsigned int cumulative = 0;
    int cutoff = CLOUD_THRESHOLD + 1;
    unsigned int out = 0;

    if (requested > TOP_N_L1) requested = TOP_N_L1;
    build_cloud_score_grid();

    for (y = CLOUD_MARGIN; y < IMG_HEIGHT - CLOUD_MARGIN; y += CLOUD_STEP) ++grid_h;
    for (x = CLOUD_MARGIN; x < IMG_WIDTH - CLOUD_MARGIN; x += CLOUD_STEP) ++grid_w;

    for (gy = CLOUD_SCORE_MAX; gy >= CLOUD_THRESHOLD + 1; --gy) {
        cumulative += score_histogram[gy];
        if (cumulative >= requested) {
            cutoff = gy;
            break;
        }
    }

    for (gy = 0; gy < grid_h && out < requested; ++gy) {
        for (gx = 0; gx < grid_w && out < requested; ++gx) {
            unsigned int score = cloud_score_grid[gy * CLOUD_MAX_GRID + gx];
            if (!cloud_grid_cell_is_local_max(gx, gy, grid_w, grid_h)) continue;
            if ((int)score < cutoff) continue;

            landmarks[out].x = (unsigned short)(CLOUD_MARGIN + gx * CLOUD_STEP);
            landmarks[out].y = (unsigned short)(CLOUD_MARGIN + gy * CLOUD_STEP);
            landmarks[out].score_full = (unsigned short)score;
            ++out;
        }
    }

    landmark_count = out;
    return out;
}

static void send_l0_cloud(void)
{
    unsigned int i;
    unsigned int count = collect_top_landmarks(TOP_N_L0);

    /* Header: marker | actual count(12) | step(4) | threshold(8). */
    uart_send_u32(
        ((unsigned int)MARKER_L0_HEADER << 24) |
        ((count & 0x0FFFU) << 12) |
        ((CLOUD_STEP & 0x0FU) << 8) |
        (CLOUD_THRESHOLD & 0xFFU));

    for (i = 0; i < count; ++i) {
        unsigned int score4 = ((unsigned int)landmarks[i].score_full >> 4) & 0x0FU;
        unsigned int packet =
            ((unsigned int)MARKER_L0_POINT << 24) |
            (((unsigned int)landmarks[i].x & 0x03FFU) << 14) |
            (((unsigned int)landmarks[i].y & 0x03FFU) << 4) |
            score4;
        uart_send_u32(packet);
    }
    uart_send_u32(0xFE000000U);
}

static void send_l1_cloud(void)
{
    unsigned int i;
    unsigned int count = collect_top_landmarks(TOP_N_L1);

    /* Header: marker | actual count(12) | step(4) | threshold(8). */
    uart_send_u32(
        ((unsigned int)MARKER_L1_HEADER << 24) |
        ((count & 0x0FFFU) << 12) |
        ((CLOUD_STEP & 0x0FU) << 8) |
        (CLOUD_THRESHOLD & 0xFFU));

    /* Each L1 point is packed into 20 bits and sent in 3 bytes.
     * bit layout: X[9:0] | Y[9:0] | 4 reserved zero bits. */
    for (i = 0; i < count; ++i) {
        unsigned int x = landmarks[i].x & 0x03FFU;
        unsigned int y = landmarks[i].y & 0x03FFU;
        unsigned char b0 = (unsigned char)(x >> 2);
        unsigned char b1 = (unsigned char)(((x & 0x03U) << 6) | (y >> 4));
        unsigned char b2 = (unsigned char)((y & 0x0FU) << 4);
        uart_putc((char)b0);
        uart_putc((char)b1);
        uart_putc((char)b2);
    }
    uart_send_u32(0xFE000000U);
}

static int count_occupied_blocks(int block_size)
{
    int cols = ceil_div_int(IMG_WIDTH, block_size);
    int rows = ceil_div_int(IMG_HEIGHT, block_size);
    unsigned int i;
    int count = 0;

    for (i = 0; i < MAX_ROI_CELLS; ++i) block_seen[i] = 0;

    for (i = 0; i < landmark_count; ++i) {
        int col = landmarks[i].x / block_size;
        int row = landmarks[i].y / block_size;
        int index;
        if (col >= cols || row >= rows) continue;
        index = row * MAX_ROI_GRID + col;
        if (!block_seen[index]) {
            block_seen[index] = 1;
            ++count;
        }
    }
    return count;
}

static int choose_adaptive_block_size(void)
{
    if (count_occupied_blocks(BLOCK_16) <= ROI_TARGET_BLOCKS) return BLOCK_16;
    if (count_occupied_blocks(BLOCK_8) <= ROI_TARGET_BLOCKS) return BLOCK_8;
    return BLOCK_4;
}

static void build_roi_mask(int block_size)
{
    int cols = ceil_div_int(IMG_WIDTH, block_size);
    int rows = ceil_div_int(IMG_HEIGHT, block_size);
    int radius_blocks = ceil_div_int(ROI_RADIUS_PIXELS, block_size);
    unsigned int i;

    for (i = 0; i < MAX_ROI_CELLS; ++i) roi_mask[i] = 0;

    for (i = 0; i < landmark_count; ++i) {
        int center_col = landmarks[i].x / block_size;
        int center_row = landmarks[i].y / block_size;
        int dr, dc;
        for (dr = -radius_blocks; dr <= radius_blocks; ++dr) {
            for (dc = -radius_blocks; dc <= radius_blocks; ++dc) {
                int rr = center_row + dr;
                int cc = center_col + dc;
                if (rr < 0 || cc < 0 || rr >= rows || cc >= cols) continue;
                roi_mask[rr * MAX_ROI_GRID + cc] = 1;
            }
        }
    }
}

static unsigned int calculate_block_entropy_x100(int x0, int y0, int block_w, int block_h)
{
    unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;
    int used_bins_count = 0;
    int num_pixels = block_w * block_h;
    int y, x;
    unsigned int total_q8 = 0;
    unsigned int log2_num_pixels_q8 = log2_q8[num_pixels];

    for (y = 0; y < block_h; ++y) {
        unsigned int base = (unsigned int)(y0 + y) * IMG_WIDTH + (unsigned int)x0;
        for (x = 0; x < block_w; ++x) {
            unsigned int intensity = grayscale_pixels[base + (unsigned int)x];
            if (entropy_histogram[intensity] == 0)
                entropy_used_bins[used_bins_count++] = (unsigned char)intensity;
            ++entropy_histogram[intensity];
        }
    }

    for (x = 0; x < used_bins_count; ++x) {
        unsigned int bin = entropy_used_bins[x];
        unsigned int count = entropy_histogram[bin];
        unsigned int delta = log2_num_pixels_q8 - log2_q8[count];
        total_q8 += count * delta;
        entropy_histogram[bin] = 0;
    }

    return (total_q8 * 100U) / ((unsigned int)num_pixels * 256U);
}

static void send_l2_entropy(void)
{
    int block_size;
    int cols;
    int rows;
    int row, col;
    unsigned int total_blocks;

    /* L1 determines only the analysis resolution; L2 itself is a FULL map. */
    landmark_count = collect_top_landmarks(TOP_N_L1);
    block_size = choose_adaptive_block_size();
    cols = ceil_div_int(IMG_WIDTH, block_size);
    rows = ceil_div_int(IMG_HEIGHT, block_size);
    total_blocks = (unsigned int)(rows * cols);

    uart_send_u32(
        ((unsigned int)MARKER_L2_HEADER << 24) |
        ((unsigned int)(block_size & 0xFF) << 16) |
        ((unsigned int)(cols & 0xFF) << 8) |
        ((unsigned int)(rows & 0xFF)));

    uart_send_u32(((unsigned int)MARKER_L2_COUNT << 24) | (total_blocks & 0xFFFFU));

    /* Every block is sent in raster order: top-to-bottom, left-to-right. */
    for (row = 0; row < rows; ++row) {
        for (col = 0; col < cols; ++col) {
            int x0 = col * block_size;
            int y0 = row * block_size;
            int bw = (x0 + block_size > IMG_WIDTH) ? (IMG_WIDTH - x0) : block_size;
            int bh = (y0 + block_size > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : block_size;
            unsigned int entropy = calculate_block_entropy_x100(x0, y0, bw, bh);
            unsigned int entropy_x10 = (entropy + 5U) / 10U;
            unsigned int packet =
                ((unsigned int)MARKER_L2_ENTROPY << 24) |
                (((unsigned int)col & 0xFFU) << 16) |
                (((unsigned int)row & 0xFFU) << 8) |
                (entropy_x10 & 0xFFU);
            uart_send_u32(packet);
        }
    }

    uart_send_u32(0xFE000000U);
}

static void stream_block_pixels(int x0, int y0, int bw, int bh)
{
    unsigned char *img = (unsigned char *)IMAGE_ADDRESS;
    int y;
    for (y = 0; y < bh; ++y) {
        const unsigned char *row_ptr = &img[(y0 + y) * IMG_WIDTH + x0];
        uart_send_bytes(row_ptr, (unsigned int)bw);
    }
}

static void send_l3_roi_image(void)
{
    int block_size;
    int cols;
    int rows;
    int row, col;
    unsigned int selected_count = 0;

    landmark_count = collect_top_landmarks(TOP_N_L1);
    block_size = choose_adaptive_block_size();
    cols = ceil_div_int(IMG_WIDTH, block_size);
    rows = ceil_div_int(IMG_HEIGHT, block_size);
    build_roi_mask(block_size);

    for (row = 0; row < rows; ++row) {
        for (col = 0; col < cols; ++col) {
            int index = row * MAX_ROI_GRID + col;
            roi_keep[index] = 0;
            if (roi_mask[index]) {
                int x0 = col * block_size;
                int y0 = row * block_size;
                int bw = (x0 + block_size > IMG_WIDTH) ? (IMG_WIDTH - x0) : block_size;
                int bh = (y0 + block_size > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : block_size;
                unsigned int entropy = calculate_block_entropy_x100(x0, y0, bw, bh);
                if (entropy >= CUT_ENTROPY_THRESHOLD_X100) {
                    roi_keep[index] = 1;
                    ++selected_count;
                }
            }
        }
    }

    uart_send_u32(
        ((unsigned int)MARKER_L3_HEADER << 24) |
        ((unsigned int)(block_size & 0xFF) << 16) |
        ((unsigned int)(cols & 0xFF) << 8) |
        ((unsigned int)(rows & 0xFF)));
    uart_send_u32(((unsigned int)MARKER_L3_META << 24) | (CUT_ENTROPY_THRESHOLD_X100 & 0xFFFFU));
    uart_send_u32(((unsigned int)MARKER_L3_COUNT << 24) | (selected_count & 0xFFFFU));

    for (row = 0; row < rows; ++row) {
        for (col = 0; col < cols; ++col) {
            int index = row * MAX_ROI_GRID + col;
            if (roi_keep[index]) {
                int x0 = col * block_size;
                int y0 = row * block_size;
                int bw = (x0 + block_size > IMG_WIDTH) ? (IMG_WIDTH - x0) : block_size;
                int bh = (y0 + block_size > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : block_size;
                uart_send_u32(
                    ((unsigned int)MARKER_L3_BLOCK << 24) |
                    (((unsigned int)col & 0xFFU) << 16) |
                    (((unsigned int)row & 0xFFU) << 8));
                stream_block_pixels(x0, y0, bw, bh);
            }
        }
    }
    uart_send_u32(0xFE000000U);
}

static void send_full_image(void)
{
    int cols = ceil_div_int(IMG_WIDTH, L4_BLOCK_SIZE);
    int rows = ceil_div_int(IMG_HEIGHT, L4_BLOCK_SIZE);
    int row, col;

    /* C0: block_size | cols | rows. Payload is a sequence of C3+raw blocks. */
    uart_send_u32(
        ((unsigned int)MARKER_FULL_HEADER << 24) |
        ((unsigned int)(L4_BLOCK_SIZE & 0xFF) << 16) |
        ((unsigned int)(cols & 0xFF) << 8) |
        ((unsigned int)(rows & 0xFF)));

    for (row = 0; row < rows; ++row) {
        for (col = 0; col < cols; ++col) {
            int x0 = col * L4_BLOCK_SIZE;
            int y0 = row * L4_BLOCK_SIZE;
            int bw = (x0 + L4_BLOCK_SIZE > IMG_WIDTH) ? (IMG_WIDTH - x0) : L4_BLOCK_SIZE;
            int bh = (y0 + L4_BLOCK_SIZE > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : L4_BLOCK_SIZE;

            /* Raster order is explicit: row first, then column. */
            uart_send_u32(
                ((unsigned int)MARKER_FULL_BLOCK << 24) |
                (((unsigned int)col & 0xFFU) << 16) |
                (((unsigned int)row & 0xFFU) << 8));
            stream_block_pixels(x0, y0, bw, bh);
        }
    }
    uart_send_u32(0xFE000000U);
}

static int text_equals(const char *a, const char *b)
{
    while (*a && *b) {
        if (*a != *b) return 0;
        ++a;
        ++b;
    }
    return (*a == '\0' && *b == '\0');
}

static int decode_command(const char *cmd)
{
    if (text_equals(cmd, "AURA L0"))       return 10;
    if (text_equals(cmd, "AURA L1"))       return 11;
    if (text_equals(cmd, "AURA L2"))       return 12;
    if (text_equals(cmd, "AURA L3"))       return 13;
    if (text_equals(cmd, "AURA L4"))       return 14;
    if (text_equals(cmd, "AURA CLOUD"))    return 11;
    if (text_equals(cmd, "AURA ENTROPY"))  return 12;
    if (text_equals(cmd, "AURA CUT"))      return 13;
    if (text_equals(cmd, "AURA FULL"))     return 14;
    return 0;
}

static int uart_poll_command(char *cmd_buffer, int *cmd_pos)
{
    char c;
    while (uart_getc_nonblocking(&c)) {
        if (c == '\r' || c == '\n') {
            int mode;
            cmd_buffer[*cmd_pos] = '\0';
            mode = decode_command(cmd_buffer);
            *cmd_pos = 0;
            return mode;
        }
        if (*cmd_pos < 31) cmd_buffer[(*cmd_pos)++] = c;
        else *cmd_pos = 0;
    }
    return 0;
}

int main(void)
{
    char cmd_buffer[32];
    int cmd_pos = 0;

    print_str("\n===================================\n");
    print_str("AURA Firmware V0.4.1: Interactive Telemetry Core\n");
    print_str("L0=Top100+score L1=Top1000(XY) L2=FULL entropy L3=ROI L4=FULL blocks\n");
    print_str("===================================\n\n");

    while (1) {
        int mode = uart_poll_command(cmd_buffer, &cmd_pos);
        if (mode == 10) {
            uart_send_u32(0xAC00000AU);
            send_l0_cloud();
        } else if (mode == 11) {
            uart_send_u32(0xAC00000BU);
            send_l1_cloud();
        } else if (mode == 12) {
            uart_send_u32(0xAC00000CU);
            send_l2_entropy();
        } else if (mode == 13) {
            uart_send_u32(0xAC00000DU);
            send_l3_roi_image();
        } else if (mode == 14) {
            uart_send_u32(0xAC00000EU);
            send_full_image();
        }
    }

    return 0;
}
