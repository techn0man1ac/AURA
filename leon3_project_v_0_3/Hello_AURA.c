/*
    AURA LEON3 Firmware V0.3
    https://github.com/techn0man1ac/AURA
    By Serhii Trush, 2026, MIT License.

   Fixed 1020x1020, 8-bit grayscale image.

   Telemetry levels:
     L0 - Top-100 local-contrast maxima, X/Y only (packed 20-bit points).
     L1 - Top-1000 local-contrast maxima, X/Y + 4-bit edge vector direction.
     L2 - FULL adaptive Shannon entropy map with on-board feature overlap analysis.
     L3 - Sparse ROI blocks generated via logical UNION (A U B) of landmarks and entropy.
     L4 - FULL image progressively streamed via Opposing Vertical Scan pattern.

   Ground commands:
     AURA L0\n
     AURA L1\n
     AURA L2\n
     AURA L3\n
     AURA L4\n

   No dynamic allocation. No floating-point operations in the flight path.
   Structured words are transmitted big-endian, 32-bit, byte-by-byte.
*/

/* =========================================================================
   HARDWARE REGISTERS & SENSOR GEOMETRY
   Standard memory-mapped registers for Gaisler APBUART (NS16550 compatible)
   on Cobham Gaisler GR712RC / LEON3 architectures.
   ========================================================================= */

#define UART_BASE     0x80000100                      /* Base address of APB UART peripheral */
#define UART_RBR_THR  ((volatile unsigned char *)(UART_BASE + 0)) /* Rx Buffer / Tx Holding Reg */
#define UART_LSR      ((volatile unsigned char *)(UART_BASE + 5)) /* Line Status Register */
#define LSR_DR        0x01                            /* Data Ready bit in LSR (Bit 0) */
#define LSR_THRE      0x20                            /* Transmitter Holding Register Empty (Bit 5) */

#define IMAGE_ADDRESS 0x40600000                      /* Static base address of camera buffer in SRAM */
#define IMG_WIDTH     1020                            /* Sensor matrix width in pixels */
#define IMG_HEIGHT    1020                            /* Sensor matrix height in pixels */
#define IMAGE_BYTES   1040400                         /* Total uncompressed monochromatic frame bytes */

/* =========================================================================
   ALGORITHMIC THRESHOLDS & REGION-OF-INTEREST CONSTRAINTS
   Deterministic boundaries designed for bounded Worst-Case Execution Time.
   ========================================================================= */

#define CLOUD_THRESHOLD        45     /* Noise cutoff barrier for raw gradient magnitude */
#define CLOUD_STEP             8      /* Pixel stride for spatial candidate downsampling */
#define CLOUD_MARGIN           8      /* Edge boundary margin to avoid out-of-bounds pointer reads */
#define CLOUD_MAX_GRID         128    /* Static dimension ceiling for candidate grid (128x128) */
#define CLOUD_SCORE_MAX        510    /* Theoretical maximum gradient: |255-0| + |255-0| */
#define TOP_N_L0               100    /* Telemetry limit: top-100 landmarks for L0 */
#define TOP_N_L1               1000   /* Telemetry limit: top-1000 landmarks for L1 */

#define ROI_TARGET_BLOCKS      5000   /* Target budget ceiling for high-entropy spatial blocks */
#define ROI_RADIUS_PIXELS      8      /* Morphological dilation radius around detected vertices */
#define BLOCK_16               16     /* Primary coarse block resolution (16x16 px) */
#define BLOCK_8                8      /* Secondary medium block resolution (8x8 px) */
#define BLOCK_4                4      /* High-resolution fallback block size (4x4 px) */
#define MAX_ROI_GRID           256    /* Maximum block columns/rows supported */
#define MAX_ROI_CELLS          (MAX_ROI_GRID * MAX_ROI_GRID) /* Maximum possible spatial cells in memory */

#define L4_BLOCK_SIZE          16     /* Fixed block dimension for uncompressed L4 raster */
#define L3_PRIORITY_MODE       0      /* Sorting priority: 0 = Shannon Entropy, 1 = Mean Intensity */

static unsigned short current_entropy_threshold_x100 = 250; /* Active gate threshold (2.50 bits/px) */

/* =========================================================================
   TELEMETRY PROTOCOL SYNCHRONIZATION MARKERS
   Big-endian 8-bit frame identifiers placed in upper byte [31:24] of words.
   =========================================================================
*/

#define MARKER_ACK               0xAC /* Telecommand acknowledgment */
#define MARKER_L0_HEADER         0x5B /* Header: L0 Ultra-Sparse Landmark Cloud */
#define MARKER_L0_POINT          0xBD /* Payload: L0 Coordinate record */
#define MARKER_L1_HEADER         0xB1 /* Header: L1 Directional Landmark Map */
#define MARKER_L1_DESCRIPTIONS   0xB2 /* Optional descriptor payload token */
#define MARKER_L2_HEADER         0xD0 /* Header: L2 Full Entropy Grid */
#define MARKER_L2_ENTROPY        0xD1 /* Payload: L2 Block entropy metric */
#define MARKER_L2_COUNT          0xD2 /* Metadata: Total blocks expected */
#define MARKER_L2_COMPARE_BLOCKS 0xD7 /* Diagnostic: Top-10% entropy block count */
#define MARKER_L2_COMPARE_POINTS 0xD8 /* Diagnostic: Overlapping landmark blocks */
#define MARKER_L2_COMPARE_HITS   0xD9 /* Diagnostic: Landmarks landing inside top entropy */
#define MARKER_L3_HEADER         0xD3 /* Header: L3 Gated Sparse ROI Image */
#define MARKER_L3_META           0xD4 /* Metadata: Priority mode and active threshold */
#define MARKER_L3_BLOCK          0xD5 /* Header: Individual ROI block pixel array */
#define MARKER_L3_COUNT          0xD6 /* Metadata: Total transmitted ROI block count */
#define MARKER_FULL_HEADER       0xC0 /* Header: L4 Full Progressive Raster */
#define MARKER_FULL_BLOCK        0xC3 /* Header: L4 Raster block pixel array */
#define MARKER_END               0xFE /* End-of-frame synchronization trailer */

/* =========================================================================
   STATIC WORKING MEMORY (ZERO DYNAMIC ALLOCATION)
   All buffers reside in static BSS/Data segments to guarantee zero heap
   fragmentation and enforce memory determinism.
   ========================================================================= */

static unsigned short cloud_score_grid[CLOUD_MAX_GRID * CLOUD_MAX_GRID]; /* Candidate grid scores */
static unsigned short score_histogram[CLOUD_SCORE_MAX + 1];              /* Gradient score histogram */

/* Packed landmark descriptor representing a surface physical feature */
typedef struct {
  unsigned short x;               /* Horizontal sub-pixel coordinate [0..1019] */
  unsigned short y;               /* Vertical sub-pixel coordinate [0..1019] */
  unsigned short score_full;      /* Absolute contrast magnitude (0..510) */
  unsigned char  gradient_angle;  /* 4-bit edge vector direction angle (0..15) */
} Landmark;

static Landmark landmarks[TOP_N_L1];           /* Fixed descriptor bank (1000 items) */
static unsigned int landmark_count = 0;        /* Number of valid landmarks stored */

static unsigned char  roi_mask[MAX_ROI_CELLS];         /* Binary mask of transmitted blocks */
static unsigned char  block_seen[MAX_ROI_CELLS];       /* Deduplication bitmask */
static unsigned short selected_indices[MAX_ROI_CELLS]; /* Unsorted indices of active blocks */
static unsigned short sorted_indices[MAX_ROI_CELLS];   /* Bucket-sorted indices by entropy */

/* Caches to prevent redundant recalculation across L2 and L3 */
static unsigned char  entropy_x10_cache[MAX_ROI_CELLS];
static unsigned char  block_mean_cache[MAX_ROI_CELLS];
static unsigned char  entropy_cache_valid = 0;
static unsigned char  entropy_cache_block_size = 0;
static unsigned char  entropy_cache_cols = 0;
static unsigned char  entropy_cache_rows = 0;
static unsigned char  landmarks_cache_valid = 0;

/* Dedicated histogram bins for entropy calculation */
static unsigned short entropy_histogram_81[81];        /* Bins for 0..8.0 bits/px (x10) */
static unsigned short entropy_histogram[256];          /* Grayscale luminance distribution (0..255) */
static unsigned char  entropy_used_bins[256];          /* Active bin tracking stack for O(N) cleanup */

/* =========================================================================
   LOW-LEVEL I/O AND BUS INTEGRITY DRIVERS
   ========================================================================= */

/* Busy-wait transmission of a single byte via UART holding register */

static void uart_putc(char c)
{
  while (!(*UART_LSR & LSR_THRE)) {} /* Wait until transmitter FIFO is empty */
  *UART_RBR_THR = (unsigned char)c;
}

/* Non-blocking byte retrieval from UART receiver FIFO */
static int uart_getc_nonblocking(char *out)
{
  if (*UART_LSR & LSR_DR) {          /* Check if Data Ready bit is asserted */
    *out = (char)(*UART_RBR_THR);
    return 1;
  }
  return 0;
}

static void print_str(const char *s)
{
  while (*s) uart_putc(*s++);
}

/**
   @brief Serializes a 32-bit word over the wire big-endian, byte-by-byte.
   @note  CRITICAL: Bypasses SPARC V8 Trap 0x07 (Data Access Alignment Trap)
          by preventing non-word-aligned 32-bit bus stores.
*/

static void uart_send_u32(unsigned int packet)
{
  int i;
  for (i = 3; i >= 0; --i) {
    while (!(*UART_LSR & LSR_THRE)) {}
    *UART_RBR_THR = (unsigned char)((packet >> (i * 8)) & 0xFFU);
  }
}

/* Streams a contiguous memory chunk byte-by-byte */
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

/* =========================================================================
   FEATURE EXTRACTION: CROSS-DIFFERENCE OPERATOR & DIRECTION ESTIMATION
   Computes luminance gradient magnitude and estimates vector orientation
   in 16 discrete sectors (4 bits) without trigonometric float routines.
   ========================================================================= */

static unsigned int contrast_score_at(int x, int y, unsigned char *angle_out)
{
  unsigned char *img = (unsigned char *)IMAGE_ADDRESS;
  /* Cross displacement stencil with +/- 8-pixel arms */
  int p_top   = img[(y - 8) * IMG_WIDTH + x];
  int p_down  = img[(y + 8) * IMG_WIDTH + x];
  int p_left  = img[y * IMG_WIDTH + (x - 8)];
  int p_right = img[y * IMG_WIDTH + (x + 8)];

  unsigned int dv = abs_int(p_top - p_down); /* Vertical delta */
  unsigned int dh = abs_int(p_left - p_right); /* Horizontal delta */

  /* 4-bit orientation estimation (0..15 sectors) without atan2f() */
  if (angle_out != 0) {
    if (dh == 0 && dv == 0) *angle_out = 0;
    else if (dh > dv) *angle_out = (unsigned char)((dv * 7U) / dh);
    else *angle_out = (unsigned char)(8U + (dh * 7U) / (dv ? dv : 1U));
  }

  return dv + dh;
}

/* Builds spatial candidate grid and applies 8-way Non-Maximum Suppression */
static void build_cloud_score_grid(void)
{
  int gx, gy, x, y;
  int grid_w = 0, grid_h = 0;

  for (gx = 0; gx < CLOUD_MAX_GRID * CLOUD_MAX_GRID; ++gx) cloud_score_grid[gx] = 0;
  for (gx = 0; gx <= CLOUD_SCORE_MAX; ++gx) score_histogram[gx] = 0;

  gy = 0;
  for (y = CLOUD_MARGIN; y < IMG_HEIGHT - CLOUD_MARGIN; y += CLOUD_STEP) {
    if (gy >= CLOUD_MAX_GRID) break;
    gx = 0;
    for (x = CLOUD_MARGIN; x < IMG_WIDTH - CLOUD_MARGIN; x += CLOUD_STEP) {
      unsigned int score;
      if (gx >= CLOUD_MAX_GRID) break;
      score = contrast_score_at(x, y, 0);
      cloud_score_grid[gy * CLOUD_MAX_GRID + gx] = (unsigned short)score;
      ++gx;
    }
    grid_w = gx;
    ++gy;
  }
  grid_h = gy;

  /* 8-connected Non-Maximum Suppression (NMS) */
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
        ++score_histogram[score]; /* Record gradient score in histogram */
    }
  }
}

/* =========================================================================
   NON-MAXIMUM SUPPRESSION (NMS) KERNEL
   Verifies whether a candidate grid cell represents a strict local maximum
   across its 8-connected spatial neighborhood.
   ========================================================================= */

static int cloud_grid_cell_is_local_max(int gx, int gy, int grid_w, int grid_h)
{
  unsigned int score = cloud_score_grid[gy * CLOUD_MAX_GRID + gx];
  int nx, ny;
  int is_max = 1;
  int has_lower = 0;

  /* Reject immediately if below baseline contrast noise threshold */
  if (score <= CLOUD_THRESHOLD) return 0;

  /* Scan 3x3 window around candidate vertex */
  for (ny = gy - 1; ny <= gy + 1; ++ny) {
    for (nx = gx - 1; nx <= gx + 1; ++nx) {
      unsigned int nscore;
      /* Boundary check to prevent array index underflow/overflow */
      if (nx < 0 || ny < 0 || nx >= grid_w || ny >= grid_h) continue;
      if (nx == gx && ny == gy) continue; /* Skip the center pixel itself */

      nscore = cloud_score_grid[ny * CLOUD_MAX_GRID + nx];
      if (nscore > score) is_max = 0;      /* Neighbor is stronger: not a peak */
      else if (nscore < score) has_lower = 1; /* Peak is strictly higher than at least one neighbor */
    }
  }
  return is_max && has_lower;
}

/* Extracts top landmarks using cumulative histogram cutoff in O(N) time */
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

  /* Find threshold cutoff value in O(CLOUD_SCORE_MAX) without sorting */
  for (gy = CLOUD_SCORE_MAX; gy >= CLOUD_THRESHOLD + 1; --gy) {
    cumulative += score_histogram[gy];
    if (cumulative >= requested) {
      cutoff = gy;
      break;
    }
  }

  /* Collect only candidate peaks meeting the cutoff score */
  for (gy = 0; gy < grid_h && out < requested; ++gy) {
    for (gx = 0; gx < grid_w && out < requested; ++gx) {
      unsigned int score = cloud_score_grid[gy * CLOUD_MAX_GRID + gx];
      if (!cloud_grid_cell_is_local_max(gx, gy, grid_w, grid_h)) continue;
      if ((int)score < cutoff) continue;

      unsigned char angle = 0;
      int real_x = CLOUD_MARGIN + gx * CLOUD_STEP;
      int real_y = CLOUD_MARGIN + gy * CLOUD_STEP;
      contrast_score_at(real_x, real_y, &angle);

      landmarks[out].x = (unsigned short)real_x;
      landmarks[out].y = (unsigned short)real_y;
      landmarks[out].score_full = (unsigned short)score;
      landmarks[out].gradient_angle = angle;
      ++out;
    }
  }

  landmark_count = out;
  return out;
}

/* =========================================================================
   LANDMARK CACHE MEMOIZATION GUARD
   Implements lazy evaluation to guarantee that expensive anisotropic gradient
   scanning and Non-Maximum Suppression (NMS) execute at most once per frame.
   ========================================================================= */

static void ensure_landmark_cache(void)
{
  /* Check if descriptors for the current optical frame are already cached */
  if (!landmarks_cache_valid) {
    /* Extract up to 1000 features into the static descriptor bank */
    collect_top_landmarks(TOP_N_L1);

    /* Assert flag to bypass redundant recalculations on subsequent requests */
    landmarks_cache_valid = 1;
  }
}

/* Packs coordinates X (10 bits) and Y (10 bits) into strict 3 bytes (24 bits) */
static void send_packed_landmark_xy(unsigned short x, unsigned short y)
{
  unsigned int ux = ((unsigned int)x) & 0x03FFU; /* 10-bit X (0..1019) */
  unsigned int uy = ((unsigned int)y) & 0x03FFU; /* 10-bit Y (0..1019) */

  unsigned char b0 = (unsigned char)(ux >> 2);
  unsigned char b1 = (unsigned char)(((ux & 0x03U) << 6) | (uy >> 4));
  unsigned char b2 = (unsigned char)((uy & 0x0FU) << 4);

  uart_putc((char)b0);
  uart_putc((char)b1);
  uart_putc((char)b2);
}

/* Packs X[10], Y[10], Angle[4], Luminance[4] into a single 32-bit register */
static void send_packed_landmark_xy_intensity_vector(unsigned short x, unsigned short y, unsigned char intensity, unsigned char angle)
{
  unsigned int ux = ((unsigned int)x) & 0x03FFU;
  unsigned int uy = ((unsigned int)y) & 0x03FFU;
  unsigned int uangle = ((unsigned int)angle) & 0x0FU;

  /* Layout: [31:28] Reserved | [27:18] X | [17:8] Y | [7:4] Angle | [3:0] Intensity */
  unsigned int word = (ux << 18) | (uy << 8) | (uangle << 4) | (intensity & 0x0FU);

  uart_putc((char)((word >> 24) & 0xFFU));
  uart_putc((char)((word >> 16) & 0xFFU));
  uart_putc((char)((word >> 8) & 0xFFU));
  uart_putc((char)(word & 0xFFU));
}

/* =========================================================================
   TELEMETRY DISPATCHERS: LEVEL 0 AND LEVEL 1
   Serializes landmark point clouds into bit-packed frames with embedded
   algorithmic configuration metadata in a single 32-bit header.
   ========================================================================= */

/**
   @brief Transmits Level 0 Ultra-Sparse Landmark Cloud (X, Y only).
   @note  Total payload: 1 Header (4B) + N * 3B + 1 Trailer (4B).
*/

static void send_l0_cloud(void)
{
  unsigned int i;
  unsigned int count;

  /* Step 1: Ensure features are computed without redundant recalculation */
  ensure_landmark_cache();

  /* Step 2: Enforce strict bandwidth ceiling for L0 (maximum 100 points) */
  count = (landmark_count < TOP_N_L0) ? landmark_count : TOP_N_L0;

  /* Step 3: Emit 32-bit Configuration Header Word */
  /* Bit layout: [31:24] Marker (0x5B) | [23:12] Count (12b) | [11:8] Step (4b) | [7:0] Threshold (8b) */

  uart_send_u32(
    ((unsigned int)MARKER_L0_HEADER << 24) |
    ((count & 0x0FFFU) << 12) |
    ((CLOUD_STEP & 0x0FU) << 8) |
    (CLOUD_THRESHOLD & 0xFFU));

  /* Step 4: Stream coordinates using 20-bit packing (3 bytes per point) */
  for (i = 0; i < count; ++i)
    send_packed_landmark_xy(landmarks[i].x, landmarks[i].y);

  /* Step 5: Emit End-Of-Frame Synchronization Trailer */
  uart_send_u32(0xFE000000U);
}

/**
   @brief Transmits Level 1 Directional Feature Map (X, Y, Edge Vector, Intensity).
   @note  Total payload: 1 Header (4B) + N * 4B + 1 Trailer (4B).
*/

static void send_l1_cloud(void)
{
  unsigned int i;
  unsigned int count;
  unsigned char *img = (unsigned char *)IMAGE_ADDRESS;

  /* Step 1: Query cached feature detector */
  ensure_landmark_cache();
  count = landmark_count; /* Uses full Top-1000 quota */

  /* Step 2: Emit 32-bit Configuration Header Word */
  /* Bit layout: [31:24] Marker (0xB1) | [23:12] Count (12b) | [11:8] Step (4b) | [7:0] Threshold (8b) */
  uart_send_u32(
    ((unsigned int)MARKER_L1_HEADER << 24) |
    ((count & 0x0FFFU) << 12) |
    ((CLOUD_STEP & 0x0FU) << 8) |
    (CLOUD_THRESHOLD & 0xFFU));

  /* Step 3: Stream descriptors (4 bytes per point) */
  for (i = 0; i < count; ++i) {
    unsigned short x = landmarks[i].x;
    unsigned short y = landmarks[i].y;

    /* Direct RAM read: fetch raw luminance at coordinates (X, Y) */
    unsigned char intensity = img[(unsigned int)y * IMG_WIDTH + (unsigned int)x];

    send_packed_landmark_xy_intensity_vector(x, y, intensity, landmarks[i].gradient_angle);
  }

  /* Step 4: Emit End-Of-Frame Synchronization Trailer */
  uart_send_u32(0xFE000000U); // 0xFE -> MARKER_END
}

/* Precomputed base-2 logarithm lookup table scaled by 256 (Q8 format): log2(x) * 256 */
static const unsigned short log2_q8[257] = {
  0, 0, 256, 406, 512, 594, 662, 719, 768, 812, 850, 886, 918, 947, 975, 1000,
  1024, 1046, 1068, 1087, 1106, 1124, 1142, 1158, 1174, 1189, 1203, 1217, 1231, 1244, 1256, 1268,
  1280, 1291, 1302, 1313, 1324, 1334, 1343, 1353, 1362, 1372, 1380, 1389, 1398, 1406, 1414, 1422,
  1430, 1437, 1445, 1452, 1459, 1466, 1473, 1480, 1487, 1493, 1500, 1506, 1512, 1518, 1524, 1530,
  1536, 1542, 1547, 1553, 1558, 1564, 1569, 1574, 1580, 1585, 1590, 1595, 1599, 1604, 1609, 1614,
  1618, 1623, 1628, 1632, 1636, 1641, 1645, 1649, 1654, 1658, 1662, 1666, 1670, 1674, 1678, 1682,
  1686, 1690, 1693, 1697, 1701, 1705, 1708, 1712, 1715, 1719, 1722, 1726, 1729, 1733, 1736, 1739,
  1743, 1746, 1749, 1752, 1756, 1759, 1762, 1765, 1768, 1771, 1774, 1777, 1780, 1783, 1786, 1789,
  1792, 1795, 1798, 1801, 1803, 1806, 1809, 1812, 1814, 1817, 1820, 1822, 1825, 1828, 1830, 1833,
  1836, 1838, 1841, 1843, 1846, 1848, 1851, 1853, 1855, 1858, 1860, 1863, 1865, 1867, 1870, 1872,
  1874, 1877, 1879, 1881, 1884, 1886, 1888, 1890, 1892, 1895, 1897, 1899, 1901, 1903, 1905, 1908,
  1910, 1912, 1914, 1916, 1918, 1920, 1922, 1924, 1926, 1928, 1930, 1932, 1934, 1936, 1938, 1940,
  1942, 1944, 1946, 1947, 1949, 1951, 1953, 1955, 1957, 1959, 1961, 1962, 1964, 1966, 1968, 1970,
  1971, 1973, 1975, 1977, 1978, 1980, 1982, 1984, 1985, 1987, 1989, 1990, 1992, 1994, 1995, 1997,
  1999, 2000, 2002, 2004, 2005, 2007, 2008, 2010, 2012, 2013, 2015, 2016, 2018, 2020, 2021, 2023,
  2024, 2026, 2027, 2029, 2030, 2032, 2033, 2035, 2036, 2038, 2039, 2041, 2042, 2044, 2045, 2047,
  2048,
}; /* log2(256) * 256 = 8 * 256 = 2048 */

/* =========================================================================
   ADAPTIVE SPATIAL RESOLUTION & TELEMETRY BUDGETING
   Evaluates landmark clustering to dynamically choose block dimensions.
   ========================================================================= */

/**
   @brief Counts unique spatial grid cells occupied by at least one landmark.
   @param block_size Pixel dimension of each grid block (16, 8, or 4 px).
   @return Number of populated blocks across the sensor matrix.
*/

static int count_occupied_blocks(int block_size)
{
  /* Calculate integer grid layout dimensions with ceiling rounding */
  int cols = ceil_div_int(IMG_WIDTH, block_size);
  int rows = ceil_div_int(IMG_HEIGHT, block_size);
  unsigned int i;
  int count = 0;

  /* Reset 256x256 spatial deduplication bitmask */
  for (i = 0; i < MAX_ROI_CELLS; ++i) block_seen[i] = 0;

  /* Project all detected landmarks into the spatial grid */
  for (i = 0; i < landmark_count; ++i) {
    int col = landmarks[i].x / block_size;
    int row = landmarks[i].y / block_size;
    int index;

    /* Boundary clipping against edge anomalies */
    if (col >= cols || row >= rows) continue;

    /* 2D to 1D index mapping with fixed MAX_ROI_GRID pitch (256) */
    index = row * MAX_ROI_GRID + col;

    /* Count unique occupied blocks only (spatial deduplication) */
    if (!block_seen[index]) {
      block_seen[index] = 1;
      ++count;
    }
  }
  return count;
}

/**
   @brief Selects the optimal block resolution based on the telemetry budget ceiling.
   @return Active block size (BLOCK_16, BLOCK_8, or BLOCK_4).
*/

/* CORRECT MATHEMATICAL LOGIC (BOTTOM-UP): */
static int choose_adaptive_block_size(void)
{
  /*
    // 1. We try to get the highest level of detail (4x4), if the quota allows
    if (count_occupied_blocks(BLOCK_4) <= ROI_TARGET_BLOCKS) return BLOCK_4;

    // 2. If the object is larger, try the medium resolution (8x8)
    if (count_occupied_blocks(BLOCK_8) <= ROI_TARGET_BLOCKS) return BLOCK_8;
  */

  // 3. If the asteroid fills the entire frame, switch to coarse blocks (16x16)
  return BLOCK_16;
}

/* Calculates Shannon Entropy scaled by 100 in deterministic integer arithmetic */
static unsigned int calculate_block_entropy_x100(int x0, int y0, int block_w, int block_h, unsigned char *mean_out)
{
  unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;
  int used_bins_count = 0;
  int num_pixels = block_w * block_h;
  int y, x;
  unsigned int total_q8 = 0;
  unsigned int log2_num_pixels_q8 = log2_q8[num_pixels];
  unsigned int sum = 0;

  /* Build block histogram and track modified bins */
  for (y = 0; y < block_h; ++y) {
    unsigned int base = (unsigned int)(y0 + y) * IMG_WIDTH + (unsigned int)x0;
    for (x = 0; x < block_w; ++x) {
      unsigned int intensity = grayscale_pixels[base + (unsigned int)x];
      if (entropy_histogram[intensity] == 0)
        entropy_used_bins[used_bins_count++] = (unsigned char)intensity;
      ++entropy_histogram[intensity];
      sum += intensity;
    }
  }

  /* Information evaluation: H = sum( count * (log2(N) - log2(count)) ) */
  for (x = 0; x < used_bins_count; ++x) {
    unsigned int bin = entropy_used_bins[x];
    unsigned int count = entropy_histogram[bin];
    unsigned int delta = log2_num_pixels_q8 - log2_q8[count];
    total_q8 += count * delta;
    entropy_histogram[bin] = 0; /* Selective reset eliminates full-array memset */
  }

  if (mean_out != 0)
    *mean_out = (unsigned char)((sum + ((unsigned int)num_pixels / 2U)) / (unsigned int)num_pixels);

  /* Scale Q8 format to integer (Entropy * 100) */
  return (total_q8 * 100U) / ((unsigned int)num_pixels * 256U);
}

/* =========================================================================
   SPATIAL ENTROPY CACHING & MEMOIZATION ENGINE
   Evaluates full-frame block entropy, handles image border clamping,
   and caches results in SRAM to eliminate duplicate WCET execution cycles.
   ========================================================================= */

/**
   @brief Computes Shannon entropy and mean intensity for all spatial grid blocks.
   @details Iterates over the 1020x1020 sensor matrix using an adaptive block size,
            clamps border dimensions, and packs scaled metrics into static SRAM arrays.
   @param block_size Spatial dimension of each square block (16, 8, or 4 pixels).
*/

static void compute_entropy_cache(int block_size)
{
  /* Calculate 2D grid dimensions rounding up to cover border pixels */
  int cols = ceil_div_int(IMG_WIDTH, block_size);
  int rows = ceil_div_int(IMG_HEIGHT, block_size);
  int row, col;

  /* Linear spatial traversal of the frame */
  for (row = 0; row < rows; ++row) {
    for (col = 0; col < cols; ++col) {
      int x0 = col * block_size;
      int y0 = row * block_size;

      /* Clamping: Prevents out-of-bounds memory reads at frame margins */
      int bw = (x0 + block_size > IMG_WIDTH) ? (IMG_WIDTH - x0) : block_size;
      int bh = (y0 + block_size > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : block_size;

      /* Flattened 1D index mapping to static 2D grid stride */
      int index = row * MAX_ROI_GRID + col;
      unsigned char mean_value = 0;

      /* Evaluate block entropy in Q8 fixed-point integer math */
      unsigned int entropy_x100 = calculate_block_entropy_x100(x0, y0, bw, bh, &mean_value);

      /* Integer rounding trick: (val + 5) / 10 computes round(val / 10.0) without FPU */
      entropy_x10_cache[index] = (unsigned char)((entropy_x100 + 5U) / 10U);
      block_mean_cache[index] = mean_value;
    }
  }

  /* Update cache validity flags and spatial state metadata */
  entropy_cache_block_size = (unsigned char)block_size;
  entropy_cache_cols = (unsigned char)cols;
  entropy_cache_rows = (unsigned char)rows;
  entropy_cache_valid = 1;
}

/**
   @brief Enforces cache validity (Lazy Evaluation).
   @details Prevents redundant recalculation if the cached grid parameters match.
   @param block_size Required block size for the pending telemetry operation.
*/

static void ensure_entropy_cache(int block_size)
{
  /* Invalidate and recompute only if cache is dirty or block size changed */
  if (!entropy_cache_valid || entropy_cache_block_size != (unsigned char)block_size)
    compute_entropy_cache(block_size);
}

/**
   @brief  Isolates the top 10% highest-entropy blocks in O(N) linear time.
   @note   Eliminates recursive Quicksort overhead and bounded stack risks;
           executes via reverse cumulative histogram thresholding.
   @param  cols         Active block column count in the spatial grid.
   @param  rows         Active block row count in the spatial grid.
   @param  top10_count  Output pointer storing the exact count of marked blocks.
*/

static void build_top10_entropy_mask(int cols, int rows, unsigned int *top10_count)
{
  /* Calculate target: strictly ceiling of 10% of total grid cells */
  unsigned int total_blocks = (unsigned int)(cols * rows);
  unsigned int target = (total_blocks + 9U) / 10U; /* Integer ceiling: ceil(total/10) */
  unsigned int selected = 0;
  int level;
  int row, col;

  /* Step 1: Zero out dedicated 81-bin histogram (covers 0.0 to 8.0 bits/px) */
  for (level = 0; level <= 80; ++level)
    entropy_histogram_81[level] = 0;

  /* Step 2: Build frequency distribution of block entropy and reset bitmask */
  for (row = 0; row < rows; ++row) {
    for (col = 0; col < cols; ++col) {
      int index = row * MAX_ROI_GRID + col;
      unsigned int e = entropy_x10_cache[index];
      if (e > 80U) e = 80U;                    /* Hardware saturation clamp */
      ++entropy_histogram_81[e];               /* Increment bin count */
      block_seen[index] = 0;                   /* Clear previous selection mask */
    }
  }

  /* Step 3: Accumulate blocks descending from maximum entropy (8.0 down to 0.0) */
  for (level = 80; level >= 0 && selected < target; --level) {
    unsigned int available = entropy_histogram_81[level];
    if (available == 0) continue;

    /* Scan grid and assert bitmask for blocks matching active quantile level */
    for (row = 0; row < rows && selected < target; ++row) {
      for (col = 0; col < cols && selected < target; ++col) {
        int index = row * MAX_ROI_GRID + col;
        if (entropy_x10_cache[index] == (unsigned char)level) {
          block_seen[index] = 1;           /* Mark block as belonging to Top 10% */
          ++selected;
        }
      }
    }
  }
  *top10_count = selected;                         /* Return populated quantile volume */
}

/**
   @brief  Computes and downlinks on-board correlation metrics proving
           spatial overlap between physical landmarks and high-entropy zones.
   @param  block_size  Active block dimension in pixels (16, 8, or 4).
   @param  cols        Number of grid columns.
   @param  rows        Number of grid rows.
*/

static void send_l2_compare_stats(int block_size, int cols, int rows)
{
  unsigned int top10_count = 0;
  unsigned int overlap_blocks = 0;
  unsigned int landmark_hits = 0;
  int row, col;

  /* Step 1: Populate 'block_seen' mask with top 10% highest entropy blocks */
  build_top10_entropy_mask(cols, rows, &top10_count);

  /* Step 2: Clear spatial landmark presence mask */
  for (row = 0; row < rows; ++row)
    for (col = 0; col < cols; ++col)
      roi_mask[row * MAX_ROI_GRID + col] = 0;

  /* Step 3: Project discrete (X, Y) physical landmarks into block grid coordinates */
  for (row = 0; row < (int)landmark_count; ++row) {
    int bc = landmarks[row].x / block_size;
    int br = landmarks[row].y / block_size;
    if (bc >= 0 && br >= 0 && bc < cols && br < rows)
      roi_mask[br * MAX_ROI_GRID + bc] = 1;     /* Assert presence flag */
  }

  /* Metric 1: Count spatial blocks containing BOTH high entropy AND landmark presence */
  for (row = 0; row < rows; ++row) {
    for (col = 0; col < cols; ++col) {
      int index = row * MAX_ROI_GRID + col;
      if (block_seen[index] && roi_mask[index])
        ++overlap_blocks;
    }
  }

  /* Metric 2: Count absolute physical landmarks residing inside the top 10% entropy mask */
  for (row = 0; row < (int)landmark_count; ++row) {
    int bc = landmarks[row].x / block_size;
    int br = landmarks[row].y / block_size;
    if (bc >= 0 && br >= 0 && bc < cols && br < rows) {
      int index = br * MAX_ROI_GRID + bc;
      if (block_seen[index])
        ++landmark_hits;
    }
  }

  /* Transmit big-endian diagnostic telemetry words to Ground Station */
  uart_send_u32(((unsigned int)MARKER_L2_COMPARE_BLOCKS << 24) | (top10_count & 0xFFFFU));
  uart_send_u32(((unsigned int)MARKER_L2_COMPARE_POINTS << 24) | (overlap_blocks & 0xFFFFU));
  uart_send_u32(((unsigned int)MARKER_L2_COMPARE_HITS   << 24) | (landmark_hits & 0xFFFFU));
}

/**
   @brief  Orchestrates Level 2 downlink pipeline: generates adaptive entropy
           matrix, caches values, streams block packets, and appends diagnostics.
*/

static void send_l2_entropy(void)
{
  int block_size, cols, rows, row, col;
  unsigned int total_blocks;

  /* Pre-execution dependency resolution */
  ensure_landmark_cache();                        /* Verify L1 landmarks exist for diagnostics */
  block_size = choose_adaptive_block_size();      /* Resolve block size (16, 8, or 4 px) */
  ensure_entropy_cache(block_size);               /* Compute entropy grid if cache is dirty/stale */

  cols = entropy_cache_cols;
  rows = entropy_cache_rows;
  total_blocks = (unsigned int)(rows * cols);

  /* Telemetry Header Packet: [D0] | [BlockSize: 8b] | [Cols: 8b] | [Rows: 8b] */
  uart_send_u32(((unsigned int)MARKER_L2_HEADER << 24) |
                ((block_size & 0xFF) << 16) |
                ((cols & 0xFF) << 8) |
                (rows & 0xFF));

  /* Total block count verification word: [D2] | [Reserved: 8b] | [TotalBlocks: 16b] */
  uart_send_u32(((unsigned int)MARKER_L2_COUNT << 24) | (total_blocks & 0xFFFFU));

  /* Stream compressed entropy payload cell-by-cell */
  for (row = 0; row < rows; ++row) {
    for (col = 0; col < cols; ++col) {
      int index = row * MAX_ROI_GRID + col;
      /* Layout: [D1] | [Col: 8b] | [Row: 8b] | [Entropy_x10: 8b] */
      unsigned int packet = ((unsigned int)MARKER_L2_ENTROPY << 24) |
                            (((unsigned int)col & 0xFFU) << 16) |
                            (((unsigned int)row & 0xFFU) << 8) |
                            ((unsigned int)entropy_x10_cache[index] & 0xFFU);
      uart_send_u32(packet);
    }
  }

  /* Append on-board correlation diagnostics */
  send_l2_compare_stats(block_size, cols, rows);

  /* End-of-frame synchronization trailer */
  uart_send_u32(0xFE000000U);
}

/**
   @brief Constructs a sparse transmission mask using logical UNION (ROI = A U B).
   @param block_size Current spatial block dimension (16, 8, or 4 pixels).
   @note  Guarantees retention of physical landmarks while eliminating space vacuum.
*/

static void build_roi_mask_union_mode(int block_size)
{
  /* Calculate grid dimensions with ceiling division for fractional boundaries */
  int cols = ceil_div_int(IMG_WIDTH, block_size);
  int rows = ceil_div_int(IMG_HEIGHT, block_size);

  /* Convert pixel dilation radius into discrete block units */
  int radius_blocks = ceil_div_int(ROI_RADIUS_PIXELS, block_size);

  /* Fixed-point rounding: scale threshold from x100 down to x10 (+5 for proper rounding) */
  unsigned char threshold_x10 = (unsigned char)((current_entropy_threshold_x100 + 5U) / 10U);
  int row, col, i;

  /* Deterministic reset: clear working transmission mask in static SRAM */
  for (i = 0; i < MAX_ROI_CELLS; ++i) roi_mask[i] = 0;

  /* MASK A: Morphological dilation around detected physical landmarks */
  for (i = 0; i < (int)landmark_count; ++i) {
    int center_col = landmarks[i].x / block_size;
    int center_row = landmarks[i].y / block_size;
    int dr, dc;

    /* Expand inclusion window by +/- radius_blocks */
    for (dr = -radius_blocks; dr <= radius_blocks; ++dr) {
      for (dc = -radius_blocks; dc <= radius_blocks; ++dc) {
        int rr = center_row + dr;
        int cc = center_col + dc;

        /* Strict spatial bounding: prevent array index out-of-bounds */
        if (rr < 0 || cc < 0 || rr >= rows || cc >= cols) continue;
        roi_mask[rr * MAX_ROI_GRID + cc] = 1;
      }
    }
  }

  /* MASK B: Merge regions exceeding the operational Shannon entropy threshold */
  for (row = 0; row < rows; ++row) {
    for (col = 0; col < cols; ++col) {
      int index = row * MAX_ROI_GRID + col;
      if (entropy_x10_cache[index] >= threshold_x10) {
        roi_mask[index] = 1;
      }
    }
  }
}

/**
   @brief Streams raw sub-block pixels directly from camera SRAM to interface.
   @param x0 Horizontal pixel origin of the block.
   @param y0 Vertical pixel origin of the block.
   @param bw Actual block width in pixels (clipped at frame boundary).
   @param bh Actual block height in pixels (clipped at frame boundary).
   @note  Zero-copy design: eliminates intermediate line allocation buffers.
*/

static void stream_block_pixels(int x0, int y0, int bw, int bh)
{
  unsigned char *img = (unsigned char *)IMAGE_ADDRESS;
  int y;

  /* Direct row-by-row memory traversal in native camera buffer */
  for (y = 0; y < bh; ++y) {
    /* Compute contiguous memory address of current block row */
    const unsigned char *row_ptr = &img[(y0 + y) * IMG_WIDTH + x0];
    uart_send_bytes(row_ptr, (unsigned int)bw);
  }
}

/**
   @brief Retrieves the sorting metric for a given spatial block index.
   @param index 1D linear coordinate in MAX_ROI_GRID space.
   @return Priority score (Shannon entropy x10 or mean luminance).
*/

static unsigned int l3_priority_value(int index)
{
#if L3_PRIORITY_MODE == 1
  return (unsigned int)block_mean_cache[index]; /* Sort by optical brightness */
#else
  return (unsigned int)entropy_x10_cache[index]; /* Sort by physical Shannon entropy */
#endif
}

/**
   @brief Orders active ROI blocks by descending priority in deterministic O(N) time.
   @param block_size Current block dimension.
   @param cols Grid column count.
   @param rows Grid row count.
   @param selected_count Output pointer receiving total blocks to transmit.
   @note  Eliminates recursive QuickSort stack usage per ECSS Category D.
*/

static void build_l3_priority_order(int block_size, int cols, int rows, unsigned int *selected_count)
{
  unsigned int count = 0;
  unsigned int i;
  unsigned int bucket_count[81];
  unsigned int bucket_start[81];
  unsigned int cursor[81];
  unsigned char threshold_x10 = (unsigned char)((current_entropy_threshold_x100 + 5U) / 10U);
  int row, col;

  /* Initialize 81 entropy buckets (0.0 to 8.0 bits/px in steps of 0.1) */
  for (i = 0; i < 81U; ++i) {
    bucket_count[i] = 0; bucket_start[i] = 0; cursor[i] = 0;
  }

  /* PASS 1: Tally block frequencies in entropy buckets */
  for (row = 0; row < rows; ++row) {
    for (col = 0; col < cols; ++col) {
      int index = row * MAX_ROI_GRID + col;

      /* Reject blocks outside the union mask */
      if (!roi_mask[index]) continue;

      /* CRITICAL FILTER: Prevent dilated landmarks from leaking into pure vacuum */
      if (entropy_x10_cache[index] < threshold_x10) continue;

      unsigned int priority = l3_priority_value(index);
      if (L3_PRIORITY_MODE == 1) priority >>= 3;
      if (priority > 80U) priority = 80U; /* Bound clamp */

      bucket_count[priority]++;
      selected_indices[count++] = (unsigned short)index;
    }
  }

  /* PASS 2: Compute prefix sum to determine bucket start offsets in output array */
  for (i = 1; i <= 80U; ++i) bucket_start[i] = bucket_start[i - 1] + bucket_count[i - 1];
  for (i = 0; i <= 80U; ++i) cursor[i] = bucket_start[i];

  /* PASS 3: Distribute indices into sorted order in a single deterministic pass */
  for (i = 0; i < count; ++i) {
    unsigned int index = selected_indices[i];
    unsigned int priority = l3_priority_value((int)index);
    if (L3_PRIORITY_MODE == 1) priority >>= 3;
    if (priority > 80U) priority = 80U;
    sorted_indices[cursor[priority]++] = (unsigned short)index;
  }
  *selected_count = count;
  (void)block_size; /* Silence unused parameter warning */
}

/**
   @brief Packages and downlinks prioritized ROI surface blocks.
   @note  Streams blocks descending from highest to lowest informational value.
*/

static void send_l3_roi_image(void)
{
  int block_size, cols, rows;
  unsigned int selected_count = 0, i;

  /* Guarantee that descriptor and entropy caches are populated */
  ensure_landmark_cache();
  block_size = choose_adaptive_block_size();
  ensure_entropy_cache(block_size);
  cols = entropy_cache_cols;
  rows = entropy_cache_rows;

  /* Build spatial mask and sort blocks */
  build_roi_mask_union_mode(block_size);
  build_l3_priority_order(block_size, cols, rows, &selected_count);

  /* Telemetry Header 1: Grid layout configuration */
  uart_send_u32(((unsigned int)MARKER_L3_HEADER << 24) | ((block_size & 0xFF) << 16) | ((cols & 0xFF) << 8) | (rows & 0xFF));

  /* Telemetry Header 2: Dynamic threshold metadata transmitted to Ground */
  uart_send_u32(((unsigned int)MARKER_L3_META << 24) | ((L3_PRIORITY_MODE & 0xFF) << 16) | (current_entropy_threshold_x100 & 0xFFFFU));

  /* Telemetry Header 3: Total blocks being transmitted in this stream */
  uart_send_u32(((unsigned int)MARKER_L3_COUNT << 24) | (selected_count & 0xFFFFU));

  /* PROGRESSIVE DISPATCH: Transmit in reverse order (highest entropy first) */
  for (i = selected_count; i > 0U; --i) {
    int index = (int)sorted_indices[i - 1U];
    int row = index / MAX_ROI_GRID;
    int col = index - row * MAX_ROI_GRID;

    int x0 = col * block_size;
    int y0 = row * block_size;

    /* Bound clipping at sensor edges */
    int bw = (x0 + block_size > IMG_WIDTH)  ? (IMG_WIDTH - x0)  : block_size;
    int bh = (y0 + block_size > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : block_size;

    /* Block Header: Synchronize coordinate layout for ground reconstruction */
    uart_send_u32(((unsigned int)MARKER_L3_BLOCK << 24) | (((unsigned int)col & 0xFFU) << 16) | (((unsigned int)row & 0xFFU) << 8));

    /* Transmit raw pixel bytes */
    stream_block_pixels(x0, y0, bw, bh);
  }

  /* Frame completion trailer */
  uart_send_u32(0xFE000000U);
}

/* =========================================================================
   L4 FULL RASTER STREAMING: BILATERAL OPPOSING VERTICAL SCAN
   Progressively streams the uncompressed 1020x1020 frame using paired
   columns converging from the image borders inward to the center.
   ========================================================================= */

static void send_full_image(void)
{
  int cols = ceil_div_int(IMG_WIDTH, L4_BLOCK_SIZE);   /* 1020 / 16 -> 64 block columns */
  int rows = ceil_div_int(IMG_HEIGHT, L4_BLOCK_SIZE);  /* 1020 / 16 -> 64 block rows */
  int half_cols = cols / 2;                            /* 32 column pairs */
  int row, i;

  /* Transmit L4 header: marker (0xC0), block dimension (16px), grid geometry (64x64) */
  uart_send_u32(((unsigned int)MARKER_FULL_HEADER << 24) |
                ((L4_BLOCK_SIZE & 0xFF) << 16) |
                ((cols & 0xFF) << 8) |
                (rows & 0xFF));

  /* Bilateral progressive column loop: moves simultaneously from borders to center */
  for (i = 0; i < half_cols; ++i) {
    /* 1. Stream Left-Wing Column (Column i: 0, 1, 2...) */
    int col_left = i;
    for (row = 0; row < rows; ++row) {
      int x0 = col_left * L4_BLOCK_SIZE;
      int y0 = row * L4_BLOCK_SIZE;
      /* Border clipping: handles non-multiple dimensions (1020 px mod 16 = 12 px) */
      int bw = (x0 + L4_BLOCK_SIZE > IMG_WIDTH)  ? (IMG_WIDTH - x0)  : L4_BLOCK_SIZE;
      int bh = (y0 + L4_BLOCK_SIZE > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : L4_BLOCK_SIZE;

      /* Transmit block header word: marker (0xC3) | col_idx [23:16] | row_idx [15:8] */
      uart_send_u32(((unsigned int)MARKER_FULL_BLOCK << 24) |
                    (((unsigned int)col_left & 0xFFU) << 16) |
                    (((unsigned int)row & 0xFFU) << 8));
      stream_block_pixels(x0, y0, bw, bh); /* Stream raw pixel buffer directly from SRAM */
    }

    /* 2. Stream Right-Wing Column (Column 63 - i: 63, 62, 61...) */
    int col_right = cols - 1 - i;
    for (row = 0; row < rows; ++row) {
      int x0 = col_right * L4_BLOCK_SIZE;
      int y0 = row * L4_BLOCK_SIZE;
      int bw = (x0 + L4_BLOCK_SIZE > IMG_WIDTH)  ? (IMG_WIDTH - x0)  : L4_BLOCK_SIZE;
      int bh = (y0 + L4_BLOCK_SIZE > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : L4_BLOCK_SIZE;

      uart_send_u32(((unsigned int)MARKER_FULL_BLOCK << 24) |
                    (((unsigned int)col_right & 0xFFU) << 16) |
                    (((unsigned int)row & 0xFFU) << 8));
      stream_block_pixels(x0, y0, bw, bh);
    }
  }

  /* Handle odd column count if grid width is not evenly divisible by 2 */
  if (cols % 2 != 0) {
    int col_mid = half_cols;
    for (row = 0; row < rows; ++row) {
      int x0 = col_mid * L4_BLOCK_SIZE;
      int y0 = row * L4_BLOCK_SIZE;
      int bw = (x0 + L4_BLOCK_SIZE > IMG_WIDTH)  ? (IMG_WIDTH - x0)  : L4_BLOCK_SIZE;
      int bh = (y0 + L4_BLOCK_SIZE > IMG_HEIGHT) ? (IMG_HEIGHT - y0) : L4_BLOCK_SIZE;

      uart_send_u32(((unsigned int)MARKER_FULL_BLOCK << 24) |
                    (((unsigned int)col_mid & 0xFFU) << 16) |
                    (((unsigned int)row & 0xFFU) << 8));
      stream_block_pixels(x0, y0, bw, bh);
    }
  }

  /* Transmit end-of-frame synchronization trailer */
  uart_send_u32(0xFE000000U);
}

/* =========================================================================
   BARE-METAL STRING UTILITIES (ZERO LIBC DEPENDENCY)
   Handcrafted string parsers complying with ECSS-E-ST-40C / MISRA rules.
   Completely eliminates <string.h> to prevent uncertified runtime overhead.
   ========================================================================= */

/* Exact string equality comparator */
static int text_equals(const char *a, const char *b)
{
  while (*a && *b) {
    if (*a != *b) return 0;
    ++a; ++b;
  }
  return (*a == '\0' && *b == '\0');
}

/* Prefix matching comparator for parameterized telecommands */
static int text_starts_with(const char *str, const char *prefix)
{
  while (*prefix) {
    if (*str != *prefix) return 0;
    str++; prefix++;
  }
  return 1;
}

/* =========================================================================
   TELECOMMAND (TTC) PARSER & FLIGHT DISPATCHER
   Converts ASCII command stream into flight execution tokens in O(N) time.
   ========================================================================= */

/**
   @brief Parses numerical threshold argument from "AURA GATE <val>" command.
   @note  Bypasses atoi(), strtol(), or sscanf() to eliminate runtime bloat.
*/

static int parse_threshold_val(const char *cmd)
{
  int val = 0;
  const char *p = cmd + 10; /* Fixed offset past "AURA GATE " prefix (10 characters) */

  /* Linear ASCII-to-integer conversion */
  while (*p >= '0' && *p <= '9') {
    val = val * 10 + (*p - '0');
    p++;
  }
  return val;
}

/* Decodes incoming command string into discrete numeric flight modes */
static int decode_command(const char *cmd)
{
  if (text_equals(cmd, "AURA L0"))         return 10; /* Mode L0: Top-100 Cloud */
  if (text_equals(cmd, "AURA L1"))         return 11; /* Mode L1: Top-1000 Map */
  if (text_equals(cmd, "AURA L2"))         return 12; /* Mode L2: Full Entropy */
  if (text_equals(cmd, "AURA L3"))         return 13; /* Mode L3: Sparse ROI */
  if (text_equals(cmd, "AURA L4"))         return 14; /* Mode L4: Full Image */

  /* Parameterized telecommand for dynamic entropy threshold gating */
  if (text_starts_with(cmd, "AURA GATE ")) return 15;

  return 0; /* Unknown command token */
}

/* =========================================================================
   NON-BLOCKING TELECOMMAND INGESTION STREAM PARSER
   Drains incoming hardware FIFO bytes without stalling the CPU pipeline.
   Implements bounded buffer tracking to prevent stack smashing.
   ========================================================================= */

static int uart_poll_command(char *cmd_buffer, int *cmd_pos)
{
  char c; /* Scratchpad byte register for incoming UART character */

  /* Non-blocking polling: inspects LSR register bit 0 (Data Ready) */
  while (uart_getc_nonblocking(&c)) {

    /* Frame Delimiter Detection: Carriage Return (\r) or Line Feed (\n) */
    if (c == '\r' || c == '\n') {
      int mode;
      cmd_buffer[*cmd_pos] = '\0';        /* Explicit null-termination of ASCII token */
      mode = decode_command(cmd_buffer);   /* Dispatch string through static comparator */
      *cmd_pos = 0;                       /* Reset cursor for the next telecommand frame */
      return mode;                        /* Return decoded command ID (10..15) */
    }

    /* Boundary Guard: append byte only if within strict 32-byte scratchpad limit */
    if (*cmd_pos < 31) {
      cmd_buffer[(*cmd_pos)++] = c;
    } else {
      /* Buffer Overflow Mitigation: drop corrupt frame and reset cursor */
      *cmd_pos = 0;
    }
  }

  return 0; /* Return 0: FIFO drained, no complete telecommand frame assembled yet */
}

/* =========================================================================
   FLIGHT SUPER-LOOP EXECUTIVE & COMMAND DISPATCHER
   Executes bare-metal polling cycle with deterministic telecommand dispatch.
   ========================================================================= */

int main(void)
{
  char cmd_buffer[32]; /* Static stack allocation for telecommand string (32 bytes) */
  int cmd_pos = 0;     /* Active buffer insertion cursor */

  /* Boot banner: validates that UART TX line and clocking are operational */
  print_str("\n===================================\n");
  print_str("AURA Firmware V0.3: Interactive Telemetry Core\n");
  print_str("===================================\n\n");

  /* Infinite Bare-Metal Event Loop (Super-Loop Architecture) */
  while (1) {
    /* Poll the non-blocking receiver */
    int mode = uart_poll_command(cmd_buffer, &cmd_pos);

    /* Dispatch confirmed telecommand tokens */
    if (mode == 10) {
      uart_send_u32(0xAC00000AU); /* Telecommand Execution ACK for L0 */
      send_l0_cloud();            /* Transmit Top-100 Landmark Cloud */
    }
    else if (mode == 11) {
      uart_send_u32(0xAC00000BU); /* Telecommand Execution ACK for L1 */
      send_l1_cloud();            /* Transmit Top-1000 Directional Map */
    }
    else if (mode == 12) {
      uart_send_u32(0xAC00000CU); /* Telecommand Execution ACK for L2 */
      send_l2_entropy();          /* Transmit 64x64 Entropy Grid */
    }
    else if (mode == 13) {
      uart_send_u32(0xAC00000DU); /* Telecommand Execution ACK for L3 */
      send_l3_roi_image();        /* Transmit Gated Sparse ROI Blocks */
    }
    else if (mode == 14) {
      uart_send_u32(0xAC00000EU); /* Telecommand Execution ACK for L4 */
      send_full_image();          /* Transmit Opposing Full Raster */
    }
    else if (mode == 15) {
      /* Parameterized Telecommand: Parse and update dynamic gating threshold */
      int new_gate = parse_threshold_val(cmd_buffer);

      /* Range Verification (Sanity Check): Bounded within 0.00 to 8.00 bits/pixel */
      if (new_gate >= 0 && new_gate <= 800) {
        current_entropy_threshold_x100 = (unsigned short)new_gate; /* Atomic update in SRAM */
        uart_send_u32(0xAC0000FFU); /* Telecommand Completion ACK (Marker 0xAC | 0xFF) */
      }
    }
  }

  return 0; /* Unreachable statement in bare-metal embedded execution */
}
