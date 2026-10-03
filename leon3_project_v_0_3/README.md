# AURA: Autonomous Unsupervised Feature-Tracking for Real-Time Deep-Space Navigation

![AURA Ground Station](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/main/leon3_project_v_0_3/AURA_GrndSeg_Screenshot.png)

## Project Overview
**AURA** is an experimental on-board flight software pipeline designed for real-time, resource-constrained edge computing during deep-space small-body rendezvous operations. The primary objective of the architecture is to process high-resolution optical matrices locally, compute statistical data density patterns, and isolate high-entropy regions of interest (ROI) to facilitate autonomous proximity operations without saturated communication downlinks.

This implementation achieves **Technology Readiness Level 4 (TRL 4)** validation, operating within a simulated aerospace environment representative of the European Space Agency's (ESA) **Hera** deep-space mission profile.

---

## Technical Architecture & Core Constraints
To comply with strict aerospace software engineering standards (such as ECSS Category D) and target space-grade hardware specifications (GR712RC Dual-Core SPARC V8), the software pipeline is bound by the following low-level operational limits:
*   **Hardware Architecture:** 32-bit SPARC V8 (LEON3) processor core executing at a flight-representative **250 MIPS**.
*   **Memory Restrictions:** Strict **16 MB RAM** static partition sandbox. Dynamic memory allocation (`malloc`, `free`) is entirely omitted to enforce execution determinism.
*   **Sensor Interfacing:** Interfaced with a simulated **Hera AFC** navigation camera utilizing a monochrome sensor configuration: **1020x1020 pixels, strict 8-bit Grayscale** (1 byte per pixel, total raw frame size: 1,040,400 bytes).
*   **Zero Floating-Point Unit (FPU) Overhead:** Fixed-point integer mathematical models completely replace standard floating-point functions (`float`, `double`, `log2f`). Logarithmic probabilities are resolved using ultra-fast bitwise arithmetic via a pre-calculated `log2_q8` lookup array in Q8 fixed-point format.
*   **On-Board Entropy Cache:** The Shannon entropy calculation results are fully cached (`entropy_x10_cache[]`, `block_mean_cache[]`) and shared between L2 and L3 channels, completely eliminating duplicate CPU clock-cycle overhead.

## Multi-Level Telemetry & Bidirectional Command Array
AURA operates as an interactive, closed-loop **Telemetry & Telecommand (TTC)** system. Data serialization circumvents human-readable strings inside the real-time processing loop. Instead, the firmware packs localized statistical telemetry directly into high-density **32-bit unsigned integer registers** (`uint32`), allocating parameters down to the exact bit level:

*   **L0 — Top-100 Landmark Cloud:** Downlinks packed local-contrast maxima coordinates (X, Y) compressed into 20 bits per point, transmitted via 3 sequential bytes. Traffic reduction: **99.97%**.
*   **L1 — Top-1000 Landmark Map with Edge Vectors:** Packs 10-bit X, 10-bit Y, a 4-bit pixel intensity, and a **4-bit localized anisotropic gradient direction angle**. This transforms standard coordinate points into directional surface descriptors (ORB-like features). Traffic reduction: **99.92%**.
*   **L2 — Full Entropy Map with Cross-Validation:** Streams a continuous matrix of Shannon entropy metrics (adaptive 16px/8px/4px blocks) with integrated on-board diagnostic cross-matching (MARKER_L2_COMPARE) mapping structural landmarks against high-entropy zones. Traffic reduction: **98.42%**.
*   **L3 — Sparse ROI Image (Logical UNION Mode):** Transmits highly informative surface segments generated via the logical union (\(\text{ROI} = A \cup B\)) of landmark-driven and entropy-driven masks. Blocks are prioritized on-board and streamed **highest-entropy first**, allowing Ground Segments to receive scientific payload anchors instantly. Bandwidth savings: **~79.00%**.
*   **L4 — Full Image (Opposing Vertical Scan):** Raw uncompressed frame verification layer. To support rapid optical silhouette stabilization, blocks are progressively streamed via a **bilinear counter-directional pattern** executing from the left and right frame borders simultaneously towards the center.

### On-Board Software Gating Telecommand (Uplink Loop)
Ground operators can dynamically tune the balance between downlink bandwidth volume and scientific data density by transmitting an **Uplink Telecommand (`AURA GATE <val>`)** in real-time. The virtual spacecraft captures the instruction via the UART interface, unblocks the execution thread, and rewrites the on-board filtering register on the fly.

### Bit Allocation for L2 Entropy Serialization Word:

| Bit Range | Size (Bits) | Description |
|---|---|---|
| **[31:24]** | 8 | Synchronization / Data frame identifier marker (`0xD1`). |
| **[23:16]** | 8 | Column Index (`col_idx`), representing block X-coordinate layout. |
| **[15:8]**  | 8 | Row Index (`row_idx`), representing block Y-coordinate layout. |
| **[7:0]**   | 8 | Scaled Shannon Entropy value (Entropy × 10). |

*   **Trap & Exception Mitigation:** The packed 32-bit words are transmitted over the physical interface byte-by-byte via sequential register flushing. This prevents unaligned word memory access anomalies, completely eliminating the risk of critical processor exceptions (**SPARC Trap 0x07 / Data Access Alignment Trap**).

---

## Repository Structure
The production-ready V0.3 workspace contains the following core files:
*   `Hello_AURA.c` — Standalone core flight software executing the fixed-point telemetry processing loops and TTC parsing.
*   `experiment_test.elf` — The compiled space-grade executable target binary.
*   `image.bin` — Raw 8-bit monochrome asteroid camera sensor matrix direct-mapped at `0x40600000`.
*   `leon3.repl` / `script.resc` — Renode hardware platform infrastructure and automation deployment configurations.
*   `AuraGroundUI.py` — Multi-channel Ground Segment console running an asynchronous receiver thread, dynamic heatmap scaling, and real-time dashboard plotting.
*   `logs/aura_telemetry_log.csv` — Automated performance logger capturing runtime mission audits.

## Deployment & Execution Procedure

### Step 1: Toolchain Cross-Compilation
To cross-compile the standalone flight software application from source using the official Aeroflex Gaisler BCC2 cross-compiler toolchain, execute the following command within a Windows PowerShell terminal:

```powershell
& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g Hello_AURA.c -o experiment_test.elf "-Wl,-Ttext=0x40000000" "-Wl,-z,muldefs" -lgcc
```

### Step 2: Launch the Spacecraft Emulation Framework
In a separate terminal window, initiate the Software-in-the-Loop (SIL) validation inside the Renode environment:
```powershell
renode script.resc
```

### Step 3: Initialize the Ground Segment Console
Launch the English-standardized real-time telemetry decoder to listen for loopback interface streaming:
```powershell
python AuraGroundUI.py
```

### Step 4: Interactive Operation & Analytics Export
1. Click the **"Connect"** button on the Ground Console to establish the synchronous radio link.
2. Select any requested telemetry level (`L0` to `L4`) to command the onboard computer.
3. Adjust the **Shannon Entropy Compression Threshold** slider on the Ground UI and click **"Transmit Gate Command"** to rewrite the spacecraft's data filtering rules in real time.
4. **Data Export:** Right-click on any active plot component (Landmark View, Entropy Map, or Science Frame) to trigger a native **1020x1020 high-fidelity export** saved via Pillow directly as a raw array.
5. **Mission Logs:** Open the `logs/aura_telemetry_log.csv` directory to analyze byte counts, compression ratios, and transmission timestamps.
