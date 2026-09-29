# AURA: Autonomous Unsupervised Feature-Tracking for Real-Time Deep-Space Navigation

![AURA Ground Station](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/develop/leon3_project_v_0_3/AURA_GrndSeg_Screenshot.png)

## Project Overview
**AURA** is an experimental on-board flight software pipeline designed for real-time, resource-constrained edge computing during deep-space small-body rendezvous operations. The primary objective of the architecture is to process high-resolution optical matrices locally, compute statistical data density patterns, and isolate high-entropy regions of interest (ROI) to facilitate autonomous proximity operations without saturated communication downlinks.

This implementation is architected to achieve **Technology Readiness Level 4 (TRL 4)** validation, operating within a simulated aerospace environment representative of the European Space Agency's (ESA) **Hera** deep-space mission profile.

---

## Technical Architecture & Core Constraints
To comply with strict aerospace software engineering standards (such as ECSS Category D) and target space-grade hardware specifications (GR712RC Dual-Core SPARC V8), the software pipeline is bound by the following low-level operational limits:
*   **Hardware Architecture:** 32-bit SPARC V8 (LEON3) processor core executing at a flight-representative **250 MIPS**.
*   **Memory Restrictions:** Strict **16 MB RAM** static partition sandbox. Dynamic memory allocation (`malloc`, `free`) is entirely omitted to enforce execution determinism.
*   **Sensor Interfacing:** Interfaced with a simulated **Hera AFC** navigation camera utilizing a monochrome sensor configuration: **1020x1020 pixels, strict 8-bit Grayscale** (1 byte per pixel, total raw frame size: 1,040,400 bytes).
*   **Zero Floating-Point Unit (FPU) Overhead:** Fixed-point integer mathematical models completely replace standard floating-point functions (`float`, `double`, `log2f`). Logarithmic probabilities are resolved using ultra-fast bitwise arithmetic via a pre-calculated `log2_q8` lookup array in Q8 fixed-point format.
*   **Histogram Footprint Optimization:** Features a dedicated tracking stack that enables precise, point-by-point clearing of modified memory indexes. This bounds clearing operations to \(O(N)\) efficiency (where \(N\) is the count of active grayscale channels per block), maintaining internal CPU cache efficiency.

---

## Multi-Level Telemetry & Link Compression Footprint
Data serialization circumvents human-readable ASCII or string parsing inside the real-time processing loop. Instead, the firmware packs localized statistical telemetry directly into high-density **32-bit unsigned integer registers** (`uint32`), allocating data parameters down to the exact bit level based on the selected Telemetry Level:

*   **L0 — Top-100 Landmark Cloud + Score:** Downlinks local-contrast maxima coordinates (X, Y) with an attached 4-bit intensity score. Reduces traffic by **99.96%**.
*   **L1 — Top-1000 Landmark Map (X, Y only):** Streams high-density packed coordinates (20 bits per point, transmitted byte-by-byte via 3 sequential bytes) to bypass telemetry score overhead. Reduces traffic by **99.94%**.
*   **L2 — Full Adaptive Entropy Map:** Streams a continuous 64x64 matrix of Shannon entropy metrics (16px blocks) covering the entire sensor field. Reduces traffic by **98.42%**.
*   **L3 — Sparse ROI Image Blocks:** Transmits only high-entropy surface segments (where \(Entropy \geq 2.50\) bits/pixel), entirely nulling out empty deep-space arrays. Achieves **~79.00% bandwidth savings** while retaining 100% of scientific landmarks.
*   **L4 — Full Image / Block Stream:** Raw uncompressed frame verification layer, progressively streaming 16x16 pixel blocks in explicit raster order.

### Bit Allocation for L2 Entropy Serialization Word:

| Bit Range | Size (Bits) | Description |
|---|---|---|
| **[31:24]** | 8 | Synchronization / Data frame identifier marker (`0xA5`). |
| **[23:16]** | 8 | Column Index (`col_idx`), representing block X-coordinate layout. |
| **[15:8]**  | 8 | Row Index (`row_idx`), representing block Y-coordinate layout. |
| **[7:0]**   | 8 | Scaled Shannon Entropy value (\(Entropy \times 10\)). |

*   **Trap & Exception Mitigation:** The packed 32-bit words are transmitted over the physical interface byte-by-byte via sequential register flushing. This prevents unaligned word memory access anomalies, completely eliminating the risk of critical processor exceptions (**SPARC Trap 0x07 / Data Access Alignment Trap**).

---

## Repository Structure
The production-ready V0.3 workspace contains the following core files:
*   `Hello_AURA.c` — Independent, standalone core flight software application executing the bare-metal fixed-point telemetry pipeline.
*   `experiment_test.elf` — The final compiled space-grade executable binary containing embedded image matrices.
*   `image.bin` — The raw 8-bit monochrome binary matrix extracted for hardware memory direct mapping (`0x40600000`).
*   `leon3.repl` / `script.resc` — Renode platform description and automation deployment scripts establishing loopback socket bindings.
*   `AuraGroundUI.py` — The Ground Segment interactive multi-channel visualizer decoding binary flows into a synchronized real-time multi-display dashboard.
*   `logs/aura_telemetry_log.csv` — Automated mission logger tracking byte volumes, bandwidth savings, and compression factors dynamically.

---

## Deployment & Execution Procedure

### Step 1: Toolchain Cross-Compilation
To compile the independent flight software from source using the official Aeroflex Gaisler BCC2 cross-compiler toolchain, execute the following command within a Windows PowerShell terminal. This bypasses default startup routines and links the custom assembly bootloader:

```powershell
& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g Hello_AURA.c -o experiment_test.elf "-Wl,-Ttext=0x40000000" "-Wl,-z,muldefs" -lgcc
```

### Step 2: Launch the Spacecraft Emulation Framework
In the primary command terminal, initiate the software-in-the-loop validation inside the Renode environment:
```powershell
renode script.resc
```

### Step 3: Initialize the Ground Segment Console
Open a separate terminal window and launch the English-standardized real-time telemetry decoder:
```powershell
python AuraGroundUI.py
```

### Step 4: Interactive Operation & Analytics Export
1. Click the **"Connect"** button on the top-right of the Ground Console to bond the telemetry channel.
2. Select any requested telemetry level (`L0` to `L4`) to command the virtual LEON3 core.
3. Observe **zero-latency real-time block rendering** as data progressive packages arrive over the loopback interface.
4. **Data Export:** Right-click on any of the active plot displays (Landmark Cloud, Entropy Map, or Science Frame) to save the current dataset into a clean, native **1020x1020 PNG image** without UI borders.
5. **Mission Logs:** Explore the `logs/aura_telemetry_log.csv` file for automated, step-by-step performance audits.
