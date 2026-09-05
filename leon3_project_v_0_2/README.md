# AURA: Autonomous Unsupervised Feature-Tracking for Real-Time Deep-Space Navigation

## Project Overview
**AURA** is an experimental on-board flight software pipeline designed for real-time, resource-constrained edge computing during deep-space small-body rendezvous operations. The primary objective of the architecture is to process high-resolution optical matrices locally, compute statistical data density patterns, and isolate high-entropy regions of interest (ROI) to facilitate autonomous proximity operations.

This implementation is architected to achieve **Technology Readiness Level 4 (TRL 4)** validation, operating within a simulated aerospace environment representative of the European Space Agency's (ESA) **Hera** deep-space mission profile.

---

## Technical Architecture & Core Constraints
To comply with strict aerospace software engineering standards (such as ECSS Category D) and target space-grade hardware specifications (GR712RC Dual-Core SPARC V8), the software pipeline is bound by the following low-level operational limits:
*   **Hardware Architecture:** 32-bit SPARC V8 (LEON3) processor core executing at a flight-representative **250 MIPS**.
*   **Memory Restrictions:** Strict **16 MB RAM** static partition sandbox. Dynamic memory allocation (`malloc`, `free`) is entirely omitted to enforce execution determinism.
*   **Sensor Interfacing:** Interfaced with a simulated **Hera AFC** navigation camera utilizing a monochrome *FaintStar2* sensor configuration: **1020x1020 pixels, strict 8-bit Grayscale** (1 byte per pixel, total raw frame size: 1,040,400 bytes).
*   **Zero Floating-Point Unit (FPU) Overhead:** Fixed-point integer mathematical models completely replace standard floating-point functions (`float`, `double`, `log2f`). Logarithmic probabilities are resolved using ultra-fast bitwise arithmetic.
*   **Histogram Footprint Optimization:** Features a dedicated tracking stack that enables precise, point-by-point clearing of modified memory indexes. This bounds clearing operations to $O(N)$ efficiency (where $N$ is the count of active grayscale channels per block), maintaining internal CPU cache efficiency.

---

## Telemetry Serialization & Downlink Footprint Compression
Data serialization circumvents human-readable ASCII or string parsing inside the real-time processing loop. Instead, the firmware packs localized statistical telemetry directly into high-density **32-bit unsigned integer registers** (`uint32`), allocating data parameters down to the exact bit level:

| Bit Range | Size (Bits) | Description |
|---|---|---|
| **[31:24]** | 8 | Synchronization / Data frame identifier marker (`0xA5`). |
| **[23:17]** | 7 | Column Index (`col_idx`), representing block X-coordinate coordinate layout (up to 2048 px). |
| **[16:10]** | 7 | Row Index (`row_idx`), representing block Y-coordinate coordinate layout (up to 1536 px). |
| **[9:0]** | 10 | Scaled Shannon Entropy value ($Entropy \times 100$). Supports an active integer range from 0 to 1023. |

*   **Trap & Exception Mitigation:** The packed 32-bit words are transmitted over the physical interface byte-by-byte via sequential register flushing. This prevents unaligned word memory access anomalies, completely eliminating the risk of critical processor exceptions (**SPARC Trap 0x07 / Data Access Alignment Trap**).
*   **Performance Metrics:** Telemetry validation reports a **85.2% absolute lossless reduction** in downlink data volume compared to baseline text streaming, achieving a **6.75x bandwidth optimization factor** over the telemetry link.

---

## Repository Structure
The production-ready workspace contains the following core files:
*   `Hello_AURA.c` — The standalone core flight software application executing the fixed-point block entropy pipeline.
*   `experiment_test.elf` — The final 18 MB compiled space-grade executable binary containing embedded image matrices.
*   `hera_types.h` — Injection type-definition header providing strict compliance mapping for standard integer specifications.
*   `hera_interface.h` — Official ESA OSIP API interface prototype declarations for the Hera mission payload suite.
*   `hera_client_stub.c` — The original flight simulation stub managing camera synchronization frames.
*   `images_data.7z` — The compressed archive containing the main flight data bank header (`images_data.h`). **Must be extracted before compilation.**
*   `image.bin` — The raw 8-bit monochrome binary matrix extracted for hardware memory direct mapping (`0x40600000`).
*   `leon3.repl` — The Renode hardware platform description file enforcing the exact 16 MB memory map layout.
*   `script.resc` — The automation deployment script establishing the socket bindings and CPU clock performance.
*   `telemetry_live_visualizer.py` — The Ground Segment analytics visualizer decoding binary masks into a real-time heatmap.
*   `start.S` / `stub_utils.h` — Low-level assembly initialization sequences and printing primitives for the SPARC architecture.

---

## Deployment & Execution Procedure

### Step 0: Extract the Flight Data Bank
Before initiating the compilation pipeline, you must extract the compressed flight data bank header containing the integrated 404-frame optical matrices:
1. Locate the `images_data.7z` archive in the project root directory.
2. Extract the file using 7-Zip or any compatible decompression utility.
3. Ensure that the resulting file **`images_data.h`** is placed directly in the project root directory alongside `Hello_AURA.c`.

### Step 1: Toolchain Cross-Compilation
To recompile the flight software from source using the official Aeroflex Gaisler BCC2 cross-compiler toolchain, execute the following multi-stage compilation pipeline within a Windows PowerShell terminal:

```powershell
# 1. Compile the flight application layer into an object file
& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g -include hera_types.h -c Hello_AURA.c -o Hello_AURA.o

# 2. Compile the mission simulation stub layer into an object file
& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g -include hera_types.h -c hera_client_stub.c -o hera_client_stub.o

# 3. Link objects into the final aerospace ELF image aligned at target memory space
& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" Hello_AURA.o hera_client_stub.o -o experiment_test.elf "-Wl,-Ttext=0x40000000" "-Wl,-z,muldefs" -lgcc
```

### Step 2: Launch the Spacecraft Emulation Framework
In the primary command terminal, initiate the software-in-the-loop validation inside the Renode environment:
```powershell
renode script.resc
```

### Step 3: Initialize the Ground Segment Visualizer
Open a separate terminal window and launch the telemetry live decoder to listen for incoming binary flows from the spacecraft:
```powershell
python telemetry_live_visualizer.py
```

Once execution commences, the onboard application will process the locked 1020x1020 image grid. The telemetry stream will route dynamically over the host interface loopback (`127.0.0.1:12345`), rendering a live, interactive mathematical heatmap of the asteroid terrain in the Ground Segment visualizer window.
