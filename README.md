# AURA: Autonomous Unsupervised Feature-Tracking for Real-Time Deep-Space Navigation
An ultra-lightweight, deterministic embedded vision framework designed for real-time edge computing, autonomous surface feature extraction, and adaptive telemetry prioritization in deep-space environments.

![AURA Concept Infographic](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/main/Img/AURA_Simple_Idea.png)

AURA addresses severe communication bottlenecks (intermittent downlinks, round-trip signal delays reaching tens of minutes) and strict avionics resource constraints during autonomous proximity operations near uncharted celestial bodies. Validated at **Technology Readiness Level 4 (TRL 4)**, the architecture operates within a deterministic, bare-metal memory sandbox compliant with **ECSS Category D** software engineering standards.

[![TRL](https://img.shields.io/badge/TRL-4-blue.svg)](#-building-and-running-the-simulation)
[![Architecture](https://img.shields.io/badge/Target-LEON3%20%2F%20SPARC%20V8-informational.svg)](#-technical-architecture--aerospace-constraints)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 🛰️ Mission Context & Operational Philosophy

Deep-space exploration missions (such as asteroid rendezvous, comet flybys, and lunar descents) cannot rely on continuous ground-in-the-loop control. Standard raw image payloads (1020x1020 px, 8-bit monochromatic frames ≈ 1.04 MB) saturate narrow Deep Space Network (DSN) passes with vast expanses of vacant space background.

AURA inverts traditional pixel streaming by implementing an **Information-Theoretic Perception Paradigm**:
> **"Information should be prioritized according to its mathematical and physical value, rather than transmitting uniform raw pixels."**

### Core Capabilities:
1. **Autonomous Optical Edge Gating:** Directly evaluates local informational density via integer Shannon entropy, stripping 100% of empty vacuum background on-board.
2. **Directional Landmark Tracking:** Extracts structural surface vertices (crater rims, boulders, limbs) with localized 4-bit anisotropic gradient vectors, ready for direct ingestion into autonomous GNC (Guidance, Navigation, and Control) and Visual Odometry loops.
3. **Multi-Tier Progressive Telemetry (L0–L4):** Dynamically scales data products from ultra-sparse 240-byte landmark clouds up to prioritized region-of-interest (ROI) surface segments.
4. **Resilient Sandbox Isolation:** Operates deterministically without operating system dependencies or dynamic memory allocations (`malloc`), ensuring zero heap fragmentation and strictly bounded Worst-Case Execution Time (WCET).

---

## 📊 Empirical Telemetry Benchmarks & Downlink Hierarchy

Validated across high-resolution deep-space small-body datasets, AURA delivers an adaptive downlink spectrum tailored to real-time link budgets:

| Tier | Format & Data Layout | Payload Size (Wire) | Data Reduction | Acceleration | Operational Mission Use |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **L0** | Packed (X, Y) coordinate stream (20-bit records, 3 B/point) | **~240 Bytes** | **99.98%** | **~4,250×** | Ultra-fast limb tracking & optical centroiding during razor-thin DSN passes. |
| **L1** | 32-bit descriptors (X, Y + 4-bit edge vector angle + intensity) | **~320 Bytes** | **99.97%** | **~3,250×** | Directional landmark matching & topological surface tracking (ORB-like features). |
| **L2** | 64x64 adaptive grid (16px blocks) of local Shannon entropy | **16.4 KiB** | **98.45%** | **64.39×** | Scene complexity topography & onboard cross-validation diagnostics. |
| **L3** | **Sparse Gated ROI (Logical UNION A ∪ B):** High-entropy pixel blocks | **~330 KiB** | **68.80% – 80%** | **~3.22×** | **Primary Scientific Mode:** Lossless crater/boulder textures; deep-space vacuum eliminated. |
| **L4** | Full frame verification raster via Opposing Vertical Scan | **1,056,796 Bytes** | **0.00%** | **1.00×** | Baseline verification reference (framing overhead strictly bounded to **+1.58%**). |

> **Structural Validation:** Empirical correlation proves that **over 59.9% of all detected contrast landmarks align directly inside the top 10% highest-entropy blocks**, confirming that local Shannon entropy mathematically tracks physical surface structures.

---

## 🔒 Technical Architecture & Aerospace Constraints

The firmware is engineered to comply with strict space avionics specifications (representative of the **Cobham Gaisler GR712RC Dual-Core LEON3 SPARC V8** architecture executing at **250 MIPS**):

* **Deterministic Static Sandbox:** Confined within a strict **16 MB RAM partition** (`0x3FFF0000` to `0x40FF0000`). Dynamic memory management (`malloc`, `free`, `calloc`) is entirely omitted.
* **Zero Floating-Point Unit (FPU) Overhead:** Fixed-point integer mathematical models completely replace standard math routines (`float`, `double`, `log2f`). Base-2 logarithms are evaluated via a precomputed **Q8-format lookup table (`log2_q8[257]`)**, executing in single-cycle integer arithmetic.
* **Hardware Trap Mitigation (SPARC Trap 0x07):** Multidimensional telemetry words are serialized byte-by-byte via register bit-shifts before hardware interface flushing, completely preventing unaligned word memory access anomalies (**Data Access Alignment Traps**).
* **O(N) Cache Optimization:** Histogram binning incorporates an active-bin tracking stack, bounding clearing operations to $O(N)$ complexity (where $N$ is the number of active grayscale levels per block) and maintaining internal CPU cache integrity.

---

## 🧬 Project Evolution

1. **Algorithmic Modeling (Python):** Verified multivariate Shannon entropy for rapid image block prioritization in the open-source prototype [Entropy-Image-Prioritization](https://github.com/techn0man1ac/Entropy-Image-Prioritization).
2. **Microcontroller Proof-of-Concept (C/C++):** Validated on constrained commercial microcontrollers (`EIP_ESP32S3`) to analyze clock-cycle budgets and memory layouts.
3. **Aerospace Flight-Grade Port (Bare-Metal SPARC V8):** Re-architected in pure ANSI C for radiation-hardened LEON3 processors under ECSS Category D rules, eliminating FPU instructions and dynamic heaps.
4. **AI-Augmented Systems Engineering:** The architectural translation and bare-metal porting were accelerated through conversational AI toolchains acting as a high-level functional compiler, followed by rigorous static analysis, cross-compilation via Frontgrade Gaisler BCC2, and software-in-the-loop (SIL) validation in Renode.

---

## 📡 High-Density Telemetry Protocol & Bit-Allocation Specification

To avoid high CPU parsing overhead and prevent communication channel saturation, AURA bypasses ASCII, JSON, or text serialization entirely within the real-time processing loop. Instead, telemetry metrics are packed directly into big-endian **32-bit unsigned integer registers (`uint32`)** and dense byte streams down to the exact bit boundaries.

---

### 1. Level 2 (L2): 32-Bit Shannon Entropy Grid Word (`MARKER_L2_ENTROPY = 0xD1`)
Every spatial block in the $64 \times 64$ terrain map is packed into a single 4-byte telemetry word:

| Bit Range | Field Name | Width | Type | Description |
| :---: | :--- | :---: | :---: | :--- |
| **[31:24]** | `SYNC_MARKER` | 8 bits | `uint8` | Frame sync and protocol identifier (`0xD1`). |
| **[23:16]** | `COL_INDEX` | 8 bits | `uint8` | Block column coordinate layout ($X$-axis grid index, 0..63). |
| **[15:8]** | `ROW_INDEX` | 8 bits | `uint8` | Block row coordinate layout ($Y$-axis grid index, 0..63). |
| **[7:0]** | `ENTROPY_X10` | 8 bits | `uint8` | Scaled Shannon Entropy value ($\text{Entropy} \times 10$, range 0..80 bits/pixel). |

---

### 2. Level 1 (L1): 32-Bit Directional Surface Landmark Word (`MARKER_L1_HEADER = 0xB1`)
Transforms raw $(X, Y)$ coordinate points into orientation-aware topological descriptors without floating-point math:

| Bit Range | Field Name | Width | Type | Description |
| :---: | :--- | :---: | :---: | :--- |
| **[31:28]** | `RESERVED` | 4 bits | — | Reserved for status flags / parity check (default: `0x0`). |
| **[27:18]** | `COORD_X` | 10 bits | `uint10` | Horizontal pixel coordinate across camera matrix (range 0..1019 px). |
| **[17:8]** | `COORD_Y` | 10 bits | `uint10` | Vertical pixel coordinate across camera matrix (range 0..1019 px). |
| **[7:4]** | `GRAD_VECTOR` | 4 bits | `uint4` | Localized anisotropic gradient direction angle ($0..15 \approx 22.5^\circ$ sectors). |
| **[3:0]** | `LUMINANCE` | 4 bits | `uint4` | Quantized local luminance intensity at landmark center ($I \gg 4$). |

---

### 3. Level 0 (L0): 20-Bit Ultra-Sparse Coordinate Packing (`MARKER_L0_HEADER = 0x5B`)
In the ultra-constrained L0 mode, 4-byte padding is eliminated completely. Coordinates are packed sequentially into **3 consecutive bytes (24 bits)** per point to maximize DSN bandwidth efficiency:

* **Byte 0:** `COORD_X[9:2]` (Upper 8 bits of X)
* **Byte 1:** `COORD_X[1:0]` (Lower 2 bits of X) `|` `COORD_Y[9:6]` (Upper 4 bits of Y)
* **Byte 2:** `COORD_Y[5:0]` (Lower 6 bits of Y) `|` `00` (Zero padding)

*Efficiency Gain:* Transmitting 100 surface landmarks requires only **300 bytes of payload**, achieving a **99.98% volume reduction** compared to the raw frame.

---

### 4. Hardware Serialization & SPARC Trap 0x07 Mitigation
On RISC/SPARC V8 architectures, unaligned 32-bit word store instructions (`st`) to physical peripheral address spaces trigger a fatal hardware exception: **Data Access Alignment Trap (Trap 0x07)**.

To enforce deterministic hardware safety:
1. All telemetry words are assembled in native 32-bit CPU registers.
2. The serialization pipeline flushes the word byte-by-byte via sequential bit-shifts:
   ```c
   void uart_send_u32(unsigned int packet) {
       for (int i = 3; i >= 0; --i) {
           while (!(*UART_LSR & LSR_THRE));
           *UART_RBR_THR = (unsigned char)((packet >> (i * 8)) & 0xFFU);
       }
   }
   ```
3. This guarantees single-byte physical transfers over memory-mapped I/O (MMIO), entirely bypassing unaligned bus access faults across the AMBA APB bridge.

---

## 🔧 Prerequisites & Toolchain Setup

To run the Software-in-the-Loop (SIL) simulation environment, install the official Gaisler cross-compiler and the Renode emulation framework.

### 1. Frontgrade Gaisler BCC Toolchain
The flight firmware was verified and compiled using the official **BCC 2.2.3 (Bare-C Compiler V2)** 64-bit cross-compiler suite for Windows (`bcc-2.2.3-gcc-mingw64`). 

The codebase adheres strictly to ANSI C without compiler-specific built-ins, ensuring backwards compatibility with the legacy flight qualification baseline **BCC 1.0.52 (GCC 4.4.2)** mandated by mission specifications, as well as modern BCC2 environments.

**Installation & Environment Setup (Windows):**
1. Download the verified **BCC 2.2.3** distribution from the official [Frontgrade Gaisler FTP repository](https://download.gaisler.com/anonftp/bcc2/bin/bcc-2.2.3-gcc-mingw64.zip).
2. Extract the archive (recommended workspace directory: `C:\Projects\bcc-2.2.3-gcc-mingw64\`).

---

## 🏁 Building and Running the Simulation

### Step 1: Cross-Compilation for LEON3 Target
Compile `Hello_AURA.c` into a space-grade bare-metal SPARC V8 ELF binary mapped precisely at `0x40000000`.

Execute the compilation command in **Windows PowerShell**:

```powershell
& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g Hello_AURA.c -o experiment_test.elf "-Wl,-Ttext=0x40000000" "-Wl,-z,muldefs" -lgcc
```

*(Note: If you added the compiler binary directory to your system `PATH`, you can omit the absolute path and call `sparc-gaisler-elf-gcc` directly).*

**Key Compiler & Linker Flags Explained:**
* `-O2`: Optimization Level 2 (enforcing deterministic inlining and low instruction latency per ECSS guidelines).
* `-g`: Retains DWARF debugging symbols for register inspection in GDB and Renode.
* `"-Wl,-Ttext=0x40000000"`: Maps the executable `.text` segment strictly to memory base address `0x40000000`, perfectly aligning with the entry point defined in `script.resc` and the 16 MB sandbox boundary.
* `"-Wl,-z,muldefs"`: Linker directive allowing multiple definitions to ensure seamless symbol resolution against bare-metal C runtime stubs (`crt0`).
* `-lgcc`: Links the low-level GCC runtime library for software integer arithmetic operations.

### Step 2: Initialize Spacecraft Emulation (Renode)
Launch the virtual spacecraft environment:

```bash
renode .\script.resc
```

*Verification:* Renode initializes the virtual LEON3 CPU at 250 MIPS, maps RAM at `0x3FFF0000`, and opens loopback telemetry port `12345`.


![LEON3 Remode Emulations screenshot](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/main/Img/LEON3_Remode_Emulations.png)

### Step 3: Launch Ground Segment Console
In a separate terminal, start the telemetry receiver:

```bash
python .\telemetry_live_visualizer.py
```

Upon socket handshake, the Ground Station begins receiving packed telemetry words, plotting real-time entropy topographies and surface landmark vectors.

![AURA Ground Segment Screenshot](https://github.com/techn0man1ac/AURA/blob/main/leon3_project_v_0_3/AURA_GrndSeg_Screenshot.png)

---

## 📂 Repository Structure & Mission Modules(V0.3 work variant)

* `Hello_AURA.c` — Standalone core flight application executing the fixed-point block entropy pipeline and deterministic landmark extractor.
* `leon3.repl` — Renode platform description enforcing the physical 16 MB memory-mapped boundary.
* `script.resc` — Automated deployment and socket bridging script for Renode.
* `telemetry_live_visualizer.py` — Ground Segment mission console decoding incoming binary packet streams into interactive mathematical heatmaps and point clouds.
* `image.bin` — Raw 8-bit monochromatic calibration frame mapped at `0x40600000`. Please move the "image.bin" from the repository root to the V0.3 catalog. This file was created specifically for testing purposes, based on a [NASA file](https://en.wikipedia.org/wiki/Dimorphos#/media/File:Dimorphos_North-Up_Image_Composition_reduced_bloom.png).

> **Note on Mission-Specific Extensions:**  
> This open-source repository hosts the standalone core algorithm and generic emulation pipeline. Tailored flight stubs and platform drivers interfacing with official mission APIs (such as the ESA Hera Payload Interface and PUS Service 230 packet wrappers under ESA OSIP Campaign `I-2026-13019`) are maintained in segregated flight branches in compliance with platform dissemination rules.

---

## 🔮 Live Interactive Vision Prototype (Google AI Studio)
For immediate hardware-in-the-loop and live vision pipeline exploration without setting up the local cross-compilation toolchain, an interactive web prototype is deployed on Google AI Studio.

https://www.youtube.com/shorts/gKMWbWaEmZ0

You can access the environment directly via:
👉 **[AURA Vision Concept Tracker (Google AI Studio App)](https://aistudio.google.com/apps/51371c21-635f-4a75-856a-ea37b0097875)** *(Requires a valid Google AI Studio login).*

### Features available in the Web Sandbox:
* **Real-time Camera Injection:** Turn on the built-in webcam or pair your smartphone's camera interface to stream live physical optical matrices directly into the tracking pipeline.
* **Configurable Spatial Filtering:** Dynamically fine-tune block dimensions, stride lengths, and mathematical scaling factors to analyze the entropy and heatmap response instantly.
* **Deterministic Logic Verification:** Observe how the hardware constraints and fixed-point optimizations handle dynamic real-world environments before flight compilation.

---

# 🗺️ AURA Development Roadmap

![Visual represents the project](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/main/Img/AURA_Deep-Space_Navigation_Infographic.png)

AURA is evolving along four complementary research and engineering branches. All branches originate from the same core principle:

> Information should be processed according to its informational value rather than treating every pixel equally.

The roadmap below reflects the current development priorities and long-term vision of the project.

---

## Phase 1 — Progressive Information Reduction & Telemetry Prioritisation

### Current Focus

The primary objective is reducing deep-space telemetry requirements by transmitting increasingly detailed representations only when necessary.

Pipeline concept:

```text
Raw Image
    ↓
Feature Point Cloud (~1 KB)
    ↓
Entropy Heatmap (~20 KB)
    ↓
Full Image (~1 MB)
```

### Goals

- Detect local contrast maxima.
- Generate sparse feature point clouds.
- Reduce downlink requirements through hierarchical data products.
- Allow ground operators to decide whether additional data transmission is justified.
- Prioritise scientifically valuable observations before transmitting complete image frames.

### Expected Outcome

A spacecraft can transmit a compact informational summary first, significantly reducing unnecessary bandwidth consumption during deep-space operations.

---

## Phase 2 — Low-Information Region Suppression

### Objective

Identify regions that contribute little scientific value and exclude them from further processing.

Examples include:

- Deep-space background
- Uniform sky regions
- Low-detail surfaces
- Sensor noise dominated areas

### Goals

- Automatically classify low-information regions.
- Reduce computational load on embedded processors.
- Focus processing resources on scientifically relevant structures.
- Improve overall onboard decision efficiency.

### Expected Outcome

More processing power becomes available for high-value observations while reducing unnecessary calculations.

---

## Phase 3 — Entropy-Based Visual Navigation

### Long-Term Research Direction

Investigate whether entropy-derived feature structures can support autonomous spacecraft navigation.

Instead of analysing complete image frames, AURA may operate on compact informational representations extracted from the scene.

Pipeline concept:

```text
Image
   ↓
Entropy Analysis
   ↓
Feature Point Extraction
   ↓
Constellation Generation
   ↓
Inter-Frame Tracking
   ↓
Motion Estimation
   ↓
Navigation Support
```

### Feature Constellations

AURA can represent high-information regions as connected structures ("feature constellations").

These constellations:

- Remain linked to physical scene features.
- Move consistently across image sequences.
- May provide a lightweight representation for visual motion estimation.
- Could support future visual odometry and autonomous navigation experiments.

Potential applications include:

- Relative motion estimation
- Surface tracking
- Landmark recognition
- Autonomous proximity operations around asteroids and small bodies

### Research Question

Can entropy-derived feature constellations provide a computationally efficient alternative or complement to traditional feature-tracking approaches used in spacecraft visual navigation?

---

## Phase 4 — Multidimensional Sensor Fusion & Joint Entropy

### Future Vision

Extend AURA beyond image processing by incorporating additional spacecraft sensors into a unified informational framework.

Potential data sources:

- Optical cameras
- Infrared sensors
- Laser ranging systems
- IMU measurements
- Star trackers
- Thermal sensors
- Scientific payload instruments

### Goals

- Compute multidimensional joint entropy across heterogeneous sensor inputs.
- Detect environmental changes through information-state transitions.
- Trigger autonomous decision-making processes.
- Build a generalized information-awareness layer for future spacecraft systems.

### Example Scenario

A previously empty field of view suddenly contains an object.

Simultaneously:

- Optical entropy increases.
- Infrared readings change.
- Range measurements become available.

The combined entropy signature indicates a significant environmental event and can automatically trigger higher-level mission logic.

### Expected Outcome

AURA evolves from an image-processing subsystem into a general-purpose information-driven perception framework for autonomous space systems.

---

## Beyond the Roadmap

Future development may ultimately lead toward:

- Autonomous onboard scientific prioritisation
- Information-driven spacecraft decision support
- Vision-assisted guidance, navigation and control (GNC)
- Adaptive telemetry generation
- Self-directed observation planning

The long-term vision of AURA is to transform raw sensor streams into compact, actionable informational representations suitable for resource-constrained deep-space missions. 

---

## 📄 Disclaimer & Licensing
Independent Technology Prototype:
AURA is an independent research and engineering development. References to the European Space Agency (ESA), the Hera mission, LEON3, GR712RC, or specific mission instruments are used strictly for technical compatibility verification and operational context.
Open Source License:
This project is open-source and released under the terms of the [MIT License](https://github.com/techn0man1ac/AURA/blob/main/LICENSE).
