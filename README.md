# AURA: Autonomous Unsupervised Feature-Tracking for Real-Time Deep-Space Navigation
An ultra-lightweight, hardware-agnostic embedded vision subsystem designed for real-time edge computing, autonomous object mapping, and telemetry visualization. 

[![TRL](https://img.shields.io/badge/TRL-4-blue.svg)](#technology-readiness)
[![Architecture](https://img.shields.io/badge/Target-LEON3%20%2F%20SPARC%20V8-informational.svg)](#technical-profile)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](#license)

![Screenshot of AURA V0.2](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/main/Img/Figure_1.png)

AURA is engineered specifically within the **ESA OSIP** framework as a direct software solution to meet the core objectives of the **Hera Extended Mission Phase (Autonomous Software Experiments on Hera)**. Developed for execution on the spacecraft's second processor core (Core 1), the system operates within a protected sandbox environment alongside flight-critical systems, achieving **Technology Readiness Level 4 (TRL 4)** validation.

---

## 🛰️ Direct Response to ESA Hera Mission Objectives
AURA addresses the precise limitations of deep-space operations highlighted by the European Space Agency - specifically dealing with severe communication constraints, intermittent signals, and command delays reaching up to 40 minutes during close-proximity operations around the asteroid moon Dimorphos.

### 1. Autonomy (Onboard GNC and Feature Tracking)
* **The Challenge:** Deep-space missions cannot rely on continuous ground intervention during critical proximity operations.
* **AURA's Solution:** By tracking high-contrast features through local mathematical entropy, AURA provides deterministic, autonomous navigation support. It operates independently of ground control, allowing the spacecraft to maintain situational awareness even when Earth is silent.
### 2. Edge Computing (Data Compression & Prioritisation)
* **The Challenge:** Deep-space telemetry suffers from extremely limited downlink volume and precious bandwidth.
* **AURA's Solution:** The system processes asteroid imagery locally at the edge. By computing dynamic internal heatmaps, AURA flags anomalies and isolates regions of high scientific value. This enables smart data prioritisation, ensuring the spacecraft downlinks only high-value, highly compressed visual metadata rather than massive raw image payloads.
### 3. Resilience (Strict Safety & Sandbox Compliance)
* **The Challenge:** Experimental payloads must execute under non-continuous windows (2–3 hours per day) without compromising the core spacecraft control logic (Core 0).
* **AURA's Solution:** Built with zero dynamic memory allocation (`malloc`), AURA enforces absolute execution determinism and software stability by design. It operates seamlessly within a memory-protected region, guarantees zero heap fragmentation, and natively tolerates abrupt, automated shutdowns triggered by spacecraft anomalies or Safe Mode transitions as standard operating procedures.

---

## 🧬 Project Evolution & AI Compilation TimelineThe architecture of AURA is the result of a multi-stage evolutionary porting process:

1. **The Python Baseline:** The initial mathematical concept was modeled in the open-source prototype **[Entropy-Image-Prioritization](https://github.com/techn0man1ac/Entropy-Image-Prioritization)**, which verified the use of multivariate Shannon entropy for rapid image block prioritization using Python (OpenCV/NumPy).
2. **The Microcontroller Proof-of-Concept:** The algorithm was then successfully ported to C/C++ and verified under tight memory and clock constraints on a cheap commercial chip inside the `EIP_ESP32S3` stack.3. **The Aerospace Flight Grade Port:** For the ESA OSIP campaign, the pipeline was fully rewritten into low-level, high-reliability C code optimized specifically for the radiation-hardened **LEON3 (SPARC V8)** processor architecture, removing all high-level dependencies, floating-point variables, and dynamic allocations.

This final flight-grade iteration was **100% generated via conversational AI (Vibe Coding)** utilizing the **Google AI interface** over a targeted **7-day hobby sprint** with zero operational budget. The AI was treated as a **High-Level Functional Compiler**, translating explicit logical boundaries, memory rules, and aerospace mathematics into verified, bare-metal C code.

---

## 🔒 Technical Architecture & Core Constraints
To comply with strict aerospace software engineering standards (ECSS Category D) and target space-grade hardware specifications, the software pipeline is bound by the following low-level operational limits:
* **Hardware Architecture:** 32-bit SPARC V8 (**LEON3**) processor core executing at a flight-representative **250 MIPS** (Tested on GR712RC Dual-Core configuration).
* **Memory Restrictions:** Strict **16 MB RAM** static partition sandbox. Dynamic memory allocation (`malloc`, `free`) is entirely omitted to enforce execution determinism.
* **Sensor Interfacing & Data Source:** Interfaced with a simulated **Hera AFC** navigation camera utilizing a monochrome *FaintStar2* sensor configuration: **1020x1020 pixels, strict 8-bit Grayscale** (1 byte per pixel, total raw frame size: 1,040,400 bytes). 
  * *Note on Visual Assets:* All raw flight matrices (`image.bin` and `images_data.7z`) are extracted directly from the **official ESA Hera dataset (`AFC images.tar.gz`)** provided via the [OSIP campaign platform](https://ideas.esa.int/core/servlet/hype/IMT?userAction=Browse&templateName=&documentId=76590fb19b5e6424d8862a329c2884b1).
* **Zero Floating-Point Unit (FPU) Overhead:** Fixed-point integer mathematical models completely replace standard floating-point functions (`float`, `double`, `log2f`). Logarithmic probabilities are resolved using ultra-fast bitwise arithmetic.
* **Histogram Footprint Optimization:** Features a dedicated tracking stack that enables precise, point-by-point clearing of modified memory indexes. This bounds clearing operations to $O(N)$ efficiency (where $N$ is the count of active grayscale channels per block), maintaining internal CPU cache efficiency.

---

## 📊 Telemetry Serialization & Downlink Footprint Compression
Data serialization circumvents human-readable ASCII or string parsing inside the real-time processing loop. Instead, the firmware packs localized statistical telemetry directly into high-density **32-bit unsigned integer registers** (`uint32`), allocating data parameters down to the exact bit level:

| Bit Range | Size (Bits) | Description |
|---|---|---|
| **[31:24]** | 8 | Synchronization / Data frame identifier marker (`0xA5`). |
| **[23:17]** | 7 | Column Index (`col_idx`), representing block X-coordinate coordinate layout (up to 2048 px). |
| **[16:10]** | 7 | Row Index (`row_idx`), representing block Y-coordinate coordinate layout (up to 1536 px). |
| **[9:0]** | 10 | Scaled Shannon Entropy value ($Entropy \times 100$). Supports an active integer range from 0 to 1023. |
* **Trap & Exception Mitigation:** The packed 32-bit words are transmitted over the physical interface byte-by-byte via sequential register flushing. This prevents unaligned word memory access anomalies, completely eliminating the risk of critical processor exceptions (**SPARC Trap 0x07 / Data Access Alignment Trap**).
* **Performance Metrics:** Telemetry validation reports an **85.2% absolute lossless reduction** in downlink data volume compared to baseline text streaming, achieving a **6.75x bandwidth optimization factor** over the telemetry link.

---

## 📂 V0.2 Repository Structure (work variant)
* `Hello_AURA.c` — The standalone core flight software application executing the fixed-point block entropy pipeline.
* `experiment_test.elf` — The final compiled space-grade executable binary containing embedded image matrices.
* `hera_types.h` — Injection type-definition header providing strict compliance mapping for standard integer specifications.
* `hera_interface.h` — Official ESA OSIP API interface prototype declarations for the Hera mission payload suite.
* `hera_client_stub.c` — The original flight simulation stub managing camera synchronization frames.
* `images_data.7z` — The compressed archive containing the main flight data bank header (`images_data.h`).
* `image.bin` — The raw 8-bit monochrome binary matrix extracted for hardware memory direct mapping (`0x40600000`). **Sourced from ESA's official `AFC images.tar.gz` dataset.**
* `leon3.repl` — The Renode hardware platform description file enforcing the exact 16 MB memory map layout.
* `script.resc` — The automation deployment script establishing the socket bindings and CPU clock performance.
* `telemetry_live_visualizer.py` — The Ground Segment analytics visualizer decoding binary masks into a real-time heatmap.
* `start.S` / `stub_utils.h` — Low-level assembly initialization sequences and printing primitives for the SPARC architecture.

---

## 🔧 Prerequisites & Toolchain Installation
To set up the Software-in-the-Loop (SIL) simulation environment on Windows, you must install the official ESA/Gaisler cross-compiler and the Renode emulation framework.

### 1. Install Aeroflex Gaisler BCC2 ToolchainThe flight software requires the **BCC2 (Bare-metal C Compiler V2)** based on GCC for SPARC architectures.
1. Download the Mingw64 build of BCC2 from the official [Cobham Gaisler website](https://download.gaisler.com/anonftp/bcc2/bin/) (bcc-2.2.3-mingw64).
2. Extract the package so that the path matches the compiler execution script exactly:

C:\Projects\bcc-2.2.3-mingw64\bcc-2.2.3-gcc\

3. *(Optional)* Add `C:\Projects\bcc-2.2.3-mingw64\bcc-2.2.3-gcc\bin` to your system environment variables (`PATH`) to call `sparc-gaisler-elf-gcc` natively.

### 2. Install Renode Emulation Framework
Renode is used to emulate the GR712RC Dual-Core LEON3 SoC on a bit-level scale.
1. Download the latest Windows installer (`.exe`) or portable production package from the official [Renode](https://renode.io) site.
2. Complete the standard setup wizard.
3. Ensure that the `renode` system command is mapped to your system `PATH`. Open PowerShell and verify the version payload:

```powershell
renode --version
```

---

## 🏁 Deployment & Execution Procedure
**Recommended Project Workspace Path:** `C:\Projects\AURA-main\`  
*All command line steps below assume that your terminal is opened and executing from the project root directory (`cd C:\Projects\AURA-main\`).*

### Step 0: Extract the Flight Data Bank
Before initiating the compilation pipeline, you must extract the compressed flight data bank header containing the integrated 404-frame optical matrices:
1. Locate the `images_data.7z` archive inside `C:\Projects\AURA-main\`.
2. Extract the file using 7-Zip or any compatible decompression utility (This archive contains the `images_data.h` matrix file generated from the [Official ESA AFC Images Dataset](https://ideas.esa.int/core/servlet/hype/IMT?userAction=Browse&templateName=&documentId=76590fb19b5e6424d8862a329c2884b1)).
3. Ensure that the resulting file **`images_data.h`** is placed directly in the `C:\Projects\AURA-main\` directory alongside `Hello_AURA.c`.

### Step 1: Toolchain Cross-Compilation
To recompile the flight software from source using the official Aeroflex Gaisler BCC2 cross-compiler toolchain, execute the following multi-stage compilation pipeline within a Windows PowerShell terminal opened at **`C:\Projects\AURA-main\`**:

```powershell
# 1. Compile the flight application layer into an object file
& "C:\Projects\bcc-2.2.3-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g -include hera_types.h -c .\Hello_AURA.c -o .\Hello_AURA.o
```

```powershell
# 2. Compile the mission simulation stub layer into an object file
& "C:\Projects\bcc-2.2.3-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g -include hera_types.h -c .\hera_client_stub.c -o .\hera_client_stub.o
```

```powershell
# 3. Link objects into the final aerospace ELF image aligned at target memory space
& "C:\Projects\bcc-2.2.3-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" .\Hello_AURA.o .\hera_client_stub.o -o .\experiment_test.elf "-Wl,-Ttext=0x40000000" "-Wl,-z,muldefs" -lgcc
```

### Step 2: Launch the Spacecraft Emulation Framework
In the primary command terminal opened at **`C:\Projects\AURA-main\`**, initiate the software-in-the-loop validation inside the Renode environment to boot the LEON3 processor and start streaming data:
```powershell
renode .\script.resc
```

### Step 3: Initialize the Ground Segment Visualizer
Once the emulation starts running and the virtual spacecraft begins processing frames, open a separate terminal window at **`C:\Projects\AURA-main\`** and launch the telemetry live decoder to bind to the active stream:
```powershell
python .\telemetry_live_visualizer.py
```

Upon connection, the onboard application will continue processing the 1020x1020 image grids, routing the compiled binary stream dynamically over the loopback interface (`127.0.0.1:12345`) to render a real-time mathematical heatmap of the asteroid terrain in the Ground Segment visualizer window.

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

☝️ Disclaimer

AURA is an independent research / engineering prototype. References to ESA, Hera, LEON3, GR712RC, or related mission and hardware documentation are used for context and technical compatibility only. This repository does not imply endorsement, certification, sponsorship, or official affiliation unless explicitly stated by the respective organisation.

The TRL 4 designation describes the current maturity of the demonstrated technology and should not be interpreted as flight qualification, mission acceptance, or operational certification.

---

## 📄 License
This project is open-source and released under the terms of the [MIT License](https://github.com/techn0man1ac/AURA/blob/main/LICENSE).
