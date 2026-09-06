## 🏁 Quick Start Instructions

![Screenshot of AURA V0.1.1](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/main/leon3_project_v_0_1_1/Figure_2.png)

Execute all commands from your project root directory (C:\Projects\bcc-2.2.3-gcc-mingw64\leon3_project_v_0_1_1\) inside a Windows PowerShell window.
## Step 1: Cross-Compile the C Source for LEON3
Run this command to compile hello.c into a space-grade bare-metal ELF binary, precisely mapping the text section to memory address 0x40000000:

```powershell
& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g .\hello.c -o .\experiment_test.elf "-Wl,-Ttext=0x40000000" "-Wl,-z,muldefs" -lgcc
```

## Step 2: Launch the Ground Segment Receiver
Open a separate terminal window, navigate to the same folder, and start the universal Python radar interface. It will open port 12345 and wait for the point cloud data stream:

```powershell
python .\telemetry_live_visualizer.py
```

## Step 3: Run the Spacecraft Emulation Framework
Return to your primary terminal or use a new one to boot up the virtual spacecraft. Ensure your script.resc file is modified to load @experiment_test.elf:

```powershell
renode .\script.resc
```

Once Renode initializes the virtual LEON3 CPU, the Python window will instantly receive the packed 0xBD telemetry stream and dynamically overlay green landmark crosshairs corresponding to the asteroid's outer rim and deep craters.

## 🧠 Core Engineering: How It Works & Downlink Efficiency

### Why It Works (The Core Problem)
Deep-space navigation cameras (like the simulated Hera AFC sensor) generate massive data payloads—a single 1020x1020 image requires **1.04 Megabytes** of raw buffer memory. Transmission of such high-volume payloads across millions of kilometers back to Earth is strictly bottlenecked by razor-thin radio downlink windows and severe signal delays. 

### How It Works (The V0.1.1 Algorithm)
Branch **V0.1.1** mitigates this constraint by shifting processing directly to the edge using **Limb-based Optical Navigation**:
1. **Linear Grid Scanning:** The firmware directly processes raw pixels from the physical camera address space (`IMAGE_ADDRESS`) in a highly optimized memory-linear pattern.
2. **Anisotropic Edge Estimation:** For every checked pixel coordinate, an ultra-fast fixed-point cross-gradient operator computes absolute luminance differences along vertical and horizontal vector indices ($|P_{top} - P_{bottom}| + |P_{left} - P_{right}|$).
3. **Contrast Peak Isolation:** If the total structural score breaks the custom contrast barrier (`THRESHOLD`), the software recognizes the point as a critical landmark—such as a deep crater fault or the asteroid’s outer silhouette (limb) against black space.
4. **Deterministic Mitigation (WCET):** The calculation breaks execution instantly once it reaches a strict hard cap (`MAX_LANDMARKS`), preserving the real-time execution budget of the LEON3 CPU.

### Where the Massive Data Savings Come From
The data footprint optimization is achieved by switching from continuous spatial mapping to **Sparse Tokenized Serialization**:
* **The V0.2 Baseline Approach:** The previous iteration evaluated every single image cell, constantly shipping thousands of continuous block matrices over the wire. This configuration saturated the UART channel with **4,624 packets (~18.5 KB)** per frame.
* **The V0.1.1 Sparse Approach:** The new tracking subsystem entirely strips out the concept of a contiguous telemetry map. Instead, it extracts and transmits *only* a sparse array of high-priority coordinate points ($X$, $Y$) and their visual contrast grades, tightly packed into tiny 4-byte packages.
* **The Optimization Factor:** By dropping background blocks and filtering exclusively for the highest contrast vertices, the downlink bandwidth consumption drops to a strict ceiling of **64 to 256 packets (256 bytes to 1 KB)** per frame. This yields an astronomical **95%+ radio traffic reduction** compared to the original telemetry payload.
