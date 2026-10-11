# Deployment & Execution Procedure

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

![AURA Ground Station](https://raw.githubusercontent.com/techn0man1ac/AURA/refs/heads/main/leon3_project_v_0_3/AURA_GrndSeg_Screenshot.png)


### Step 4: Interactive Operation & Analytics Export
1. Click the **"Connect"** button on the Ground Console to establish the synchronous radio link.
2. Select any requested telemetry level (`L0` to `L4`) to command the onboard computer.
3. Adjust the **Shannon Entropy Compression Threshold** slider on the Ground UI and click **"Transmit Gate Command"** to rewrite the spacecraft's data filtering rules in real time.
4. **Data Export:** Right-click on any active plot component (Landmark View, Entropy Map, or Science Frame) to trigger a native **1020x1020 high-fidelity export** saved via Pillow directly as a raw array.
5. **Mission Logs:** Open the `logs/aura_telemetry_log.csv` directory to analyze byte counts, compression ratios, and transmission timestamps.
