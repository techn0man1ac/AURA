## 🏁 Quick Start Instructions
Execute all commands from your project root directory (C:\Projects\bcc-2.2.3-gcc-mingw64\leon3_project_v_0_1_1\) inside a Windows PowerShell window.
## Step 1: Cross-Compile the C Source for LEON3
Run this command to compile hello.c into a space-grade bare-metal ELF binary, precisely mapping the text section to memory address 0x40000000:

& "C:\Projects\bcc-2.2.3-gcc-mingw64\bcc-2.2.3-gcc\bin\sparc-gaisler-elf-gcc.exe" -O2 -g .\hello.c -o .\experiment_test.elf "-Wl,-Ttext=0x40000000" "-Wl,-z,muldefs" -lgcc

## Step 2: Launch the Ground Segment Receiver
Open a separate terminal window, navigate to the same folder, and start the universal Python radar interface. It will open port 12345 and wait for the point cloud data stream:

python .\telemetry_live_visualizer.py

## Step 3: Run the Spacecraft Emulation Framework
Return to your primary terminal or use a new one to boot up the virtual spacecraft. Ensure your script.resc file is modified to load @experiment_test.elf:

renode .\script.resc

Once Renode initializes the virtual LEON3 CPU, the Python window will instantly receive the packed 0xBD telemetry stream and dynamically overlay green landmark crosshairs corresponding to the asteroid's outer rim and deep craters.