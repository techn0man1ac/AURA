import os
from pathlib import Path

def generate_hera_header_failproof():
    target_dir = Path(r"C:\Projects\bcc-2.2.3-gcc-mingw64\OSIP\AFC_images\AFC_IMAGES")
    
    if not target_dir.exists() or not target_dir.is_dir():
        print(f"[ERROR] Directory not found: {target_dir}")
        return

    print(f"=== Starting Fail-Proof Technical Audit of: {target_dir.resolve()} ===")
    all_files = os.listdir(target_dir)

    # Пряме фільтрування за розширеннями файлів (без регулярних виразів)
    bin_files = sorted([f for f in all_files if f.lower().endswith('.bin') and not f.startswith("._")])
    png_files = sorted([f for f in all_files if f.lower().endswith('.png') and not f.startswith("._")])
    txt_files = sorted([f for f in all_files if f.lower().endswith('.txt') and not f.startswith("._")])

    expected_bin_size = 1020 * 1020  # 1,040,400 байт
    valid_bin_files = []

    # Перевірка розміру кожного бінарного файлу на відповідність специфікації камери AFC
    for bin_name in bin_files:
        file_path = target_dir / bin_name
        if file_path.stat().st_size == expected_bin_size:
            valid_bin_files.append(bin_name)

    print("\n=======================================================")
    print("🛰️ HERA AFC DATASET VALIDATION REPORT (DIRECT EXTRACTION)")
    print("=======================================================")
    print(f"• Total Files in Folder: {len(all_files)}")
    print(f"• Detected Raw BIN Files: {len(bin_files)}")
    print(f"• Detected PNG Previews: {len(png_files)}")
    print(f"• Detected Metadata TXT Logs: {len(txt_files)}")
    print(f"• Hardware-Compliant BIN Matrices (1020x1020): {len(valid_bin_files)}")
    print("=======================================================\n")

    # ГЕНЕРАЦІЯ ЗАГОЛОВНОГО ФАЙЛУ IMAGES_DATA.H (СТАНДАРТ ANNEX D МІСІЇ HERA)
    header_path = target_dir / "images_data.h"
    
    if len(valid_bin_files) > 0:
        first_bin_name = valid_bin_files[0]
        first_bin_path = target_dir / first_bin_name

        print(f"[AURA] Compiling C-Header static array from flight matrix: {first_bin_name}")
        
        with open(first_bin_path, "rb") as bin_f:
            raw_bytes = bin_f.read()

        with open(header_path, "w", encoding="utf-8") as h_f:
            h_f.write("#ifndef IMAGES_DATA_H\n#define IMAGES_DATA_H\n\n")
            h_f.write(f"// Generated automatically from Hera Flight Bank. Source: {first_bin_name}\n")
            h_f.write("#define IMG_WIDTH 1020\n")
            h_f.write("#define IMG_HEIGHT 1020\n")
            h_f.write("#define IMG_SIZE (IMG_WIDTH * IMG_HEIGHT)\n\n")
            
            h_f.write(f"// Raw 8-bit Grayscale telemetry pixel stream\n")
            h_f.write(f"const unsigned char afc_frame_test_bin[] = {{\n    ")
            
            # Побайтовий дамп матриці у шістнадцятковому форматі (Hex Dump)
            hex_bytes = [f"0x{b:02X}" for b in raw_bytes]
            for i in range(0, len(hex_bytes), 16):
                h_f.write(", ".join(hex_bytes[i:i+16]))
                if i + 16 < len(hex_bytes):
                    h_f.write(",\n    ")
            
            h_f.write("\n};\n\n")
            h_f.write("#define AFC_TEST_IMAGES 1\n")
            h_f.write("const unsigned char* const afc_images_bank[AFC_TEST_IMAGES] = {\n")
            h_f.write("    afc_frame_test_bin\n};\n\n")
            h_f.write("#endif // IMAGES_DATA_H\n")
            
        print(f"[SUCCESS] Official Annex D Header compiled and saved to:\n--> {header_path.resolve()}")
    else:
        print("[ERROR] No hardware-compliant 1020x1020 .bin files found to generate the header.")

if __name__ == "__main__":
    generate_hera_header_failproof()
