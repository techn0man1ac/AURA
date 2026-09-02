import os
from pathlib import Path
from PIL import Image

def convert_raw_bin_to_png(bin_file_path, output_format="PNG"):
    bin_path = Path(bin_file_path)
    
    if not bin_path.exists():
        print(f"[ERROR] Source binary file not found: {bin_path}")
        return

    expected_size = 1020 * 1020  # Ліміт матриці камери Hera AFC (1,040,400 байт)
    file_size = bin_path.stat().st_size
    
    if file_size != expected_size:
        print(f"[ERROR] Non-compliant file size: {file_size} bytes.")
        print(f"Expected exactly {expected_size} bytes for 1020x1020 Grayscale image.")
        return

    print(f"[AURA] Reading raw aerospace binary array: {bin_path.name}")
    
    # Зчитуємо сирі байти з диска
    with open(bin_path, "rb") as f:
        raw_bytes = f.read()

    # Створюємо об'єкт зображення з сирих байтів
    # Режим "L" означає 8-бітний Grayscale (відтінки сірого)
    img = Image.frombytes("L", (1020, 1020), raw_bytes)

    # Формуємо ім'я вихідного файлу
    output_ext = ".png" if output_format.upper() == "PNG" else ".jpg"
    output_path = bin_path.with_suffix(output_ext)

    # Збереження на диск
    img.save(output_path, format=output_format)
    print(f"[SUCCESS] Image generated and saved to:\n--> {output_path.resolve()}")

if __name__ == "__main__":
    # Вкажіть ім'я будь-якого .bin файлу з вашої папки AFC_IMAGES
    target_bin = r"C:\Projects\bcc-2.2.3-gcc-mingw64\OSIP\AFC_images\AFC_IMAGES\100005.49997997284 AFC_0 Guidance TM(139,14) APID(292).bin"
    
    # Конвертуємо у PNG (або вкажіть "JPEG")
    convert_raw_bin_to_png(target_bin, output_format="PNG")
