from PIL import Image

# 1. Open any input JPG image of your choice
img = Image.open("MilkaCat.jpg")

# 2. Force resize to high-resolution deep-space camera standard 2048x1536
img = img.resize((2048, 1536))

# 3. Ensure full 24-bit color RGB channel mode (R, G, B)
img_rgb = img.convert("RGB")

# 4. Save as raw color binary byte stream (2048 * 1536 * 3 = 9,437,184 bytes)
with open("image.bin", "wb") as f:
    f.write(img_rgb.tobytes())

print("High-res image.bin successfully created! Size:", img_rgb.tobytes().__len__(), "bytes.")
