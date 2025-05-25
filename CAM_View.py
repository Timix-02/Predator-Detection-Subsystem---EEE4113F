import serial
from PIL import Image
import time

# === CONFIGURATION ===
PORT = "COM4"         # Replace with your serial port (e.g. /dev/ttyUSB0 or COM3)
BAUD = 460800         # Match your STM32 baud rate
WIDTH = 320           # Set image width
HEIGHT = 240          # Set image height

# === FUNCTION: Convert RGB565 to RGB888 ===
def rgb565_to_rgb888(byte1, byte2):
    value = byte2 << 8 | byte1
    r = (value >> 11) & 0x1F
    g = (value >> 5) & 0x3F
    b = value & 0x1F
    r = (r << 3) | (r >> 2)
    g = (g << 2) | (g >> 4)
    b = (b << 3) | (b >> 2)
    return (r, g, b)

# === MAIN LOGIC ===
ser = serial.Serial(PORT, BAUD, timeout=5)


# --- Step 1: Send START to STM32 ---
ser.write(b"START\n")


# --- Step 2: Read until 'END' ---
raw_image = b""
raw_image = ser.read_until('END')
raw_image = raw_image[:-3]
ser.close()
print("Sent START command to STM32, waiting for data...")
print(f"Received {len(raw_image)} bytes.")

pixels = [rgb565_to_rgb888(raw_image[i], raw_image[i+1]) for i in range(0, len(raw_image), 2)]

img = Image.new("RGB", (WIDTH, HEIGHT))
img.putdata(pixels)
img = img.transpose(Image.FLIP_TOP_BOTTOM)
img.show()
img.save("received_image.png")
