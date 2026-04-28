import os
from PIL import Image

ROOT = r"C:\Users\smith\Desktop\manycore\RM_OLED"
FRAMES_DIR = os.path.join(ROOT, "badapple_frames")
OUT_H = os.path.join(ROOT, "USER", "inc", "bad_apple_frames.h")
OUT_C = os.path.join(ROOT, "USER", "src", "bad_apple_frames.c")

W, H = 128, 64
TH = 128  # threshold

files = sorted([f for f in os.listdir(FRAMES_DIR) if f.lower().endswith(".png")])

def img_to_page_bytes(path):
    img = Image.open(path).convert("L").resize((W, H))
    p = img.load()
    out = bytearray()
    for page in range(H // 8):
        for x in range(W):
            b = 0
            for bit in range(8):
                y = page * 8 + bit
                v = 1 if p[x, y] < TH else 0
                b |= (v << bit)
            out.append(b)
    return out

frames = [img_to_page_bytes(os.path.join(FRAMES_DIR, f)) for f in files]

with open(OUT_H, "w", encoding="ascii") as h:
    h.write("#ifndef __BAD_APPLE_FRAMES_H__\n#define __BAD_APPLE_FRAMES_H__\n\n")
    h.write("#include <stdint.h>\n\n")
    h.write(f"#define BAD_APPLE_FRAME_WIDTH {W}\n")
    h.write(f"#define BAD_APPLE_FRAME_HEIGHT {H}\n")
    h.write(f"#define BAD_APPLE_FRAME_SIZE {W*H//8}\n")
    h.write(f"#define BAD_APPLE_FRAME_COUNT {len(frames)}\n\n")
    h.write("extern const uint8_t bad_apple_frames[BAD_APPLE_FRAME_COUNT][BAD_APPLE_FRAME_SIZE];\n\n")
    h.write("#endif\n")

with open(OUT_C, "w", encoding="ascii") as c:
    c.write('#include "bad_apple_frames.h"\n\n')
    c.write("const uint8_t bad_apple_frames[BAD_APPLE_FRAME_COUNT][BAD_APPLE_FRAME_SIZE] = {\n")
    for fr in frames:
        c.write("    {\n        ")
        for i, b in enumerate(fr):
            c.write(f"0x{b:02X}, ")
            if (i + 1) % 16 == 0 and i + 1 < len(fr):
                c.write("\n        ")
        c.write("\n    },\n")
    c.write("};\n")

print("done:", len(frames), "frames")
print(OUT_H)
print(OUT_C)