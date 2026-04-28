import math
import struct

MIDI_PATH = r"C:\Users\smith\Desktop\manycore\RM_OLED\bad_apple.mid"
OUT_PATH = r"C:\Users\smith\Desktop\manycore\RM_OLED\USER\inc\buzzer_tune_bad_apple.h"

# Trim length to match your video clip (ms)
MAX_MS = 20000
# Aggressive cleanup for buzzer (monophonic PWM)
MIN_MS = 120          # merge anything shorter than this
QUANT_MS = 60         # quantize duration to this grid (ms), set 0 to disable
FREQ_MIN = 220        # drop notes outside range
FREQ_MAX = 3000

def read_vlq(data, i):
    v = 0
    while True:
        b = data[i]
        i += 1
        v = (v << 7) | (b & 0x7F)
        if not (b & 0x80):
            break
    return v, i

def midi_note_to_freq(note):
    return int(round(440.0 * (2.0 ** ((note - 69) / 12.0))))

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def quantize_ms(ms):
    if QUANT_MS <= 0:
        return ms
    # Round to nearest grid, keep at least 1 grid.
    q = int(round(ms / QUANT_MS)) * QUANT_MS
    if q <= 0:
        q = QUANT_MS
    return q

with open(MIDI_PATH, "rb") as f:
    d = f.read()

if d[0:4] != b"MThd":
    raise RuntimeError("Not a MIDI file")

hdr_len = struct.unpack(">I", d[4:8])[0]
fmt, ntrks, division = struct.unpack(">HHH", d[8:14])
ppqn = division
p = 8 + hdr_len

tempo_us_per_qn = 500000  # default 120 BPM
events = []  # (abs_tick, type, note, vel)

for _ in range(ntrks):
    if d[p:p+4] != b"MTrk":
        raise RuntimeError("Bad track header")
    trk_len = struct.unpack(">I", d[p+4:p+8])[0]
    trk = d[p+8:p+8+trk_len]
    p += 8 + trk_len

    i = 0
    abs_tick = 0
    running = None
    while i < len(trk):
        delta, i = read_vlq(trk, i)
        abs_tick += delta
        status = trk[i]
        if status < 0x80:
            if running is None:
                raise RuntimeError("Running status error")
            status = running
        else:
            i += 1
            running = status

        if status == 0xFF:  # meta
            meta_type = trk[i]
            i += 1
            ln, i = read_vlq(trk, i)
            payload = trk[i:i+ln]
            i += ln
            if meta_type == 0x51 and ln == 3:
                tempo_us_per_qn = (payload[0] << 16) | (payload[1] << 8) | payload[2]
            continue

        if status in (0xF0, 0xF7):  # sysex
            ln, i = read_vlq(trk, i)
            i += ln
            continue

        evt = status & 0xF0
        if evt in (0x80, 0x90):
            note = trk[i]
            vel = trk[i+1]
            i += 2
            if evt == 0x90 and vel > 0:
                events.append((abs_tick, "on", note, vel))
            else:
                events.append((abs_tick, "off", note, vel))
        elif evt in (0xA0, 0xB0, 0xE0):
            i += 2
        elif evt in (0xC0, 0xD0):
            i += 1
        else:
            pass

events.sort(key=lambda x: x[0])

# Build monophonic line: highest active note
active = set()
last_tick = 0
segments = []  # (note_or_0, dt_tick)

for tick, typ, note, vel in events:
    if tick > last_tick:
        cur = max(active) if active else 0
        segments.append((cur, tick - last_tick))
        last_tick = tick
    if typ == "on":
        active.add(note)
    else:
        active.discard(note)

# Convert ticks->ms
ms_per_tick = (tempo_us_per_qn / 1000.0) / ppqn
notes = []
total = 0

for note, dt_tick in segments:
    dur = int(round(dt_tick * ms_per_tick))
    if dur <= 0:
        continue
    if total + dur > MAX_MS:
        dur = MAX_MS - total
    if dur <= 0:
        break
    freq = midi_note_to_freq(note) if note > 0 else 0
    notes.append((freq, dur))
    total += dur
    if total >= MAX_MS:
        break

# Merge adjacent same freq
merged = []
for f, t in notes:
    if merged and merged[-1][0] == f:
        merged[-1] = (f, merged[-1][1] + t)
    else:
        merged.append((f, t))

# Drop out-of-range frequencies (treat as rest), then merge again.
filtered = []
for f, t in merged:
    if f != 0 and (f < FREQ_MIN or f > FREQ_MAX):
        f = 0
    filtered.append((f, t))

merged = []
for f, t in filtered:
    if merged and merged[-1][0] == f:
        merged[-1] = (f, merged[-1][1] + t)
    else:
        merged.append((f, t))

# Merge short segments into previous one, then quantize durations.
clean = []
for f, t in merged:
    if not clean:
        clean.append((f, t))
        continue
    if t < MIN_MS:
        # absorb into previous segment
        pf, pt = clean[-1]
        clean[-1] = (pf, pt + t)
    else:
        clean.append((f, t))

merged = []
for f, t in clean:
    qt = quantize_ms(t)
    if merged and merged[-1][0] == f:
        merged[-1] = (f, merged[-1][1] + qt)
    else:
        merged.append((f, qt))

with open(OUT_PATH, "w", encoding="ascii") as f:
    f.write("#ifndef __BUZZER_TUNE_BAD_APPLE_H__\n")
    f.write("#define __BUZZER_TUNE_BAD_APPLE_H__\n\n")
    f.write("#include <stdint.h>\n\n")
    f.write("typedef struct { uint16_t freq_hz; uint16_t dur_ms; } BuzzerNote_t;\n\n")
    f.write(f"#define BAD_APPLE_TUNE_LEN {len(merged)}\n")
    f.write("static const BuzzerNote_t g_bad_apple_tune[BAD_APPLE_TUNE_LEN] = {\n")
    for freq, dur in merged:
        f.write(f"    {{{freq}, {dur}}},\n")
    f.write("};\n\n#endif\n")

print("done")
print("notes:", len(merged))
print("out:", OUT_PATH)