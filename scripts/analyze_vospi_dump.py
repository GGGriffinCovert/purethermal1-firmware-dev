#!/usr/bin/env python3
"""
analyze_vospi_dump.py - decode a raw memory dump of lepton_buffers[] taken
from a PureThermal over SWD, and report how the received SPI data lines up
with VoSPI packet framing.

Taking the dump (STM32CubeProgrammer, *Hot plug* mode so the MCU is not reset):
  1. grep lepton_buffers main.map                 -> base address
  2. Memory & File editing: Address = base, Size = 0xF060  (4 x 15384 bytes)
  3. Read, then Save As... -> lepton_buffers.bin
  4. python3 scripts/analyze_vospi_dump.py lepton_buffers.bin

Verdicts, per buffer:
  HEALTHY         packets 0..59 in order, headers at byte 0 of each record.
  ALIGNED/PARTIAL headers at byte 0, but the read started or ended at the
                  wrong packet (a timing problem, fixable by walking packets).
  FRAMING OFFSET  VoSPI headers sit k halfwords into every 244-byte record:
                  the MCU's packet boundaries and the sensor's disagree. The
                  resync loop moves in whole records, so it can never fix
                  this. Only a VoSPI re-sync (/CS high + SCK idle > 185 ms)
                  or a sensor reset can.
  BIT SLIP        headers only appear after shifting the bitstream 1..15
                  bits: the sensor counted a different number of SCK edges
                  than the MCU sent.
  NO VOSPI        nothing packet-like at any offset: idle bus (0xFFFF), or
                  data that did not come from the sensor.

`--selftest` builds synthetic buffers for each case and checks the verdicts.
"""

import argparse
import random
import struct
import sys

RECORD_LINES = 63                 # IMAGE_NUM_LINES + TELEMETRY_MAX_LINES
IMAGE_LINES = 60
PKT_BYTES = {"rgb": 244, "y16": 164}
STRIDE = 15384                    # sizeof(lepton_buffer), -fno-short-enums
NUMBER_OFF = RECORD_LINES * 244   # 15372: uint8_t number
STATUS_OFF = NUMBER_OFF + 4       # 15376: lepton_status status (int)
SEGMENT_OFF = STATUS_OFF + 4      # 15380: uint8_t segment
STATUS_NAMES = {0: "OK", 1: "TRANSFERRING", 2: "RESYNC", 4: "CONTINUE"}
MIN_RUN = 6                       # consecutive packets needed to call it VoSPI

# CRC-16, x^16 + x^12 + x^5 + 1, MSB first. The datasheet does not state the
# seed; 0 is assumed. If a known-good buffer shows 0 CRC passes, ignore CRC.
_CRC_TABLE = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x1021) if _c & 0x8000 else (_c << 1)
    _CRC_TABLE.append(_c & 0xFFFF)


def crc16(data, crc=0):
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def packet_crc(pw):
    """CRC of one packet (list of wire-order words), ID top nibble and CRC
    field zeroed, as the VoSPI spec defines it."""
    data = bytearray()
    for j, w in enumerate(pw):
        if j == 0:
            w &= 0x0FFF
        elif j == 1:
            w = 0
        data += bytes((w >> 8, w & 0xFF))
    return crc16(data)


def is_discard(h):
    return (h & 0x0F00) == 0x0F00


def is_video(h):
    return h is not None and not is_discard(h) and (h & 0x0FFF) < RECORD_LINES


def pkt(h):
    return h & 0x0FFF


def label(h):
    if h is None:
        return "----"
    if h == 0xFFFF:
        return "FFFF"
    if is_discard(h):
        return "disc"
    if is_video(h):
        s = "p%d" % pkt(h)
        if pkt(h) == 20 and (h >> 12) & 7:
            s += "/s%d" % ((h >> 12) & 7)
        return s
    return "%04X" % h


def shifted(words, s):
    """Re-cut the bitstream s bits later: word i = bits [16i+s, 16i+s+16)."""
    if s == 0:
        return words
    return [((a << s) | (b >> (16 - s))) & 0xFFFF for a, b in zip(words, words[1:])]


def best_run(hdrs):
    """Longest stretch of consecutive lines holding video packets n, n+1, ...
    Returns (length, first_line, first_packet)."""
    best = (0, None, None)
    i = 0
    while i < len(hdrs):
        if not is_video(hdrs[i]):
            i += 1
            continue
        j = i
        while j + 1 < len(hdrs) and is_video(hdrs[j + 1]) and pkt(hdrs[j + 1]) == pkt(hdrs[j]) + 1:
            j += 1
        if j - i + 1 > best[0]:
            best = (j - i + 1, i, pkt(hdrs[i]))
        i = j + 1
    return best


def headers(ws, P, k, nlines):
    return [ws[i * P + k] if i * P + k < len(ws) else None for i in range(nlines)]


def search(words, P, nlines):
    """Try every bit shift and halfword offset; best framing first."""
    out = []
    for s in range(16):
        ws = shifted(words, s)
        for k in range(P):
            L, line0, p0 = best_run(headers(ws, P, k, nlines))
            out.append((L, s, k, line0, p0))
    out.sort(key=lambda r: (-r[0], r[1], r[2]))
    return out


def crc_passes(words, P, s, k, line0, L):
    ws = shifted(words, s)
    ok = 0
    for i in range(line0, line0 + L):
        pw = ws[i * P + k: i * P + k + P]
        if len(pw) == P and packet_crc(pw) == pw[1]:
            ok += 1
    return ok


def analyze_region(raw, fmt, nlines):
    """raw: bytes of one buffer's lines[] region. Returns a result dict."""
    P = PKT_BYTES[fmt] // 2
    nwords = min(len(raw) // 2, RECORD_LINES * P)
    words = list(struct.unpack("<%dH" % nwords, raw[: nwords * 2]))
    res = search(words, P, nlines)
    L, s, k, line0, p0 = res[0]
    base = next(r for r in res if r[1] == 0 and r[2] == 0)
    h0 = headers(words, P, 0, nlines)
    r = dict(fmt=fmt, P=P, words=words, hdrs=h0, best=res[0], base=base,
             ffff=sum(1 for w in words[: nlines * P] if w == 0xFFFF) / float(nlines * P))
    r["crc"] = crc_passes(words, P, s, k, line0, L) if L else 0
    if L < MIN_RUN:
        r["verdict"] = "NO VOSPI"
    elif (s, k) == (0, 0):
        r["verdict"] = "HEALTHY" if (line0 == 0 and p0 == 0 and L >= IMAGE_LINES) else "ALIGNED/PARTIAL"
    elif s == 0:
        r["verdict"] = "FRAMING OFFSET"
    else:
        r["verdict"] = "BIT SLIP"
    return r


def describe(r, nlines):
    L, s, k, line0, p0 = r["best"]
    P = r["P"]
    v = r["verdict"]
    lines = []
    if v == "HEALTHY":
        lines.append("packets 0..%d in order at lines 0..%d" % (L - 1, L - 1))
    elif v == "ALIGNED/PARTIAL":
        lines.append("headers at byte 0 of each record, but only packets %d..%d (lines %d..%d) are in sequence"
                     % (p0, p0 + L - 1, line0, line0 + L - 1))
        lines.append("-> packet-level timing problem (read started/ended at the wrong packet), not a framing offset")
    elif v in ("FRAMING OFFSET", "BIT SLIP"):
        bits = k * 16 + s
        lines.append("VoSPI headers found %d bits (%d halfwords + %d bits = %.1f bytes) into each %d-byte record"
                     % (bits, k, s, bits / 8.0, P * 2))
        lines.append("packets %d..%d in sequence across lines %d..%d at that offset (record offset 0 gives a run of %d)"
                     % (p0, p0 + L - 1, line0, line0 + L - 1, r["base"][0]))
        lines.append("-> the MCU's packet boundaries and the sensor's disagree. Walking whole records cannot fix this.")
    else:
        lines.append("no run of >= %d sequential packet headers at any bit or halfword offset" % MIN_RUN)
        lines.append("0xFFFF (idle MISO) fraction: %.0f%%" % (100 * r["ffff"]))
    if L >= MIN_RUN:
        lines.append("CRC pass on %d of those %d packets (seed-0 assumption; see header)" % (r["crc"], L))
    return lines


def firmware_view(r, nlines):
    h = r["hdrs"]
    first = h[0] & 0xFF if h[0] is not None else None
    seg = (h[20] >> 12) & 7 if len(h) > 20 and h[20] is not None else None
    last = h[nlines - 1] & 0xFF if h[nlines - 1] is not None else None
    return "first_packet=%s  current_segment=%s  last_end_line=%s (firmware expects 0 / 1..4 / %d)" % (
        first, seg, last, nlines - 1)


def grid(hdrs, per_row=10):
    rows = []
    for i in range(0, len(hdrs), per_row):
        rows.append("    %2d: " % i + " ".join("%-7s" % label(h) for h in hdrs[i: i + per_row]))
    return rows


def run_file(path, fmt, nlines, show_grid):
    data = open(path, "rb").read()
    if len(data) >= STRIDE:
        nbuf = len(data) // STRIDE
        regions = [(i, data[i * STRIDE: i * STRIDE + NUMBER_OFF], data[i * STRIDE: (i + 1) * STRIDE]) for i in range(nbuf)]
        print("%s: %d bytes -> %d lepton_buffer(s) of %d bytes" % (path, len(data), nbuf, STRIDE))
        if len(data) % STRIDE:
            print("  (ignoring %d trailing bytes)" % (len(data) % STRIDE))
    else:
        regions = [(0, data, None)]
        print("%s: %d bytes -> treating as one lines[] region" % (path, len(data)))

    verdicts = []
    for idx, region, whole in regions:
        if fmt == "auto":
            cands = [analyze_region(region, f, nlines) for f in ("rgb", "y16")]
            r = max(cands, key=lambda c: (c["best"][0], c["fmt"] == "rgb"))
            fmt_note = "%s (auto)" % r["fmt"]
        else:
            r = analyze_region(region, fmt, nlines)
            fmt_note = fmt
        print()
        meta = ""
        if whole is not None and len(whole) >= SEGMENT_OFF + 1:
            number = whole[NUMBER_OFF]
            status = struct.unpack_from("<i", whole, STATUS_OFF)[0]
            segment = whole[SEGMENT_OFF]
            meta = "  number=%d status=%s segment=%d" % (number, STATUS_NAMES.get(status, str(status)), segment)
            if number != idx:
                meta += "   <-- expected number=%d: wrong base address or stride?" % idx
        print("buffer %d  [%s]%s" % (idx, fmt_note, meta))
        print("  firmware's view: " + firmware_view(r, nlines))
        if show_grid:
            print("  headers at record offset 0 (what lepton_task checks):")
            for row in grid(r["hdrs"]):
                print(row)
        print("  VERDICT: %s" % r["verdict"])
        for line in describe(r, nlines):
            print("    " + line)
        verdicts.append(r["verdict"])

    print()
    print("summary: " + ", ".join("buf%d=%s" % (i, v) for i, v in enumerate(verdicts)))
    return verdicts


# ----------------------------------------------------------------- self-test

def _synth(fmt, offset_bits, start_pkt=0, idle=False, seed=1):
    rnd = random.Random(seed)
    P = PKT_BYTES[fmt] // 2

    def packet(n, seg=1):
        idw = n | ((seg << 12) if n == 20 else 0)
        pw = [idw, 0] + [rnd.randrange(65536) for _ in range(P - 2)]
        pw[1] = packet_crc(pw)
        return pw

    def discard():
        return [0x0F00 | rnd.randrange(256), rnd.randrange(65536)] + [rnd.randrange(65536) for _ in range(P - 2)]

    stream = []
    for _ in range(2):
        stream += discard()
    for n in range(start_pkt, IMAGE_LINES):
        stream += packet(n)
    for _ in range(8):
        stream += discard()
    if idle:
        stream = [0xFFFF] * len(stream)
    bits, nbits = 0, 0
    for w in stream:
        bits = (bits << 16) | w
        nbits += 16
    start = 2 * P * 16 - offset_bits          # packet `start_pkt` lands offset_bits into record 0
    words = []
    for i in range(RECORD_LINES * P):
        p = start + 16 * i
        words.append((bits >> (nbits - p - 16)) & 0xFFFF if p + 16 <= nbits else 0xFFFF)
    region = struct.pack("<%dH" % len(words), *words)
    buf = bytearray(STRIDE)
    buf[: len(region)] = region
    return bytes(buf)


def selftest():
    cases = [
        ("aligned segment", dict(offset_bits=0), "HEALTHY"),
        ("read began at packet 23", dict(offset_bits=0, start_pkt=23), "ALIGNED/PARTIAL"),
        ("37-halfword framing offset", dict(offset_bits=37 * 16), "FRAMING OFFSET"),
        ("1-halfword framing offset", dict(offset_bits=16), "FRAMING OFFSET"),
        ("5 halfwords + 3 bits", dict(offset_bits=5 * 16 + 3), "BIT SLIP"),
        ("idle bus", dict(offset_bits=0, idle=True), "NO VOSPI"),
    ]
    ok = True
    for name, kw, want in cases:
        buf = _synth("rgb", **kw)
        r = analyze_region(buf[:NUMBER_OFF], "rgb", IMAGE_LINES)
        good = r["verdict"] == want
        if want in ("FRAMING OFFSET", "BIT SLIP"):
            L, s, k, _, _ = r["best"]
            good = good and (k * 16 + s) == kw["offset_bits"] and r["crc"] == L
        if want == "HEALTHY":
            good = good and r["crc"] == IMAGE_LINES
        ok &= good
        print("%-30s -> %-16s %s" % (name, r["verdict"], "ok" if good else "FAIL (wanted %s, best=%s crc=%d)"
                                      % (want, r["best"], r["crc"])))
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("dump", nargs="?", help="raw .bin dump of lepton_buffers[] (or of one buffer)")
    ap.add_argument("--format", choices=["auto", "rgb", "y16"], default="auto",
                    help="packet size: rgb = 244 bytes (UYVY/RGB888 path), y16 = 164 bytes")
    ap.add_argument("--lines", type=int, default=IMAGE_LINES,
                    help="lines per read (IMAGE_NUM_LINES + g_telemetry_num_lines), default 60")
    ap.add_argument("--no-grid", action="store_true", help="omit the per-line header table")
    ap.add_argument("--selftest", action="store_true", help="run synthetic cases and exit")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.dump:
        ap.error("need a dump file (or --selftest)")
    run_file(a.dump, a.format, a.lines, not a.no_grid)
    return 0


if __name__ == "__main__":
    sys.exit(main())
