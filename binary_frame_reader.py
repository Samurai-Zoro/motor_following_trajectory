#!/usr/bin/env python3
"""
Reads 36-byte payload binary frames and writes CSV with:
t_ms,errA_cm,tgtA_cm,actA_cm,pwmA,pwmB,tgtB_deg,actB_deg,errB_deg,degC

Frame format: [MAGIC(2)][LEN(2)][SEQ(4)][PAYLOAD(36)][CRC16(2)] = 46 bytes

Improvements:
- Stats start when the FIRST VALID FRAME arrives (no startup penalty)
- Prints both average rate and last-second instantaneous rate
"""

import argparse, sys, time, struct, collections

try:
    import serial
    from serial.tools import list_ports
except Exception:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    raise

MAGIC = 0xA55A
PAYLOAD_LEN = 36
FRAME_LEN = 2 + 2 + 4 + PAYLOAD_LEN + 2  # 46 bytes

HDR_FMT = "<HHI"  # magic(2), len(2), seq(4)

# Payload: t_ms, errA_um, tgtA_um, actA_um, pwmA, pwmB, tgtB_cdeg, actB_cdeg, errB_cdeg, degC_cdeg
# Types:   u32,  i32,     i32,     i32,     u16,  u16,  i32,       i32,       i32,       i32
SAMPLE_FMT = "<I i i i H H i i i i"

TRL_FMT = "<H"  # CRC16

CSV_HEADER = "t_ms,errA_cm,tgtA_cm,actA_cm,pwmA,pwmB,tgtB_deg,actB_deg,errB_deg,degC"


def guess_port():
    """Try to auto-detect a suitable serial port."""
    try:
        ports = list_ports.comports()
    except Exception:
        return None
    for p in ports:
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower()
        if any(k in desc for k in ["stmicro", "nucleo", "stm"]) or \
           any(k in manu for k in ["stmicro", "stmicroelectronics"]):
            return p.device
    for p in ports:
        if any(tag in p.device for tag in ("ttyACM", "ttyUSB", "COM")):
            return p.device
    return None


def open_serial(port, baud, timeout=1.0):
    """Open serial port with specified settings."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = timeout
    ser.dsrdtr = False
    ser.rtscts = False
    ser.xonxoff = False
    ser.open()
    return ser


def crc16_ccitt(data: bytes) -> int:
    """Calculate CRC16-CCITT (polynomial 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for ch in data:
        crc ^= ch << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_output_path(path):
    """Generate output path with timestamp if not specified."""
    if path:
        return path
    return f"capture_{time.strftime('%Y%m%d_%H%M%S')}.csv"


def main():
    ap = argparse.ArgumentParser(description="Binary → CSV (36B payload, 10 fields).")
    ap.add_argument("--port", help="COM port (e.g., COM3 or /dev/ttyACM0).")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--out", help="CSV output path.")
    ap.add_argument("--duration", type=float, default=None,
                    help="Stop after N seconds (after first frame).")
    ap.add_argument("--stats-every", type=float, default=1.0,
                    help="Print stats every N seconds.")
    ap.add_argument("--flush-every", type=int, default=512)
    ap.add_argument("--skip-banner", action="store_true",
                    help="Skip consuming the one-line ASCII banner from firmware.")
    args = ap.parse_args()

    port = args.port or guess_port()
    if not port:
        print("Could not guess a serial port. Use --port.", file=sys.stderr)
        sys.exit(2)

    ser = open_serial(port, args.baud, timeout=1.0)
    print(f"[INFO] Serial open: {port} @ {args.baud} baud")

    # Consume the single ASCII banner the firmware prints at start
    if not args.skip_banner:
        try:
            banner = ser.readline()
            print(f"[INFO] Banner: {banner.decode('utf-8', errors='ignore').strip()}")
        except Exception:
            pass

    out_path = make_output_path(args.out)
    f = open(out_path, "w", buffering=1, newline="")
    f.write(CSV_HEADER + "\n")
    print(f"[INFO] Writing CSV: {out_path}")

    buf = bytearray()
    rows = 0
    bad_crc = 0
    resyncs = 0

    # Start timing on FIRST GOOD FRAME
    start = None
    last_stats = None

    # For instantaneous rate: timestamps of recent frames (last ~1s)
    recent = collections.deque()

    try:
        while True:
            # If duration is given, stop N seconds AFTER first frame
            if args.duration is not None and start is not None:
                if (time.time() - start) >= args.duration:
                    print("[INFO] Duration reached; stopping.")
                    break

            # Read available data
            chunk = ser.read(1024)
            if chunk:
                buf += chunk

            # Process complete frames
            while len(buf) >= FRAME_LEN:
                # Check for magic bytes
                if struct.unpack_from("<H", buf, 0)[0] != MAGIC:
                    # Search for magic
                    magic_bytes = struct.pack("<H", MAGIC)
                    idx = buf.find(magic_bytes)
                    if idx < 0:
                        buf.clear()
                        resyncs += 1
                        break
                    else:
                        del buf[:idx]
                        resyncs += 1
                        if len(buf) < FRAME_LEN:
                            break

                # Parse header
                _, length, _seq = struct.unpack_from(HDR_FMT, buf, 0)
                if length != PAYLOAD_LEN:
                    # Invalid length, skip one byte and resync
                    del buf[:1]
                    resyncs += 1
                    continue

                if len(buf) < FRAME_LEN:
                    break

                # Extract payload and CRC
                payload = bytes(buf[8:8 + PAYLOAD_LEN])
                (crc_rx,) = struct.unpack_from(TRL_FMT, buf, 8 + PAYLOAD_LEN)

                # Calculate CRC over len + seq + payload (bytes 2..47)
                crc_calc = crc16_ccitt(buf[2:2 + 2 + 4 + PAYLOAD_LEN])

                if crc_rx != crc_calc:
                    bad_crc += 1
                    del buf[:1]
                    continue

                # Parse payload
                (t_ms, errA_um, tgtA_um, actA_um, pwmA, pwmB,
                 tgtB_cdeg, actB_cdeg, errB_cdeg, degC_cdeg) = struct.unpack(SAMPLE_FMT, payload)

                # Convert units
                errA_cm = errA_um / 10000.0
                tgtA_cm = tgtA_um / 10000.0
                actA_cm = actA_um / 10000.0
                tgtB_deg = tgtB_cdeg / 100.0
                actB_deg = actB_cdeg / 100.0
                errB_deg = errB_cdeg / 100.0
                degC = degC_cdeg / 100.0  # Convert centidegrees to degrees

                # Write CSV row
                f.write(f"{t_ms},{errA_cm:.4f},{tgtA_cm:.4f},{actA_cm:.4f},"
                        f"{pwmA},{pwmB},{tgtB_deg:.2f},{actB_deg:.2f},{errB_deg:.2f},{degC:.2f}\n")
                rows += 1

                # Remove processed frame
                del buf[:FRAME_LEN]

                # Timing starts on first good frame
                now = time.time()
                if start is None:
                    start = now
                    last_stats = now

                recent.append(now)
                # Pop anything older than 1.0 s
                while recent and (now - recent[0]) > 1.0:
                    recent.popleft()

            # Periodic stats (only after first frame)
            if start is not None and (time.time() - last_stats) >= args.stats_every:
                now = time.time()
                elapsed = max(1e-6, now - start)
                avg_rate = rows / elapsed
                if len(recent) > 1:
                    inst_rate = len(recent) / max(1e-6, (recent[-1] - recent[0]))
                else:
                    inst_rate = 0.0
                print(f"[STATS] rows={rows}  avg={avg_rate:.1f}/s  "
                      f"inst={inst_rate:.1f}/s  crc_err={bad_crc}  resyncs={resyncs}")
                last_stats = now

    except KeyboardInterrupt:
        print("\n[INFO] Stopping (Ctrl-C).")
    finally:
        try:
            f.flush()
            f.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

    # Final stats
    if start is None:
        print("[DONE] No frames received.")
    else:
        elapsed = max(1e-6, time.time() - start)
        print(f"[DONE] Wrote {rows} rows to {out_path}. "
              f"Avg rate {rows/elapsed:.1f}/s. CRC errors: {bad_crc}. Resyncs: {resyncs}.")


if __name__ == "__main__":
    main()
