"""Decode a VCU raw CAN log (CANInnnn.BIN) into CSV.

Reads: CANInnnn.BIN copied from the SD card.
Writes: <input without extension>_can.csv next to the input.
Run: python decode_cani.py CANI0012.BIN   ->   writes CANI0012_can.csv
(any other argument count prints a usage line and exits)

the BIN files are written by VCU_BERT26_TTT-D1_2.ino 

The file starts with a 22 byte header, then one 18 byte record per CAN frame.


Header:
  bytes 0-7    "VCUCANI1"
  bytes 8-11   session number
  bytes 12-13  record size (18)
  bytes 14-17  lowest CAN ID saved
  bytes 18-21  highest CAN ID saved

Record:
  bytes 0-3    time (ms since power-on)
  bytes 4-7    CAN ID
  byte  8      data length
  byte  9      flags
  bytes 10-17  data

Flags:
  0x01  sent by the VCU (else received)
  0x02  29-bit CAN ID
  0x04  lost-frames note, not a real frame

Lost-frames note data (running totals since power-on):
  bytes 0-3  received frames lost
  bytes 4-7  torque commands lost
"""
import csv
import struct
import sys

MAGIC = b"VCUCANI1"
HEADER_STRUCT = struct.Struct("<8sIHII")   # magic, session, record size, id min, id max
RECORD_STRUCT = struct.Struct("<IIBB8s")   # time ms, CAN id, length, flags, data[8]
DROPS_STRUCT = struct.Struct("<II")        # lost-frames note payload (rx, tx lost)
FLAG_TX, FLAG_EXT, FLAG_DROPS = 0x01, 0x02, 0x04
MS_PER_S = 1000


def decode_can_log(path):
    with open(path, "rb") as f:
        data = f.read()

    magic, session, record_size, id_min, id_max = HEADER_STRUCT.unpack_from(data, 0)
    if magic != MAGIC or record_size != RECORD_STRUCT.size:
        sys.exit(f"{path}: not a CANI v1 file (magic={magic!r}, recordSize={record_size})")

    rows = [RECORD_STRUCT.unpack_from(data, off)
            for off in range(HEADER_STRUCT.size,
                             len(data) - RECORD_STRUCT.size + 1,
                             RECORD_STRUCT.size)]
    rows.sort(key=lambda r: r[0])

    out_path = path.rsplit(".", 1)[0] + "_can.csv"
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "dir", "id", "ext", "len", "data", "rx_drops", "tx_drops"])
        for time_ms, can_id, length, flags, payload in rows:
            if flags & FLAG_DROPS:
                rx_drops, tx_drops = DROPS_STRUCT.unpack(payload)
                w.writerow([time_ms / MS_PER_S, "DROP", "", "", "", "", rx_drops, tx_drops])
                continue
            w.writerow([time_ms / MS_PER_S,
                        "TX" if flags & FLAG_TX else "RX",
                        f"0x{can_id:03X}",
                        int(bool(flags & FLAG_EXT)),
                        length,
                        payload[:length].hex(" ").upper(),
                        "", ""])

    print(f"session {session}: {len(rows)} records, IDs 0x{id_min:X}-0x{id_max:X} -> {out_path}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python decode_cani.py CANInnnn.BIN")
    decode_can_log(sys.argv[1])
