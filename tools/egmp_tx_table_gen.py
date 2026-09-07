#!/usr/bin/env python3
"""Generate Software/src/battery/KIA-E-GMP-TX-TABLE.h from a CAN-FD log of the
E-GMP battery bus (M-CAN) recorded in a running car.

Source log used for the checked-in table: "m-can cangoo.asc" from
Kia_EV6GT_G-CAN_M-CAN.zip attached to
https://github.com/dalathegreat/Battery-Emulator/issues/387 (Kia EV6 GT, 22 s,
Vector .asc format).

Usage: python3 tools/egmp_tx_table_gen.py "m-can cangoo.asc" > Software/src/battery/KIA-E-GMP-TX-TABLE.h

Every ID on the bus that is not transmitted by the BMS becomes one table entry
holding its most common payload, its period, a stagger offset and flags saying
whether the driver must regenerate the CRC-16 (bytes 0-1) and the alive counter
(byte 2) each transmission. The BMS accepts frames only when both are right, so
the checked-in analysis of the log is reproduced here:

  * CRC: CRC-16/CCITT (poly 0x1021, init 0xFFFF) over data[2..DLC-1], then the
    ID low byte, then the ID high byte, XORed with 0x6E17, stored little-endian
    in bytes 0..1. Verified on 100 % of the CAN-FD frames in the log.
    0x27A uses the same structure with final XOR 0x3302; the per-entry crc_xor
       field carries the constant.
  * Counter: byte 2 increments by one per transmission and wraps at 0xFF.
"""
import collections
import re
import statistics
import sys

# IDs transmitted by the BMS itself (from a bench capture with only the BMS on
# the bus). These are received, never emulated.
BMS_IDS = {0x055, 0x150, 0x1F5, 0x215, 0x21A, 0x235, 0x245, 0x25A, 0x275, 0x2FA, 0x325, 0x330, 0x335, 0x360,
           0x365, 0x3BA, 0x3F5}

# Classic 8-byte frames whose payload changes and whose checksum is not the
# E-GMP CRC-16 (gateway-forwarded chassis/body traffic). They cannot be
# regenerated, so one recorded frame is replayed verbatim with a frozen counter,
# in their own group so they can be masked off.
FROZEN_CLASSIC_IDS = {0x1CF, 0x3AA, 0x419, 0x4EB, 0x4F0, 0x39B, 0x36F, 0x37F, 0x410}

# IDs to leave out entirely.
SKIP_IDS = set()

# Byte patches applied to the chosen payload. The BMS reports the coolant inlet
# temperature it receives over CAN; the recorded car was warm (0x33 = 51 C).
# Candidates carrying 0x33: 0x30A byte 20, 0x225 byte 3, 0x04A byte 19.
# 0x30A byte 20 is patched to 0x14 (20 C) first; move on if the BMS still shows 51 C.
PAYLOAD_PATCHES = {0x30A: {20: 0x14}}

# Frames without any checksum (bytes 0-1 are data). Replayed verbatim.
NO_CRC_IDS = {0x306}

# ECU groups, from clustering frames that are transmitted in the same time slot
# in the log. Used as a bisecting aid (see enabled_groups in the driver). Any
# ID not listed falls into a group by period.
GROUPS = [
    ("MCU_REAR_10MS", [0x04A, 0x10A, 0x19A]),
    ("MCU_FRONT_10MS", [0x060, 0x065, 0x06F, 0x0A0, 0x0B0, 0x115, 0x120]),
    ("CHASSIS_10MS", [0x035, 0x0DA, 0x0F5, 0x125, 0x130]),
    ("GROUP_20MS_A", [0x145, 0x16A, 0x1B0]),
    ("GROUP_20MS_B", [0x180, 0x185, 0x1AA, 0x175, 0x1A0]),
    ("GROUP_50MS", [0x1BA, 0x1E5, 0x1F0]),
    ("VCU_100MS", [0x2AA, 0x2B5, 0x2E0]),
    ("GROUP_100MS_B", [0x255, 0x2E5]),
    ("GROUP_100MS_C", [0x30A, 0x320]),
    ("GROUP_100MS_D", [0x33A, 0x350]),
    ("GROUP_100MS_E", [0x205, 0x385, 0x090, 0x1FA, 0x20A, 0x225, 0x250]),
    ("GROUP_200MS_A", [0x306, 0x308, 0x395]),
    ("GROUP_200MS_B", [0x2D5, 0x2EA]),
    ("GROUP_200MS_C", [0x2C0, 0x3B5, 0x315, 0x355, 0x3A0]),
    ("GROUP_1S", [0x305, 0x480, 0x3F0, 0x1F1, 0x1F2, 0x1F3, 0x1F4, 0x1F6, 0x1F7, 0x1F8, 0x1F9, 0x1FB, 0x1FC, 0x1FD,
                  0x1FE, 0x201, 0x202]),
    ("CLASSIC_GATEWAY", []),  # every remaining static classic 8-byte frame
    ("CLASSIC_FROZEN", []),  # FROZEN_CLASSIC_IDS
]

# Payloads proven to close contactors (Battery-Emulator commit 6204456a, bench
# capture from a car with HV active). The GT log above was recorded in a
# different vehicle state, so for the IDs the driver already sent we keep the
# proven payload and only regenerate CRC and counter. 0x308 is deliberately not
# in this list: the bench frame fails the CRC and its bytes 15-27 are shifted by
# one position against every sample in the log (a transcription error). Run with --payloads-from-log
# to take every payload from the log instead.
PROVEN_PAYLOADS = {
    0x10A: (0x62, 0x36, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x01, 0x00, 0x00, 0x36, 0x39, 0x35, 0x35, 0xC9, 0x02, 0x00, 0x00, 0x10, 0x00, 0x00, 0x35, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x120: (0xD4, 0x1B, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x01, 0x00, 0x00, 0x37, 0x35, 0x37, 0x37, 0xC9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x19A: (0x24, 0x9B, 0x7B, 0x55, 0x44, 0x64, 0xD8, 0x1B, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x11, 0x52, 0x00, 0x12, 0x02, 0x64, 0x00, 0x00, 0x00, 0x08, 0x13, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00),
    0x2B5: (0xBD, 0xB2, 0x42, 0x00, 0x00, 0x00, 0x00, 0x80, 0x59, 0x00, 0x2B, 0x00, 0x00, 0x04, 0x00, 0x00, 0xFA, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x2C0: (0xCC, 0xCD, 0xA2, 0x21, 0x00, 0xA1, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x2D5: (0x79, 0xFB, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x2E0: (0xC1, 0xF2, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x70, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x2E5: (0x69, 0x8A, 0x3F, 0x01, 0x00, 0x00, 0x00, 0x15, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x2EA: (0x6E, 0xBB, 0xA0, 0x0D, 0x04, 0x01, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC7, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x306: (0x00, 0x00, 0x00, 0xD2, 0x06, 0x92, 0x05, 0x34, 0x07, 0x8E, 0x08, 0x73, 0x05, 0x80, 0x05, 0x83, 0x05, 0x73, 0x05, 0x80, 0x05, 0xED, 0x01, 0xDD, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x30A: (0xB1, 0xE0, 0x26, 0x08, 0x54, 0x01, 0x04, 0x15, 0x00, 0x1A, 0x76, 0x00, 0x25, 0x01, 0x10, 0x27, 0x4F, 0x06, 0x18, 0x04, 0x33, 0x15, 0x34, 0x28, 0x00, 0x00, 0x10, 0x06, 0x21, 0x00, 0x4B, 0x06),
    0x320: (0xC6, 0xAB, 0x26, 0x41, 0x00, 0x00, 0x01, 0x3C, 0xAC, 0x0D, 0x40, 0x20, 0x05, 0xC8, 0xA0, 0x03, 0x40, 0x20, 0x2B, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x33A: (0x1A, 0x23, 0x26, 0x10, 0x27, 0x4F, 0x06, 0x00, 0xF8, 0x1B, 0x19, 0x04, 0x30, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x2E, 0x2D, 0x81, 0x25, 0x20, 0x00, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x350: (0x26, 0x82, 0x26, 0xF4, 0x01, 0x00, 0x00, 0x50, 0x90, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    0x3B5: (0xA3, 0xC8, 0x9F, 0x00, 0x00, 0x00, 0x00, 0x36, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC7, 0x02, 0x00, 0x00, 0x00, 0x00, 0x6A, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
}

PERIODS = [10, 20, 50, 100, 200, 250, 500, 1000, 1500, 2000, 5000, 8000]

FLAG_CRC16 = 1
FLAG_COUNTER = 2
FLAG_CLASSIC = 4


def crc16(data, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def egmp_crc(can_id, data):
    return crc16(list(data[2:]) + [can_id & 0xFF, can_id >> 8]) ^ 0x6E17


def parse_asc(path):
    rx = re.compile(r'^\s*([\d.]+)\s+1\s+([0-9a-fA-F]+)x?\s+(?:Rx|Tx)\s+d\s+(\d+)\s+((?:[0-9A-F]{2}\s+)*)')
    by_id = collections.defaultdict(list)
    for line in open(path, errors='ignore'):
        m = rx.match(line)
        if not m or int(m.group(3)) == 0:
            continue
        by_id[int(m.group(2), 16)].append((float(m.group(1)), tuple(int(x, 16) for x in m.group(4).split())))
    return by_id


def nearest_period(ms):
    return min(PERIODS, key=lambda p: abs(p - ms))


def main():
    by_id = parse_asc(sys.argv[1])
    group_of = {}
    for gi, (_, ids) in enumerate(GROUPS):
        for i in ids:
            group_of[i] = gi
    classic_group = len(GROUPS) - 2
    frozen_group = len(GROUPS) - 1

    entries = []
    for can_id, frames in sorted(by_id.items()):
        if can_id in BMS_IDS or can_id in SKIP_IDS or len(frames) < 3:
            continue
        ts = [t for t, _ in frames]
        period = nearest_period(statistics.median(b - a for a, b in zip(ts, ts[1:])) * 1000)
        payload = collections.Counter(d for _, d in frames).most_common(1)[0][0]
        if can_id in PROVEN_PAYLOADS and '--payloads-from-log' not in sys.argv[2:]:
            payload = PROVEN_PAYLOADS[can_id]
        dlc = len(payload)
        crc_ok = all((d[0] | (d[1] << 8)) == egmp_crc(can_id, d) for _, d in frames)
        crc_xor = 0x6E17
        if not crc_ok and can_id not in NO_CRC_IDS:
            # Same CRC structure with another final constant? (0x27A uses 0x3302.)
            xors = {(d[0] | (d[1] << 8)) ^ egmp_crc(can_id, d) ^ 0x6E17 for _, d in frames}
            if len(xors) == 1 and len(frames) > 3:
                crc_xor = xors.pop()
                crc_ok = True
        deltas = collections.Counter((b[1][2] - a[1][2]) & 0xFF for a, b in zip(frames, frames[1:]))
        total = sum(deltas.values())
        counter = (deltas[1] + deltas[2]) >= 0.9 * total and deltas[1] > 0.5 * total
        flags = 0
        if can_id in NO_CRC_IDS:
            pass
        elif can_id in FROZEN_CLASSIC_IDS:
            flags |= FLAG_CLASSIC
        elif crc_ok:
            flags |= FLAG_CRC16
            if counter:
                flags |= FLAG_COUNTER
        else:
            # Not the E-GMP CRC: only static frames can be replayed verbatim.
            if len(set(d for _, d in frames)) != 1 or dlc != 8:
                print(f"// skipping 0x{can_id:03X}: non-generic checksum with changing payload", file=sys.stderr)
                continue
            flags |= FLAG_CLASSIC
        payload = list(payload)
        for index, value in PAYLOAD_PATCHES.get(can_id, {}).items():
            payload[index] = value
        payload = tuple(payload)
        if can_id in FROZEN_CLASSIC_IDS:
            group = frozen_group
        else:
            group = group_of.get(can_id, classic_group if flags & FLAG_CLASSIC else None)
        if group is None:
            # unlisted CAN-FD frame: group by period
            group = {10: 2, 20: 4, 50: 5, 100: 10, 200: 13}.get(period, 14)
        entries.append(dict(id=can_id, dlc=dlc, period=period, flags=flags, group=group, crc_xor=crc_xor,
                            data=payload, n=len(frames)))

    # Stagger: round-robin offsets inside each period class so that no
    # millisecond carries more than a handful of frames.
    for period in sorted(set(e['period'] for e in entries)):
        cls = [e for e in entries if e['period'] == period]
        for i, e in enumerate(cls):
            e['offset'] = (i * max(1, period // max(1, len(cls)))) % period

    out = sys.stdout
    out.write("// Generated by tools/egmp_tx_table_gen.py - do not edit by hand.\n")
    out.write("// Source: Kia EV6 GT M-CAN log (issue #387), every non-BMS ID on the battery bus.\n")
    out.write("// clang-format off\n#ifndef KIA_E_GMP_TX_TABLE_H\n#define KIA_E_GMP_TX_TABLE_H\n#include <stdint.h>\n\n")
    out.write("// Flags\nstatic const uint8_t EGMP_TX_CRC16 = 1;    // regenerate CRC-16 in bytes 0-1\n")
    out.write("static const uint8_t EGMP_TX_COUNTER = 2;  // 8-bit alive counter in byte 2\n")
    out.write("static const uint8_t EGMP_TX_CLASSIC = 4;  // classic CAN frame (FD flag off)\n\n")
    out.write("// ECU groups (bit index into KiaEGmpBattery::enabled_groups)\nenum EgmpTxGroup : uint8_t {\n")
    for gi, (name, _) in enumerate(GROUPS):
        out.write(f"  EGMP_GROUP_{name} = {gi},\n")
    out.write("  EGMP_GROUP_COUNT\n};\n\n")
    out.write("struct EgmpTxFrame {\n  uint16_t id;\n  uint8_t dlc;\n  uint16_t period_ms;\n  uint16_t offset_ms;\n"
              "  uint8_t flags;\n  uint8_t group;\n  uint16_t crc_xor;  // final XOR of the CRC-16 (0x6E17 for almost every ID)\n"
              "  uint8_t data[32];\n};\n\n")
    out.write(f"static const uint16_t EGMP_TX_TABLE_SIZE = {len(entries)};\n")
    out.write("static const EgmpTxFrame EGMP_TX_TABLE[EGMP_TX_TABLE_SIZE] = {\n")
    for e in entries:
        data = ', '.join(f"0x{b:02X}" for b in e['data'])
        flagnames = ' | '.join(n for f, n in ((FLAG_CRC16, 'EGMP_TX_CRC16'), (FLAG_COUNTER, 'EGMP_TX_COUNTER'),
                                              (FLAG_CLASSIC, 'EGMP_TX_CLASSIC')) if e['flags'] & f) or '0'
        out.write(f"    // 0x{e['id']:03X}: {e['n']} frames in log, group {GROUPS[e['group']][0]}\n")
        out.write(f"    {{0x{e['id']:03X}, {e['dlc']}, {e['period']}, {e['offset']}, {flagnames}, "
                  f"EGMP_GROUP_{GROUPS[e['group']][0]}, 0x{e['crc_xor']:04X},\n     {{{data}}}}},\n")
    out.write("};\n\n#endif\n// clang-format on\n")
    print(f"// {len(entries)} entries written", file=sys.stderr)


if __name__ == '__main__':
    main()
