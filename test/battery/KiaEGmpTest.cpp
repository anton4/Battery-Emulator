#include <gtest/gtest.h>

#include <map>
#include <vector>

#include "../../Software/src/battery/KIA-E-GMP-BATTERY.h"
#include "../../Software/src/battery/KIA-E-GMP-TX-TABLE.h"
#include "../../Software/src/datalayer/datalayer.h"

#include "Arduino.h"

// TX frame capture injected by the emulated CAN layer (see emul/can.cpp).
void clear_transmitted_frames();
const std::vector<CAN_frame>& get_transmitted_frames();

namespace {

struct LoggedFrame {
  uint16_t id;
  std::vector<uint8_t> data;
  uint16_t crc_xor = 0x6E17;
};

// Frames copied verbatim from real E-GMP battery-bus recordings (EV6 GT M-CAN
// log from upstream issue #387, and the bench capture the driver used before).
// Bytes 0-1 hold the checksum the BMS/VCU/MCU put on the wire.
const LoggedFrame kLoggedFrames[] = {
    // 32-byte, motor controller (bench capture, contactors closed)
    {0x10A, {0x62, 0x36, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x01, 0x00, 0x00, 0x36, 0x39, 0x35, 0x35,
             0xC9, 0x02, 0x00, 0x00, 0x10, 0x00, 0x00, 0x35, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00}},
    // 32-byte, 0x0xx ID range
    {0x035, {0x32, 0x06, 0x7D, 0x51, 0x00, 0x00, 0x25, 0x05, 0x20, 0x00, 0x00, 0x10, 0x00, 0x40, 0x00, 0x00,
             0x04, 0x00, 0x00, 0x00, 0x60, 0x39, 0x95, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x060, {0xD6, 0xD4, 0xD8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x02, 0x40, 0xFF, 0x00, 0xFF,
             0x00, 0x00, 0x05, 0x00, 0x00, 0x08, 0xFA, 0x00, 0x40, 0x00, 0x00, 0x30, 0xFF, 0xFA, 0x00, 0x00}},
    // 32-byte, sent by the BMS itself
    {0x3BA, {0x80, 0x29, 0x73, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    // 24-byte
    {0x0DA, {0x33, 0x3B, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x02}},
    // 16-byte
    {0x125, {0xFC, 0x40, 0x63, 0x44, 0xFF, 0x02, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    // 8-byte CAN-FD frames
    {0x06F, {0x02, 0x6C, 0xD8, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x115, {0xB1, 0x9B, 0xD8, 0x00, 0x00, 0x00, 0x00, 0x00}},
    // 100 ms / 200 ms VCU-side frames (0x2B5 bench capture, 0x308 EV6 GT log)
    {0x2B5, {0xBD, 0xB2, 0x42, 0x00, 0x00, 0x00, 0x00, 0x80, 0x59, 0x00, 0x2B, 0x00, 0x00, 0x04, 0x00, 0x00,
             0xFA, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    // 0x27A: same CRC structure, different final constant (EV6 GT log)
    {0x27A,
     {0x21, 0x31, 0x0A, 0x0F, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x2C, 0x01,
      0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
     0x3302},
    {0x308, {0xA5, 0x80, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0xFF, 0x71, 0x6E, 0x86, 0x0D, 0xFB, 0x8F, 0x03, 0x37, 0xC3, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00}},
};

CAN_frame bms_frame(uint16_t id) {
  CAN_frame frame = {};
  frame.FD = true;
  frame.DLC = 32;
  frame.ID = id;
  return frame;
}

const EgmpTxFrame* table_entry(uint16_t id) {
  for (uint16_t i = 0; i < EGMP_TX_TABLE_SIZE; i++) {
    if (EGMP_TX_TABLE[i].id == id) {
      return &EGMP_TX_TABLE[i];
    }
  }
  return nullptr;
}

// Runs the driver for `duration_ms` in 1 ms steps, the cadence of the core task.
KiaEGmpBattery* run_emulation(uint32_t duration_ms, uint32_t start_ms = 100000) {
  clear_transmitted_frames();
  auto battery = new KiaEGmpBattery();
  battery->setup();
  set_millis64(start_ms);
  battery->handle_incoming_can_frame(bms_frame(0x3BA));  // BMS is alive, emulation may start
  for (uint32_t t = 0; t < duration_ms; t++) {
    set_millis64(start_ms + t);
    battery->transmit_can(static_cast<unsigned long>(start_ms + t));
  }
  return battery;
}

std::map<uint16_t, std::vector<CAN_frame>> frames_by_id() {
  std::map<uint16_t, std::vector<CAN_frame>> out;
  for (const auto& f : get_transmitted_frames()) {
    if (f.ID == 0x7E4) {
      continue;  // UDS requests are not part of the bus emulation
    }
    out[f.ID].push_back(f);
  }
  return out;
}

}  // namespace

TEST(KiaEGmpChecksumTests, ReproducesChecksumsOfRecordedFrames) {
  for (const auto& logged : kLoggedFrames) {
    uint16_t stored = logged.data[0] | (logged.data[1] << 8);
    uint16_t calculated = KiaEGmpBattery::calculate_transmit_checksum(
        logged.id, logged.data.data(), static_cast<uint8_t>(logged.data.size()), logged.crc_xor);
    EXPECT_EQ(calculated, stored) << "ID 0x" << std::hex << logged.id;
  }
}

TEST(KiaEGmpTableTests, TableIsWellFormed) {
  ASSERT_GT(EGMP_TX_TABLE_SIZE, 110);
  for (uint16_t i = 0; i < EGMP_TX_TABLE_SIZE; i++) {
    const auto& e = EGMP_TX_TABLE[i];
    EXPECT_GE(e.dlc, 8) << "0x" << std::hex << e.id;
    EXPECT_LE(e.dlc, 32) << "0x" << std::hex << e.id;
    EXPECT_GE(e.period_ms, 10) << "0x" << std::hex << e.id;
    EXPECT_LT(e.offset_ms, e.period_ms) << "0x" << std::hex << e.id;
    EXPECT_LT(e.group, EGMP_GROUP_COUNT) << "0x" << std::hex << e.id;
    // A counter without a checksum would be sent with a stale CRC.
    EXPECT_TRUE(!(e.flags & EGMP_TX_COUNTER) || (e.flags & EGMP_TX_CRC16)) << "0x" << std::hex << e.id;
    for (uint16_t j = i + 1; j < EGMP_TX_TABLE_SIZE; j++) {
      EXPECT_NE(e.id, EGMP_TX_TABLE[j].id) << "duplicate 0x" << std::hex << e.id;
    }
  }
  // The frames the branch already sent must still be in the table.
  for (uint16_t id : {0x10A, 0x120, 0x19A, 0x2B5, 0x2E0, 0x33A, 0x350, 0x2E5, 0x30A, 0x320, 0x2C0, 0x2D5, 0x2EA, 0x306,
                      0x308, 0x3B5}) {
    EXPECT_NE(table_entry(id), nullptr) << "0x" << std::hex << id;
  }
  // 0x306 carries live data without checksum and must be replayed untouched.
  EXPECT_EQ(table_entry(0x306)->flags, 0);
  // 0x27A has the odd CRC constant; the frozen classic frames are replayed as-is.
  ASSERT_NE(table_entry(0x27A), nullptr);
  EXPECT_EQ(table_entry(0x27A)->crc_xor, 0x3302);
  EXPECT_EQ(table_entry(0x27A)->flags, EGMP_TX_CRC16 | EGMP_TX_COUNTER);
  for (uint16_t id : {0x1CF, 0x3AA, 0x419, 0x4EB, 0x4F0, 0x39B, 0x36F, 0x37F, 0x410}) {
    ASSERT_NE(table_entry(id), nullptr) << "0x" << std::hex << id;
    EXPECT_EQ(table_entry(id)->flags, EGMP_TX_CLASSIC) << "0x" << std::hex << id;
    EXPECT_EQ(table_entry(id)->group, EGMP_GROUP_CLASSIC_FROZEN) << "0x" << std::hex << id;
  }
  // Coolant-inlet temperature patch (see generator): 0x225 byte 3 = 20 C.
  EXPECT_EQ(table_entry(0x225)->data[3], 0x14);
  EXPECT_EQ(table_entry(0x04A)->data[19], 0x33);  // earlier candidate, restored
  EXPECT_EQ(table_entry(0x30A)->data[20], 0x33);  // earlier candidate, restored
}

TEST(KiaEGmpEmulationTests, SendsNothingUntilBmsIsSeen) {
  clear_transmitted_frames();
  auto battery = new KiaEGmpBattery();
  battery->setup();
  for (uint32_t t = 0; t < 500; t++) {
    set_millis64(1000 + t);
    battery->transmit_can(1000 + t);
  }
  EXPECT_TRUE(get_transmitted_frames().empty());
}

TEST(KiaEGmpEmulationTests, EveryTableEntryIsSentAtItsPeriod) {
  const uint32_t duration = 3000;
  run_emulation(duration);
  auto by_id = frames_by_id();
  for (uint16_t i = 0; i < EGMP_TX_TABLE_SIZE; i++) {
    const auto& e = EGMP_TX_TABLE[i];
    auto it = by_id.find(e.id);
    ASSERT_NE(it, by_id.end()) << "0x" << std::hex << e.id << " never sent";
    const auto& frames = it->second;
    double expected = static_cast<double>(duration) / e.period_ms;
    EXPECT_NEAR(frames.size(), expected, expected * 0.1 + 1) << "0x" << std::hex << e.id;
    for (const auto& f : frames) {
      EXPECT_EQ(f.DLC, e.dlc);
      EXPECT_EQ(f.FD, !(e.flags & EGMP_TX_CLASSIC));
      EXPECT_FALSE(f.ext_ID);
    }
  }
  // Nothing outside the table (besides UDS) may be emitted.
  for (const auto& kv : by_id) {
    EXPECT_NE(table_entry(kv.first), nullptr) << "unexpected 0x" << std::hex << kv.first;
  }
}

TEST(KiaEGmpEmulationTests, ChecksumsAndCountersAreValidOnTheWire) {
  run_emulation(2000);
  auto by_id = frames_by_id();
  for (const auto& kv : by_id) {
    const EgmpTxFrame* e = table_entry(kv.first);
    ASSERT_NE(e, nullptr);
    const auto& frames = kv.second;
    for (size_t n = 0; n < frames.size(); n++) {
      const auto& f = frames[n];
      if (e->flags & EGMP_TX_CRC16) {
        uint16_t stored = f.data.u8[0] | (f.data.u8[1] << 8);
        EXPECT_EQ(stored, KiaEGmpBattery::calculate_transmit_checksum(f.ID, f.data.u8, f.DLC, e->crc_xor))
            << "0x" << std::hex << f.ID;
      } else {
        // No checksum: bytes 0-1 are payload and must be replayed verbatim.
        EXPECT_EQ(f.data.u8[0], e->data[0]);
        EXPECT_EQ(f.data.u8[1], e->data[1]);
      }
      if (e->flags & EGMP_TX_COUNTER) {
        if (n > 0) {
          EXPECT_EQ(static_cast<uint8_t>(f.data.u8[2] - frames[n - 1].data.u8[2]), 1)
              << "0x" << std::hex << f.ID << " frame " << std::dec << n;
        }
      } else {
        EXPECT_EQ(f.data.u8[2], e->data[2]);
      }
      // Everything after the counter is the template.
      for (uint8_t b = 3; b < f.DLC; b++) {
        EXPECT_EQ(f.data.u8[b], e->data[b]) << "0x" << std::hex << f.ID << " byte " << std::dec << b;
      }
    }
  }
}

TEST(KiaEGmpEmulationTests, NeverBurstsMoreThanTheCapPerTick) {
  clear_transmitted_frames();
  auto battery = new KiaEGmpBattery();
  battery->setup();
  set_millis64(5000);
  battery->handle_incoming_can_frame(bms_frame(0x3BA));
  battery->transmit_can(5000);
  size_t before = get_transmitted_frames().size();
  // Main loop stalls for 300 ms: everything is overdue at once.
  set_millis64(5300);
  battery->transmit_can(5300);
  size_t sent = get_transmitted_frames().size() - before;
  EXPECT_LE(sent, KiaEGmpBattery::MAX_TX_FRAMES_PER_TICK + 2);  // + possible UDS frames
  EXPECT_GT(sent, 0u);
}

TEST(KiaEGmpEmulationTests, CountersStayContinuousAcrossAStall) {
  clear_transmitted_frames();
  auto battery = new KiaEGmpBattery();
  battery->setup();
  set_millis64(7000);
  battery->handle_incoming_can_frame(bms_frame(0x3BA));
  uint32_t t = 7000;
  for (; t < 7200; t++) {
    set_millis64(t);
    battery->transmit_can(t);
  }
  t += 500;  // stall
  for (; t < 8200; t++) {
    set_millis64(t);
    battery->transmit_can(t);
  }
  auto by_id = frames_by_id();
  const auto& frames = by_id[0x10A];
  ASSERT_GT(frames.size(), 50u);
  for (size_t n = 1; n < frames.size(); n++) {
    EXPECT_EQ(static_cast<uint8_t>(frames[n].data.u8[2] - frames[n - 1].data.u8[2]), 1) << "frame " << n;
  }
  // After the stall the 10 ms frame must not be sent 50 times in a burst.
  EXPECT_LT(frames.size(), 200u / 10 + 1000u / 10 + 5u);
}

TEST(KiaEGmpEmulationTests, StopsEmulatingAnIdAnotherNodeSends) {
  auto battery = run_emulation(500);
  battery->handle_incoming_can_frame(bms_frame(0x2B5));  // someone else owns 0x2B5
  clear_transmitted_frames();
  for (uint32_t t = 100500; t < 101500; t++) {
    set_millis64(t);
    battery->transmit_can(t);
  }
  auto by_id = frames_by_id();
  EXPECT_EQ(by_id.count(0x2B5), 0u);
  EXPECT_GT(by_id.count(0x2E0), 0u);  // its neighbours keep going
}
