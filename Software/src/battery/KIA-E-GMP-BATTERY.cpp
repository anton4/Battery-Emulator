#include "KIA-E-GMP-BATTERY.h"
#include <Arduino.h>
#include "../battery/BATTERIES.h"
#include "../communication/can/comm_can.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/common_functions.h"  //For CRC table
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"
#include "../system_settings.h"

// Function to estimate SOC based on cell voltage
uint16_t KiaEGmpBattery::estimateSOCFromCell(uint16_t cellVoltage) {
  if (cellVoltage >= voltage[0]) {
    return SOC[0];
  }
  if (cellVoltage <= voltage[numPoints - 1]) {
    return SOC[numPoints - 1];
  }

  for (int i = 1; i < numPoints; ++i) {
    if (cellVoltage >= voltage[i]) {
      // Cast to float for proper division
      float t = (float)(cellVoltage - voltage[i]) / (float)(voltage[i - 1] - voltage[i]);

      // Calculate interpolated SOC value
      uint16_t socDiff = SOC[i - 1] - SOC[i];
      uint16_t interpolatedValue = SOC[i] + (uint16_t)(t * socDiff);

      return interpolatedValue;
    }
  }
  return 0;  // Default return for safety, should never reach here
}

// Simplified version of the pack-based SOC estimation with compensation
uint16_t KiaEGmpBattery::estimateSOC(uint16_t packVoltage, uint16_t cellCount, int16_t currentAmps) {
  // If cell count is still the default 192 but we haven't confirmed it yet
  if (!set_voltage_limits && cellCount == 192) {
    // Fall back to BMS-reported SOC while cell count is uncertain
    return (SOC_Display * 10);
  }

  if (cellCount == 0)
    return 0;

  // Convert pack voltage (decivolts) to millivolts
  uint32_t packVoltageMv = packVoltage * 100;

  // Apply internal resistance compensation
  // Current is in deciamps (-150 = -15.0A, 150 = 15.0A)
  // Resistance is in milliohms
  int32_t voltageDrop = (currentAmps * PACK_INTERNAL_RESISTANCE_MOHM) / 10;

  // Compensate the pack voltage (add the voltage drop)
  uint32_t compensatedPackVoltageMv = packVoltageMv + voltageDrop;

  // Calculate average cell voltage in millivolts
  uint16_t avgCellVoltage = compensatedPackVoltageMv / cellCount;

  // Use the cell voltage lookup table to estimate SOC
  return estimateSOCFromCell(avgCellVoltage);
}

// Fix: Change parameter types to uint16_t to match SOC values
uint16_t KiaEGmpBattery::selectSOC(uint16_t SOC_low, uint16_t SOC_high) {
  if (SOC_low == 0 || SOC_high == 0) {
    return 0;  // If either value is 0, return 0
  }
  if (SOC_low == 10000 || SOC_high == 10000) {
    return 10000;  // If either value is 100%, return 100%
  }
  return (SOC_low < SOC_high) ? SOC_low : SOC_high;  // Otherwise, return the lowest value
}

void KiaEGmpBattery::set_cell_voltages(uint8_t reading, uint8_t cellNumber) {
  if ((reading * 20) > 2600) {
    datalayer.battery.status.cell_voltages_mV[cellNumber] = (reading * 20);
  }
}

void KiaEGmpBattery::process_cell_voltage_group(const uint8_t* data, uint8_t baseCell) {
  for (int i = 0; i < 32; i++) {
    set_cell_voltages(data[4 + i], baseCell + i);
  }
}

void KiaEGmpBattery::set_voltage_minmax_limits() {

  uint8_t valid_cell_count = 0;
  for (int i = 0; i < MAX_AMOUNT_CELLS; ++i) {
    if (datalayer.battery.status.cell_voltages_mV[i] > 0) {
      ++valid_cell_count;
    }
  }
  if (valid_cell_count == 144) {
    datalayer.battery.info.number_of_cells = valid_cell_count;
    datalayer.battery.info.max_design_voltage_dV = 6048;
    datalayer.battery.info.min_design_voltage_dV = 4320;
  } else if (valid_cell_count == 180) {
    datalayer.battery.info.number_of_cells = valid_cell_count;
    datalayer.battery.info.max_design_voltage_dV = 7560;
    datalayer.battery.info.min_design_voltage_dV = 5400;
  } else if (valid_cell_count == 192) {
    datalayer.battery.info.number_of_cells = valid_cell_count;
    datalayer.battery.info.max_design_voltage_dV = 8064;
    datalayer.battery.info.min_design_voltage_dV = 5760;
  } else {
    // We are still starting up? Not all cells available.
    set_voltage_limits = false;
  }
}

void KiaEGmpBattery::update_values() {

  if (user_selected_use_estimated_SOC) {
    // Use the simplified pack-based SOC estimation with proper compensation
    datalayer.battery.status.real_soc =
        estimateSOC(batteryVoltage, datalayer.battery.info.number_of_cells, batteryAmps);
  } else {
    datalayer.battery.status.real_soc = (SOC_Display * 10);  //increase SOC range from 0-100.0 -> 100.00
  }

  datalayer.battery.status.soh_pptt = (batterySOH * 10);  //Increase decimals from 100.0% -> 100.00%

  datalayer.battery.status.voltage_dV = batteryVoltage;  //value is *10 (3700 = 370.0)

  datalayer.battery.status.current_dA = -batteryAmps;  //value is *10 (150 = 15.0)

  datalayer.battery.status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer.battery.status.real_soc) / 10000) * datalayer.battery.info.total_capacity_Wh);

  //datalayer.battery.status.max_charge_power_W = (uint16_t)allowedChargePower * 10;  //From kW*100 to Watts
  //The allowed charge power is not available. We use user set value for now
  datalayer.battery.status.max_charge_power_W = datalayer.battery.status.override_charge_power_W;

  //datalayer.battery.status.max_discharge_power_W = (uint16_t)allowedDischargePower * 10;  //From kW*100 to Watts
  //The allowed discharge power is not available. We use user set value for now
  datalayer.battery.status.max_discharge_power_W = datalayer.battery.status.override_discharge_power_W;

  datalayer.battery.status.temperature_min_dC = (int8_t)temperatureMin * 10;  //Increase decimals, 17C -> 17.0C

  datalayer.battery.status.temperature_max_dC = (int8_t)temperatureMax * 10;  //Increase decimals, 18C -> 18.0C

  datalayer.battery.status.cell_max_voltage_mV = CellVoltMax_mV;

  datalayer.battery.status.cell_min_voltage_mV = CellVoltMin_mV;

  if ((millis64() > INTERVAL_60_S) && !set_voltage_limits) {  // millis64: plain millis() wraps after 49.7 days
    set_voltage_limits = true;
    set_voltage_minmax_limits();  // Count cells, and set voltage limits accordingly
  }

  if (waterleakageSensor == 0) {
    set_event(EVENT_WATER_INGRESS, 0);
  }

  if (leadAcidBatteryVoltage < 110) {
    set_event(EVENT_12V_LOW, leadAcidBatteryVoltage);
  }
}

template <typename T>
inline String& operator<<(String& str, const T& value) {
  str += value;
  return str;
}
String KiaEGmpBattery::get_uds_info_html() {
  String content;
  content.reserve(3000);

  // clang-format off
  content << "<h4>Cells: " << String(datalayer.battery.info.number_of_cells) << "</h4>"
              "<h4>SOC (BMS): " << String(SOC_BMS) << "</h4>"
              "<h4>SOC (Display): " << String(SOC_Display) << "</h4>"
              "<h4>12V voltage: " << String(leadAcidBatteryVoltage / 10.0f, 1) << "</h4>"
              "<h4>Waterleakage: " << String(waterleakageSensor)  << "</h4>"
              "<h4>Temperature, water inlet: " << String(temperature_water_inlet)  << "</h4>"
              "<h4>Batterymanagement mode: " << String(batteryManagementMode)  << "</h4>"
              "<h4>Cumulative Charge Energy: " << String(cumulativeChargeEnergy)  << " Wh</h4>"
              "<h4>Cumulative Discharge Energy: " << String(cumulativeDischargeEnergy)  << " Wh</h4>"
              "<h4>Operation Time: " << String(opTime)  << " s</h4>"
              "<h4>BMS ignition: " << String(BMS_ign) << (BMS_ign == 0 ? " (BMS does not see the vehicle as switched on)" : "") << "</h4>"
              "<h4>BMS relay status: 0x" << String(batteryRelay, HEX) << " (" << String(batteryRelay_changes) << " changes";
  if (batteryRelay_changes > 0) {
    content << ", last " << String((millis() - batteryRelay_last_change_ms) / 1000) << " s ago";
  }
  content << ")</h4>"
             "<h4>Inverter-side voltage: " << String(inverterVoltage / 10.0f, 1) << " V (max since boot " << String(inverterVoltage_max / 10.0f, 1) << " V, pack " << String(batteryVoltage / 10.0f, 1) << " V)</h4>"
             "<h4>Relay status and inverter voltage use the Kia/Hyundai 64kWh byte positions, not yet confirmed on E-GMP</h4>"
             "<h4>PID 0101 replies: " << String(pid101_replies) << ", length " << String(pid101_len) << "</h4>"
             "<h4>PID 0101 raw (byte index: value):</h4>";
  // clang-format on
  for (uint16_t i = 0; i < pid101_len; i++) {
    if (i % 8 == 0) {
      content << "<h4 style='font-family:monospace'>" << String(i) << ":";
    }
    content << " " << (pid101_raw[i] < 0x10 ? "0" : "") << String(pid101_raw[i], HEX);
    if (i % 8 == 7 || i == pid101_len - 1) {
      content << "</h4>";
    }
  }
  content << "<h4>PID 0101 bytes that changed since boot (index: changes, seconds since last change):</h4><h4 "
             "style='font-family:monospace'>";
  bool any_change = false;
  for (uint16_t i = 0; i < pid101_len; i++) {
    if (pid101_change_count[i] > 0) {
      any_change = true;
      content << " " << String(i) << ":" << String(pid101_change_count[i]) << ","
              << String((millis() - pid101_last_change_ms[i]) / 1000);
    }
  }
  if (!any_change) {
    content << " none";
  }
  content << "</h4>";
  // clang-format off
  content <<
              "<h4>Vehicle emulation groups (EGMPGROUPS bitmask): " << String(user_selected_egmp_frame_groups) << " (0x" << String(user_selected_egmp_frame_groups, HEX) << ")</h4>"
              "<h4>Emulated vehicle frames: " << String(EGMP_TX_TABLE_SIZE) << " frame types, " << String(emulated_frames_per_second) << " frames/s, " << String(tx_frames_sent) << " sent</h4>"
              "<h4>CAN-FD send failures: " << String(datalayer.system.info.can_2518_send_fail ? "yes" : "no") << "</h4>";
  // clang-format on

  return content;
}

void KiaEGmpBattery::handle_incoming_can_frame(CAN_frame rx_frame) {

  // UDS frames (0x7EC PID/DTC replies) are handled by the superclass.
  if (handle_incoming_uds_can_frame(rx_frame)) {
    return;
  }

  startedUp = true;
  switch (rx_frame.ID) {
    case 0x055:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x150:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x1F5:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x215:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x21A:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x235:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x245:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x25A:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x275:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x2FA:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x325:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x330:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x335:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x360:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x365:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x3BA:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x3F5:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x7EC:
      //Handled in UDS Superclass
      break;
    default:
      suppress_emulated_id(rx_frame.ID);
      break;
  }
}

uint16_t KiaEGmpBattery::handle_pid(uint16_t pid, uint32_t value, const uint8_t* data, uint16_t length) {
  // Called by the UDS superclass for every successful PID response. `value` is
  // the big-endian PID value (up to 4 bytes), `data` points at the raw value
  // bytes (without the SID/DID header). Return 0 to continue the scan list.
  switch (pid) {
    case POLL_GROUP_1:  //59 bytes
      track_pid101(data, length);
      // Frame 10 (ef fb e7)
      //data[0-2] We are not sure what these are.

      // Frame 21 (ef 56 00 00 00 00 00) data3-9
      SOC_BMS = data[4] * 5;                               //56
      allowedChargePower = ((data[5] << 8) + data[6]);     //00 00 (apparently not working)
      allowedDischargePower = ((data[7] << 8) + data[8]);  //00 00 (apparently not working)
      batteryRelay = data[9];  // Relay status bits, same byte the Kia/Hyundai 64kWh BMS reports
      if (batteryRelay != batteryRelay_previous) {
        batteryRelay_last_change_ms = millis();
        batteryRelay_changes++;
        logging.printf("EGMP: BMS relay status 0x%02X -> 0x%02X, inverter side %u.%u V\n", batteryRelay_previous,
                       batteryRelay, inverterVoltage / 10, inverterVoltage % 10);
        batteryRelay_previous = batteryRelay;
      }

      //Frame 22 (00 3c 1a cd 17 16 16) data10-16
      batteryAmps = (data[10] << 8) + data[11];
      batteryVoltage = (data[12] << 8) + data[13];
      temperatureMax = data[14];
      temperatureMin = data[15];
      //temperatureAvg = data[16]; Not required

      // Frame 23 (16 15 15 15 00 7f b3) data17-23
      temperature_water_inlet = data[22];
      CellVoltMax_mV = (data[23] * 20);

      // Frame 24 (b8 b2 37 00 00 77 00) data24-30
      CellVmaxNo = data[24];
      CellVoltMin_mV = (data[25] * 20);
      CellVminNo = data[26];
      leadAcidBatteryVoltage = data[29];
      // Frame 25 (01 ec a6 00 01 e9 af) data31-37
      cumulativeChargeEnergy = data[31] << 16 | data[32] << 8 | data[33];
      cumulativeDischargeEnergy = data[35] << 16 | data[36] << 8 | data[37];

      //Frame 26 (00 01 74 0f 00 01 66) data38-44
      cumulativeChargeEnergy2 = data[39] << 16 | data[40] << 8 | data[41];
      cumulativeDischargeEnergy2 = data[43] << 16 | data[44] << 8 | data[45];  //Flow over

      //Frame 27 (a8 01 03 f3 0f 00 02) data45-51
      opTime = data[46] << 24 | data[47] << 16 | data[48] << 8 | data[49];
      BMS_ign = data[50];
      inverterVoltage = ((data[51] << 8) + data[52]);  //Flow over
      if (inverterVoltage > inverterVoltage_max && inverterVoltage < 10000) {
        inverterVoltage_max = inverterVoltage;
      }
      //Frame 28 (c9 00 00 00 00 0b b8) data52-58
      break;
    case POLL_GROUP_2:  //Cellvoltages (Cell 1-32)
      process_cell_voltage_group(data, 0);
      break;
    case POLL_GROUP_3:  //Cellvoltages (Cell 33-64)
      process_cell_voltage_group(data, 32);
      break;
    case POLL_GROUP_4:  //Cellvoltages (Cell 65-96)
      process_cell_voltage_group(data, 64);
      break;
    case POLL_GROUP_A:  //Cellvoltages (Cell 97-128)
      process_cell_voltage_group(data, 96);
      break;
    case POLL_GROUP_B:  //Cellvoltages (Cell 129-160)
      process_cell_voltage_group(data, 128);
      break;
    case POLL_GROUP_C:  //Cellvoltages (Cell 161-192)
      process_cell_voltage_group(data, 160);
      break;
    case POLL_GROUP_5:
      //Frame 0 (10 2e 62 01 05 ff fb 74) //data0-2
      //Frame21 0f 01 2c 01 01 2c 15 //data3-9
      //Frame22 15 15 15 15 15 15 6c //data10-16
      //Frame23 34 6c 34 00 00 64 1e //data17-23
      heatertemp = data[23];
      //Frame24 00 03 e8 39 38 c6 00 //data24-30
      batterySOH = (data[25] << 8) | data[26];
      //amountOfCells = data[29];
      //Frame25 53 00 00 00 00 00 00 //data31-37
      SOC_Display = data[31] * 5;
      //Frame26 00 15 15 15 16 aa aa //data38-44
      break;
    case POLL_GROUP_6:
      batteryManagementMode = data[14];
      break;
    default:  //Unknown pid
      break;
  }
  return 0;  //Continue scanning the PID list in order
}

void KiaEGmpBattery::track_pid101(const uint8_t* data, uint16_t length) {
  // Remember the raw reply and count per-byte changes. Bytes that are expected to move all the
  // time (current, pack voltage, cumulative counters, operation time) are tracked but not logged.
  if (length > PID101_MAX_LEN) {
    length = PID101_MAX_LEN;
  }
  pid101_replies++;
  for (uint16_t i = 0; i < length; i++) {
    if (pid101_replies > 1 && data[i] != pid101_raw[i]) {
      pid101_change_count[i]++;
      pid101_last_change_ms[i] = millis();
      bool noisy = (i >= 10 && i <= 13) || (i >= 30 && i <= 49);
      if (!noisy) {
        logging.printf("EGMP: PID 0101 byte %u: 0x%02X -> 0x%02X\n", i, pid101_raw[i], data[i]);
      }
    }
    pid101_raw[i] = data[i];
  }
  pid101_len = length;
}

void KiaEGmpBattery::transmit_can(unsigned long currentMillis) {
  if (startedUp) {
    // Emulate the vehicle side of the powertrain bus (VCU, MCU, ICCU, thermal nodes)
    transmit_emulated_frames(currentMillis);

    // UDS PID polling and DTC handling
    transmit_uds_can(currentMillis);
  }
}

void KiaEGmpBattery::suppress_emulated_id(uint16_t can_id) {
  // A frame we emulate arrived from the bus: some real node (or the BMS itself)
  // owns that ID, so stop sending it rather than collide.
  for (uint16_t i = 0; i < EGMP_TX_TABLE_SIZE; i++) {
    if (EGMP_TX_TABLE[i].id == can_id) {
      if (!tx_state[i].suppressed) {
        tx_state[i].suppressed = true;
        logging.printf("EGMP: 0x%03X already present on bus, not emulating it\n", can_id);
      }
      return;
    }
  }
}

void KiaEGmpBattery::transmit_emulated_frames(unsigned long currentMillis) {
  if (!tx_schedule_started) {
    // Stagger the first transmissions so no millisecond carries a burst of frames
    for (uint16_t i = 0; i < EGMP_TX_TABLE_SIZE; i++) {
      tx_state[i].next_due_ms = currentMillis + EGMP_TX_TABLE[i].offset_ms;
      tx_state[i].counter = EGMP_TX_TABLE[i].data[2];
    }
    tx_schedule_started = true;
  }

  uint8_t sent = 0;
  uint16_t index = tx_scan_start;
  for (uint16_t n = 0; n < EGMP_TX_TABLE_SIZE && sent < MAX_TX_FRAMES_PER_TICK; n++) {
    index = (tx_scan_start + n) % EGMP_TX_TABLE_SIZE;
    const EgmpTxFrame& tmpl = EGMP_TX_TABLE[index];
    TxState& state = tx_state[index];
    if (state.suppressed || !(user_selected_egmp_frame_groups & (1u << tmpl.group))) {
      continue;  // Another node sends this ID, or the group is disabled by the user
    }
    if (static_cast<int32_t>(currentMillis - state.next_due_ms) < 0) {
      continue;  // Not due yet
    }

    CAN_frame frame = {};
    frame.FD = !(tmpl.flags & EGMP_TX_CLASSIC);
    frame.ext_ID = false;
    frame.DLC = tmpl.dlc;
    frame.ID = tmpl.id;
    memcpy(frame.data.u8, tmpl.data, tmpl.dlc);
    if (tmpl.flags & EGMP_TX_COUNTER) {
      frame.data.u8[2] = state.counter++;  // 8-bit alive counter, +1 per frame on the wire
    }
    if (tmpl.flags & EGMP_TX_CRC16) {
      uint16_t crc = crc16_hyundai_canfd(frame.data.u8, tmpl.dlc, tmpl.id, tmpl.crc_xor);
      frame.data.u8[0] = static_cast<uint8_t>(crc & 0xFF);
      frame.data.u8[1] = static_cast<uint8_t>(crc >> 8);
    }
    transmit_can_frame(&frame);
    tx_frames_sent++;
    emulated_frames_sent_in_window++;
    sent++;

    state.next_due_ms += tmpl.period_ms;
    if (static_cast<int32_t>(currentMillis - state.next_due_ms) >= static_cast<int32_t>(tmpl.period_ms)) {
      // More than a full period behind (main loop stalled): resynchronise instead of bursting
      // the missed frames, the alive counter must still advance by exactly one per frame.
      state.next_due_ms = currentMillis + tmpl.period_ms;
    }
  }
  if (sent >= MAX_TX_FRAMES_PER_TICK) {
    tx_scan_start = (index + 1) % EGMP_TX_TABLE_SIZE;  // Be fair to the entries we did not reach
  } else {
    tx_scan_start = 0;
  }

  // Frames per second statistic for the advanced battery page
  if (currentMillis - emulated_frames_window_start >= INTERVAL_1_S) {
    emulated_frames_window_start = currentMillis;
    emulated_frames_per_second = emulated_frames_sent_in_window;
    emulated_frames_sent_in_window = 0;
  }
}

void KiaEGmpBattery::setup(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.system.status.battery_allows_contactor_closing = true;
  datalayer.battery.info.number_of_cells = 192;  // TODO: will vary depending on battery
  datalayer.battery.info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer.battery.info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer.battery.info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer.battery.info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer.battery.info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
  // UDS: send requests to 0x7E4, accept replies from the BMS on 0x7EC. Also passing true to isFD
  setup_uds(0x7E4, 0x7EC, true);
  // Group 1 (relay status, inverter-side voltage, ignition) is interleaved so it is
  // read every other poll (~200 ms) and a one-second contactor event is not missed.
  static const uint16_t pid_scan_list[] = {
      POLL_GROUP_1, POLL_GROUP_2, POLL_GROUP_1, POLL_GROUP_3, POLL_GROUP_1, POLL_GROUP_4, POLL_GROUP_1,
      POLL_GROUP_5, POLL_GROUP_1, POLL_GROUP_6, POLL_GROUP_1, POLL_GROUP_7, POLL_GROUP_1, POLL_GROUP_8,
      POLL_GROUP_1, POLL_GROUP_A, POLL_GROUP_1, POLL_GROUP_B, POLL_GROUP_1, POLL_GROUP_C,
  };
  set_pid_scan_list(pid_scan_list, sizeof(pid_scan_list) / sizeof(pid_scan_list[0]));
}
