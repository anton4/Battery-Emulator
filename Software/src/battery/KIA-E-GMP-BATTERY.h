#ifndef KIA_E_GMP_BATTERY_H
#define KIA_E_GMP_BATTERY_H
#include "KIA-E-GMP-TX-TABLE.h"
#include "UdsCanBattery.h"

extern bool user_selected_use_estimated_SOC;
extern uint16_t user_selected_egmp_frame_groups;

class KiaEGmpBattery : public UdsCanBattery {
 public:
  bool mandatory_charge_taper() { return true; }
  // Use the default constructor to create the first or single battery.
  KiaEGmpBattery() : UdsCanBattery() {
    datalayer_battery = &datalayer.battery;
    dtc = &datalayer_battery->dtc;
  }

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "Kia/Hyundai EGMP platform";

  String get_uds_info_html() override;
  const char* get_dtc_json_filename() override { return "kia_egmp_dtc.json"; }

  /* Upper bound on emulated frames per transmit_can() call, so a stalled main
   * loop cannot dump the whole table into the 32-frame CAN-FD TX FIFO at once. */
  static constexpr uint8_t MAX_TX_FRAMES_PER_TICK = 12;
  /* Bitmask with every EgmpTxGroup enabled (default of the EGMPGROUPS setting). */
  static constexpr uint16_t ALL_FRAME_GROUPS = (1u << EGMP_GROUP_COUNT) - 1;

 protected:
  // Called by the UDS superclass for each successful PID query response.
  uint16_t handle_pid(uint16_t pid, uint32_t value, const uint8_t* data, uint16_t length) override;

 private:
  DATALAYER_BATTERY_TYPE* datalayer_battery;

  uint16_t estimateSOC(uint16_t packVoltage, uint16_t cellCount, int16_t currentAmps);
  uint16_t selectSOC(uint16_t SOC_low, uint16_t SOC_high);
  uint16_t estimateSOCFromCell(uint16_t cellVoltage);
  void set_cell_voltages(uint8_t reading, uint8_t cellNumber);
  void process_cell_voltage_group(const uint8_t* data, uint8_t baseCell);
  void set_voltage_minmax_limits();
  void transmit_emulated_frames(unsigned long currentMillis);
  void suppress_emulated_id(uint16_t can_id);

  static const int MAX_PACK_VOLTAGE_DV = 8064;  //5000 = 500.0V
  static const int MIN_PACK_VOLTAGE_DV = 4320;
  static const int MAX_CELL_DEVIATION_MV = 150;
  static const int MAX_CELL_VOLTAGE_MV = 4250;  //Battery is put into emergency stop if one cell goes over this value
  static const int MIN_CELL_VOLTAGE_MV = 2950;  //Battery is put into emergency stop if one cell goes below this value

  // Used for SoC compensation - Define internal resistance value in milliohms for the entire pack
  // How to calculate: voltage_drop_under_known_load [Volts] / load [Amps] = Resistance
  static const int PACK_INTERNAL_RESISTANCE_MOHM = 200;  // 200 milliohms for the whole pack

  uint32_t opTime = 0;
  uint32_t cumulativeChargeEnergy = 0;
  uint32_t cumulativeDischargeEnergy = 0;
  uint32_t cumulativeChargeEnergy2 = 0;
  uint32_t cumulativeDischargeEnergy2 = 0;
  uint16_t inverterVoltage = 0;
  uint16_t soc_calculated = 500;
  uint16_t SOC_BMS = 500;
  uint16_t SOC_Display = 500;
  uint16_t batterySOH = 1000;
  uint16_t CellVoltMax_mV = 3700;
  uint16_t CellVoltMin_mV = 3700;
  uint16_t batteryVoltage = 6700;
  int16_t leadAcidBatteryVoltage = 120;
  int16_t batteryAmps = 0;
  int16_t temperatureMax = 20;
  int16_t temperatureMin = 20;
  int16_t allowedDischargePower = 0;
  int16_t allowedChargePower = 0;
  uint8_t CellVmaxNo = 0;
  uint8_t CellVminNo = 0;
  uint8_t batteryManagementMode = 0;
  uint8_t BMS_ign = 0xff;
  uint8_t batteryRelay = 0;  // PID 0x0101 relay status byte, changes when the BMS closes/opens
  uint8_t batteryRelay_previous = 0;
  unsigned long batteryRelay_last_change_ms = 0;
  uint8_t batteryRelay_changes = 0;
  uint16_t inverterVoltage_max = 0;  // highest inverter-side voltage seen since boot (0.1 V)
  uint8_t waterleakageSensor = 164;
  bool startedUp = false;
  int8_t temperature_water_inlet = 20;
  int8_t heatertemp = 20;
  bool set_voltage_limits = false;

  // Define the data points for %SOC depending on cell voltage
  const uint8_t numPoints = 100;

  const uint16_t SOC[101] = {10000, 9900, 9800, 9700, 9600, 9500, 9400, 9300, 9200, 9100, 9000, 8900, 8800, 8700, 8600,
                             8500,  8400, 8300, 8200, 8100, 8000, 7900, 7800, 7700, 7600, 7500, 7400, 7300, 7200, 7100,
                             7000,  6900, 6800, 6700, 6600, 6500, 6400, 6300, 6200, 6100, 6000, 5900, 5800, 5700, 5600,
                             5500,  5400, 5300, 5200, 5100, 5000, 4900, 4800, 4700, 4600, 4500, 4400, 4300, 4200, 4100,
                             4000,  3900, 3800, 3700, 3600, 3500, 3400, 3300, 3200, 3100, 3000, 2900, 2800, 2700, 2600,
                             2500,  2400, 2300, 2200, 2100, 2000, 1900, 1800, 1700, 1600, 1500, 1400, 1300, 1200, 1100,
                             1000,  900,  800,  700,  600,  500,  400,  300,  200,  100,  0};

  const uint16_t voltage[101] = {
      4200, 4173, 4148, 4124, 4102, 4080, 4060, 4041, 4023, 4007, 3993, 3980, 3969, 3959, 3953, 3950, 3941,
      3932, 3924, 3915, 3907, 3898, 3890, 3881, 3872, 3864, 3855, 3847, 3838, 3830, 3821, 3812, 3804, 3795,
      3787, 3778, 3770, 3761, 3752, 3744, 3735, 3727, 3718, 3710, 3701, 3692, 3684, 3675, 3667, 3658, 3650,
      3641, 3632, 3624, 3615, 3607, 3598, 3590, 3581, 3572, 3564, 3555, 3547, 3538, 3530, 3521, 3512, 3504,
      3495, 3487, 3478, 3470, 3461, 3452, 3444, 3435, 3427, 3418, 3410, 3401, 3392, 3384, 3375, 3367, 3358,
      3350, 3338, 3325, 3313, 3299, 3285, 3271, 3255, 3239, 3221, 3202, 3180, 3156, 3127, 3090, 3000};
  // ---- Vehicle emulation --------------------------------------------------------------------------
  // The BMS shares the E-GMP powertrain CAN-FD bus with the VCU, the motor inverters (MCU), the ICCU
  // (OBC + DC/DC), the e-compressor, coolant valves and more, and only keeps its contactors closed
  // while it hears them with valid CRCs (bytes 0-1, see crc16_hyundai_canfd()) and alive counters
  // (byte 2). Every non-BMS frame recorded on a real EV6 GT is replayed from KIA-E-GMP-TX-TABLE.h.
  // Frames are organised in ECU groups that are enabled through the EGMPGROUPS setting (bitmask),
  // so the set the BMS really needs can be bisected on a real pack. Only the mutable state lives
  // here, the templates stay in flash.
  struct TxState {
    unsigned long next_due_ms;
    uint8_t counter;
    bool suppressed;  // another node on the bus already sends this ID
  };
  TxState tx_state[EGMP_TX_TABLE_SIZE] = {};
  bool tx_schedule_started = false;
  uint16_t tx_scan_start = 0;  // round-robin start index when the per-tick cap is hit
  uint32_t tx_frames_sent = 0;

  uint16_t emulated_frames_per_second = 0;
  uint16_t emulated_frames_sent_in_window = 0;
  unsigned long emulated_frames_window_start = 0;

  static const int POLL_GROUP_1 = 0x0101;
  static const int POLL_GROUP_2 = 0x0102;  //Cellvoltages 1-32
  static const int POLL_GROUP_3 = 0x0103;  //Cellvoltages 33-64
  static const int POLL_GROUP_4 = 0x0104;  //Cellvoltages 65-96
  static const int POLL_GROUP_5 = 0x0105;
  static const int POLL_GROUP_6 = 0x0106;
  static const int POLL_GROUP_7 = 0x0107;
  static const int POLL_GROUP_8 = 0x0108;
  //static const int POLL_GROUP_9 = 0x0109; Does not exist
  static const int POLL_GROUP_A = 0x010A;  //Cellvoltages 97-128
  static const int POLL_GROUP_B = 0x010B;  //Cellvoltages 129-160
  static const int POLL_GROUP_C = 0x010C;  //Cellvoltages 161-192
  //static const int POLL_GROUP_D = 0x010D; Does not exist
};

#endif
