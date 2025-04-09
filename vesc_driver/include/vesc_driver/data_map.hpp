/*********************************************************************
 * Copyright (c) 2019, SoftBank corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#ifndef VESC_DRIVER_DATA_MAP_HPP_
#define VESC_DRIVER_DATA_MAP_HPP_
#include <cstdint>

/**
 * @brief ID of communication commands
 **/
enum COMM_PACKET_ID
{
  COMM_FW_VERSION = 0,
  COMM_JUMP_TO_BOOTLOADER,
  COMM_ERASE_NEW_APP,
  COMM_WRITE_NEW_APP_DATA,
  COMM_GET_VALUES,
  COMM_SET_DUTY,
  COMM_SET_CURRENT,
  COMM_SET_CURRENT_BRAKE,
  COMM_SET_ERPM,
  COMM_SET_POS,
  COMM_SET_HANDBRAKE,
  COMM_SET_DETECT,
  COMM_SET_SERVO_POS,
  COMM_SET_MCCONF,
  COMM_GET_MCCONF,
  COMM_GET_MCCONF_DEFAULT,
  COMM_SET_APPCONF,
  COMM_GET_APPCONF,
  COMM_GET_APPCONF_DEFAULT,
  COMM_SAMPLE_PRINT,
  COMM_TERMINAL_CMD,
  COMM_PRINT,
  COMM_ROTOR_POSITION,
  COMM_EXPERIMENT_SAMPLE,
  COMM_DETECT_MOTOR_PARAM,
  COMM_DETECT_MOTOR_R_L,
  COMM_DETECT_MOTOR_FLUX_LINKAGE,
  COMM_DETECT_ENCODER,
  COMM_DETECT_HALL_FOC,
  COMM_REBOOT,
  COMM_ALIVE,
  COMM_GET_DECODED_PPM,
  COMM_GET_DECODED_ADC,
  COMM_GET_DECODED_CHUK,
  COMM_FORWARD_CAN,
  COMM_SET_CHUCK_DATA,
  COMM_CUSTOM_APP_DATA,
  COMM_NRF_START_PAIRING,
  COMM_GPD_SET_FSW,
  COMM_GPD_BUFFER_NOTIFY,
  COMM_GPD_BUFFER_SIZE_LEFT,
  COMM_GPD_FILL_BUFFER,
  COMM_GPD_OUTPUT_SAMPLE,
  COMM_GPD_SET_MODE,
  COMM_GPD_FILL_BUFFER_INT8,
  COMM_GPD_FILL_BUFFER_INT16,
  COMM_GPD_SET_BUFFER_INT_SCALE,
  COMM_GET_VALUES_SETUP,
  COMM_SET_MCCONF_TEMP,
  COMM_SET_MCCONF_TEMP_SETUP,
  COMM_GET_VALUES_SELECTIVE,
  COMM_GET_VALUES_SETUP_SELECTIVE,
  COMM_EXT_NRF_PRESENT,
  COMM_EXT_NRF_ESB_SET_CH_ADDR,
  COMM_EXT_NRF_ESB_SEND_DATA,
  COMM_EXT_NRF_ESB_RX_DATA,
  COMM_EXT_NRF_SET_ENABLED,
  COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
  COMM_DETECT_APPLY_ALL_FOC,
  COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
  COMM_ERASE_NEW_APP_ALL_CAN,
  COMM_WRITE_NEW_APP_DATA_ALL_CAN,
  COMM_PING_CAN,
  COMM_APP_DISABLE_OUTPUT,
  COMM_TERMINAL_CMD_SYNC,
  COMM_GET_IMU_DATA,
  COMM_BM_CONNECT,
  COMM_BM_ERASE_FLASH_ALL,
  COMM_BM_WRITE_FLASH,
  COMM_BM_REBOOT,
  COMM_BM_DISCONNECT,
  COMM_BM_MAP_PINS_DEFAULT,
  COMM_BM_MAP_PINS_NRF5X,
  COMM_ERASE_BOOTLOADER,
  COMM_ERASE_BOOTLOADER_ALL_CAN,
  COMM_PLOT_INIT,
  COMM_PLOT_DATA,
  COMM_PLOT_ADD_GRAPH,
  COMM_PLOT_SET_GRAPH,
  COMM_GET_DECODED_BALANCE,
  COMM_BM_MEM_READ,
  COMM_WRITE_NEW_APP_DATA_LZO,
  COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
  COMM_BM_WRITE_FLASH_LZO,
  COMM_SET_CURRENT_REL,
  COMM_CAN_FWD_FRAME,
  COMM_SET_BATTERY_CUT,
  COMM_SET_BLE_NAME,
  COMM_SET_BLE_PIN,
  COMM_SET_CAN_MODE,
  COMM_GET_IMU_CALIBRATION,
  COMM_GET_MCCONF_TEMP,

  // Custom configuration for hardware
  COMM_GET_CUSTOM_CONFIG_XML,
  COMM_GET_CUSTOM_CONFIG,
  COMM_GET_CUSTOM_CONFIG_DEFAULT,
  COMM_SET_CUSTOM_CONFIG,

  // BMS commands
  COMM_BMS_GET_VALUES,
  COMM_BMS_SET_CHARGE_ALLOWED,
  COMM_BMS_SET_BALANCE_OVERRIDE,
  COMM_BMS_RESET_COUNTERS,
  COMM_BMS_FORCE_BALANCE,
  COMM_BMS_ZERO_CURRENT_OFFSET,

  // FW updates commands for different HW types
  COMM_JUMP_TO_BOOTLOADER_HW,
  COMM_ERASE_NEW_APP_HW,
  COMM_WRITE_NEW_APP_DATA_HW,
  COMM_ERASE_BOOTLOADER_HW,
  COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW,
  COMM_ERASE_NEW_APP_ALL_CAN_HW,
  COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW,
  COMM_ERASE_BOOTLOADER_ALL_CAN_HW,

  COMM_SET_ODOMETER,

  // Power switch commands
  COMM_PSW_GET_STATUS,
  COMM_PSW_SWITCH,

  COMM_BMS_FWD_CAN_RX,
  COMM_BMS_HW_DATA,
  COMM_GET_BATTERY_CUT,
  COMM_BM_HALT_REQ,
  COMM_GET_QML_UI_HW,
  COMM_GET_QML_UI_APP,
  COMM_CUSTOM_HW_DATA,
  COMM_QMLUI_ERASE,
  COMM_QMLUI_WRITE,

  // IO Board
  COMM_IO_BOARD_GET_ALL,
  COMM_IO_BOARD_SET_PWM,
  COMM_IO_BOARD_SET_DIGITAL,

  COMM_BM_MEM_WRITE,
  COMM_BMS_BLNC_SELFTEST,
  COMM_GET_EXT_HUM_TMP,
  COMM_GET_STATS,
  COMM_RESET_STATS,

  // Lisp
  COMM_LISP_READ_CODE,
  COMM_LISP_WRITE_CODE,
  COMM_LISP_ERASE_CODE,
  COMM_LISP_SET_RUNNING,
  COMM_LISP_GET_STATS,
  COMM_LISP_PRINT,

  COMM_BMS_SET_BATT_TYPE,
  COMM_BMS_GET_BATT_TYPE,

  COMM_LISP_REPL_CMD,
  COMM_LISP_STREAM_CODE,

  COMM_FILE_LIST,
  COMM_FILE_READ,
  COMM_FILE_WRITE,
  COMM_FILE_MKDIR,
  COMM_FILE_REMOVE,

  COMM_LOG_START,
  COMM_LOG_STOP,
  COMM_LOG_CONFIG_FIELD,
  COMM_LOG_DATA_F32,

  COMM_SET_APPCONF_NO_STORE,
  COMM_GET_GNSS,

  COMM_LOG_DATA_F64,
};

/**
 * @brief Map of return packets of COMM_GET_VALUES
 **/
enum PACKET_MAP
{
  TEMP_MOS = 1,
  TEMP_MOTOR = 3,
  CURRENT_MOTOR = 5,
  CURRENT_IN = 9,
  ID = 13,
  IQ = 17,
  DUTY_NOW = 21,
  ERPM = 23,
  VOLTAGE_IN = 27,
  AMP_HOURS = 29,
  AMP_HOURS_CHARGED = 33,
  WATT_HOURS = 37,
  WATT_HOURS_CHARGED = 41,
  TACHOMETER = 45,
  TACHOMETER_ABS = 49,
  FAULT_CODE = 53,
  PID_POS = 54,
  CONTROLLER_ID = 58,
  TEMP_MOS_MOTOR2 = 59,
  VD = 65,
  VQ = 69,
};

/**
 * @brief Modes and types
 **/
enum PWM_MODE
{
  PWM_MODE_NONSYNCHRONOUS_HISW = 0,
  PWM_MODE_SYNCHRONOUS,
  PWM_MODE_BIPOLAR,
};

enum COMM_MODE
{
  COMM_MODE_INTEGRATE = 0,
  COMM_MODE_DELAY,
};

enum SENSOR_MODE
{
  SENSOR_MODE_SENSORLESS = 0,
  SENSOR_MODE_SENSORED,
  SENSOR_MODE_HYBRID,
};

enum FOC_SENSOR_MODE
{
  FOC_SENSOR_MODE_SENSORLESS = 0,
  FOC_SENSOR_MODE_ENCODER,
  FOC_SENSOR_MODE_HALL,
  FOC_SENSOR_MODE_HFI,
  FOC_SENSOR_MODE_HFI_START,
};

enum SENSOR_PORT_MODE
{
  SENSOR_PORT_MODE_HALL = 0,
  SENSOR_PORT_MODE_ABI,
  SENSOR_PORT_MODE_AS5047_SPI,
  SENSOR_PORT_MODE_AD2S1205,
  SENSOR_PORT_MODE_SINCOS,
  SENSOR_PORT_MODE_TS5700N8501,
  SENSOR_PORT_MODE_TS5700N8501_MULTITURN,
  SENSOR_PORT_MODE_MT6816_SPI
};

enum MOTOR_TYPE
{
  MOTOR_TYPE_BLD = 0,
  MOTOR_TYPE_DC,
  MOTOR_TYPE_FOC,
  MOTOR_TYPE_GPD,
};

enum FOC_CC_DECOUPLING_MODE
{
  FOC_CC_DECOUPLING_DISABLED = 0,
  FOC_CC_DECOUPLING_CROSS,
  FOC_CC_DECOUPLING_BEMF,
  FOC_CC_DECOUPLING_CROSS_BEMF
};

enum FOC_OBSERVER_TYPE
{
  FOC_OBSERVER_ORTEGA_ORIGINAL = 0,
};

enum FOC_HFI_SAMPLES
{
  HFI_SAMPLES_8 = 0,
  HFI_SAMPLES_16,
  HFI_SAMPLES_32
};

enum MTPA_MODE
{
  MTPA_MODE_OFF = 0,
  MTPA_MODE_IQ_TARGET,
  MTPA_MODE_IQ_MEASURED
};

enum PID_RATE
{
  PID_RATE_25_HZ = 0,
  PID_RATE_50_HZ,
  PID_RATE_100_HZ,
  PID_RATE_250_HZ,
  PID_RATE_500_HZ,
  PID_RATE_1000_HZ,
  PID_RATE_2500_HZ,
  PID_RATE_5000_HZ,
  PID_RATE_10000_HZ,
};

enum DRV8301_OC_MODE
{
  DRV8301_OC_LIMIT = 0,
  DRV8301_OC_LATCH_SHUTDOWN,
  DRV8301_OC_REPORT_ONLY,
  DRV8301_OC_DISABLED
};

enum OUT_AUX_MODE
{
  OUT_AUX_MODE_OFF = 0,
  OUT_AUX_MODE_ON_AFTER_2S,
  OUT_AUX_MODE_ON_AFTER_5S,
  OUT_AUX_MODE_ON_AFTER_10S,
  OUT_AUX_MODE_UNUSED,
  OUT_AUX_MODE_ON_WHEN_RUNNING,
  OUT_AUX_MODE_ON_WHEN_NOT_RUNNING,
  OUT_AUX_MODE_MOTOR_50,
  OUT_AUX_MODE_MOSFET_50,
  OUT_AUX_MODE_MOTOR_70,
  OUT_AUX_MODE_MOSFET_70,
  OUT_AUX_MODE_MOTOR_MOSFET_50,
  OUT_AUX_MODE_MOTOR_MOSFET_70,
};

enum TEMP_SENSOR_TYPE
{
  TEMP_SENSOR_NTC_10K_25C = 0,
  TEMP_SENSOR_PTC_1K_100C,
  TEMP_SENSOR_KTY83_122,
  TEMP_SENSOR_NTC_100K_25C,
  TEMP_SENSOR_KTY84_130
};

enum BATTERY_TYPE
{
  BATTERY_TYPE_LIION_3_0__4_2,
  BATTERY_TYPE_LIIRON_2_6__3_6,
  BATTERY_TYPE_LEAD_ACID
};

enum BMS_TYPE
{
  BMS_TYPE_NONE = 0,
  BMS_TYPE_VESC
};

enum BMS_FWD_CAN_MODE
{
  BMS_FWD_CAN_MODE_DISABLED = 0,
  BMS_FWD_CAN_MODE_USB_ONLY,
  BMS_FWD_CAN_MODE_ANY
};

/**
 * @brief BMS configuration
 **/
struct BMS_CONFIG
{
  BMS_TYPE type;
  double t_limit_start;
  double t_limit_end;
  double soc_limit_start;
  double soc_limit_end;
  BMS_FWD_CAN_MODE fwd_can_mode;
};

/**
 * @brief MC configuration
 **/
struct MCConfiguration
{
  uint32_t signature;
  // Limits
  double l_current_max;
  double l_current_min;
  double l_in_current_max;
  double l_in_current_min;
  double l_abs_current_max;
  double l_min_erpm;
  double l_max_erpm;
  double l_erpm_start;
  double l_max_erpm_fbrake;
  double l_max_erpm_fbrake_cc;
  double l_min_vin;
  double l_max_vin;
  double l_battery_cut_start;
  double l_battery_cut_end;
  bool l_slow_abs_current;
  double l_temp_fet_start;
  double l_temp_fet_end;
  double l_temp_motor_start;
  double l_temp_motor_end;
  double l_temp_accel_dec;
  double l_min_duty;
  double l_max_duty;
  double l_watt_max;
  double l_watt_min;
  double l_current_max_scale;
  double l_current_min_scale;
  double l_duty_start;
  // Overridden limits (Computed during runtime)
  double lo_current_max;
  double lo_current_min;
  double lo_in_current_max;
  double lo_in_current_min;
  double lo_current_motor_max_now;
  double lo_current_motor_min_now;

  // BLDC switching and drive
  PWM_MODE pwm_mode;
  COMM_MODE comm_mode;
  MOTOR_TYPE motor_type;
  SENSOR_MODE sensor_mode;

  // Sensorless (bldc)
  double sl_min_erpm;
  double sl_min_erpm_cycle_int_limit;
  double sl_max_fullbreak_current_dir_change;
  double sl_cycle_int_limit;
  double sl_phase_advance_at_br;
  double sl_cycle_int_rpm_br;
  double sl_bemf_coupling_k;
  // Hall sensor
  int hall_table[8];
  double hall_sl_erpm;

  // FOC
  double foc_current_kp;
  double foc_current_ki;
  double foc_f_zv;
  double foc_dt_us;
  double foc_encoder_offset;
  bool foc_encoder_inverted;
  double foc_encoder_ratio;
  double foc_encoder_sin_offset;
  double foc_encoder_sin_gain;
  double foc_encoder_cos_offset;
  double foc_encoder_cos_gain;
  double foc_encoder_sincos_filter_constant;
  double foc_motor_l;
  double foc_motor_ld_lq_diff;
  double foc_motor_r;
  double foc_motor_flux_linkage;
  double foc_observer_gain;
  double foc_observer_gain_slow;
  double foc_observer_offset;
  double foc_pll_kp;
  double foc_pll_ki;
  double foc_duty_dowmramp_kp;
  double foc_duty_dowmramp_ki;
  double foc_openloop_rpm;
  double foc_openloop_rpm_low;
  double foc_d_gain_scale_start;
  double foc_d_gain_scale_max_mod;
  double foc_sl_openloop_hyst;
  double foc_sl_openloop_time;
  double foc_sl_openloop_time_lock;
  double foc_sl_openloop_time_ramp;
  FOC_SENSOR_MODE foc_sensor_mode;
  int foc_hall_table[8];
  double foc_hall_interp_erpm;
  double foc_sl_erpm;
  bool foc_sample_v0_v7;
  bool foc_sample_high_current;
  double foc_sat_comp;
  bool foc_temp_comp;
  double foc_temp_comp_base_temp;
  double foc_current_filter_const;
  FOC_CC_DECOUPLING_MODE foc_cc_decoupling;
  FOC_OBSERVER_TYPE foc_observer_type;
  double foc_hfi_voltage_start;
  double foc_hfi_voltage_run;
  double foc_hfi_voltage_max;
  double foc_sl_erpm_hfi;
  uint16_t foc_hfi_start_samples;
  double foc_hfi_obs_ovr_sec;
  FOC_HFI_SAMPLES foc_hfi_samples;
  bool foc_offsets_cal_on_boot;
  double foc_offsets_current[3];
  double foc_offsets_voltage[3];
  double foc_offsets_voltage_undriven[3];
  bool foc_phase_filter_enable;
  double foc_phase_filter_max_erpm;
  MTPA_MODE foc_mtpa_mode;
  // Field Weakening
  double foc_fw_current_max;
  double foc_fw_duty_start;
  double foc_fw_ramp_time;
  double foc_fw_q_current_factor;

  // GPDrive
  int gpd_buffer_notify_left;
  int gpd_buffer_interpol;
  double gpd_current_filter_const;
  double gpd_current_kp;
  double gpd_current_ki;

  PID_RATE sp_pid_loop_rate;

  // Speed PID
  double s_pid_kp;
  double s_pid_ki;
  double s_pid_kd;
  double s_pid_kd_filter;
  double s_pid_min_erpm;
  bool s_pid_allow_braking;
  double s_pid_ramp_erpms_s;

  // Pos PID
  double p_pid_kp;
  double p_pid_ki;
  double p_pid_kd;
  double p_pid_kd_proc;
  double p_pid_kd_filter;
  double p_pid_ang_div;
  double p_pid_gain_dec_angle;
  double p_pid_offset;

  // Current controller
  double cc_startup_boost_duty;
  double cc_min_current;
  double cc_gain;
  double cc_ramp_step_max;

  // Misc
  int32_t m_fault_stop_time_ms;
  double m_duty_ramp_step;
  double m_current_backoff_gain;
  uint32_t m_encoder_counts;
  SENSOR_PORT_MODE m_sensor_port_mode;
  bool m_invert_direction;
  DRV8301_OC_MODE m_drv8301_oc_mode;
  int m_drv8301_oc_adj;
  double m_bldc_f_sw_min;
  double m_bldc_f_sw_max;
  double m_dc_f_sw;
  double m_ntc_motor_beta;
  OUT_AUX_MODE m_out_aux_mode;
  TEMP_SENSOR_TYPE m_motor_temp_sens_type;
  double m_ptc_motor_coeff;
  int m_hall_extra_samples;
  // Setup info
  int si_motor_poles;
  double si_gear_ratio;
  double si_wheel_diameter;
  BATTERY_TYPE si_battery_type;
  int si_battery_cells;
  double si_battery_ah;
  double si_motor_nl_current;

  // BMS Configuration
  BMS_CONFIG bms;

  // Protect from flash corruption.
  uint16_t crc;
};

#endif  // VESC_DRIVER_DATA_MAP_HPP_
