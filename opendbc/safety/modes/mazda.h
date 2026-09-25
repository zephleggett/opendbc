#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
#define MAZDA_RADAR_UDS     0x764U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

#define MAZDA_PARAM_LONGITUDINAL 1U

static bool mazda_longitudinal = false;

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if (msg_matches(msg, MAZDA_ENGINE_DATA, MAZDA_MAIN)) {
    // sample speed: scale by 0.01 to get kph
    int speed = (msg->data[2] << 8) | msg->data[3];
    vehicle_moving = speed > 10; // moving when speed > 0.1 kph
  }

  if (msg_matches(msg, MAZDA_STEER_TORQUE, MAZDA_MAIN)) {
    int torque_driver_new = msg->data[0] - 127U;
    // update array of samples
    update_sample(&torque_driver, torque_driver_new);
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  if (msg_matches(msg, MAZDA_CRZ_CTRL, MAZDA_MAIN)) {
    bool cruise_engaged = msg->data[0] & 0x8U;
    pcm_cruise_check(cruise_engaged);
  }

  if (msg_matches(msg, MAZDA_ENGINE_DATA, MAZDA_MAIN)) {
    gas_pressed = (msg->data[4] | (msg->data[5] & 0xF0U)) != 0U;
  }

  if (msg_matches(msg, MAZDA_PEDALS, MAZDA_MAIN)) {
    brake_pressed = (msg->data[0] & 0x10U);

    // with the radar silenced, the cruise state comes from the body
    if (mazda_longitudinal) {
      pcm_cruise_check(GET_BIT(msg, 3U));
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // 0.001 m/s^2
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;
  // Check if msg is sent on the main BUS
  // steer cmd checks
  if (msg_matches(msg, MAZDA_LKAS, MAZDA_MAIN)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // cruise buttons check
  if (msg_matches(msg, MAZDA_CRZ_BTNS, MAZDA_MAIN)) {
    // allow resume spamming while controls allowed, but
    // only allow cancel while controls not allowed
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
    }
  }

  // sent on both buses in place of the radar
  if (msg->addr == MAZDA_CRZ_INFO) {
    // 13 bits with a 4096 offset
    uint32_t accel_raw = (((uint32_t)msg->data[2] & 0x3U) << 11) | ((uint32_t)msg->data[3] << 3) | ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)accel_raw - 4096;
    bool acc_active = GET_BIT(msg, 33U);
    bool resume = GET_BIT(msg, 54U);  // releases Auto Hold

    // the radar pegs the command while it isn't engaged
    bool standby = !acc_active && (desired_accel == 4094);
    if (!standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }
    if ((acc_active || resume) && !controls_allowed) {
      tx = false;
    }
  }

  if (msg->addr == MAZDA_CRZ_CTRL) {
    bool crz_active = GET_BIT(msg, 3U);
    if (crz_active && !controls_allowed) {
      tx = false;
    }
  }

  // tester present, or the programming session request that silences the radar at a standstill
  if (msg->addr == MAZDA_RADAR_UDS) {
    uint64_t dat = GET_BYTES_64_LE(msg, 0, 8);
    bool tester_present = dat == 0x0000000000803E02ULL;
    bool session_request = (dat == 0x0000000000021002ULL) && !vehicle_moving;
    if (!tester_present && !session_request) {
      tx = false;
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};

  // the radar is only silenced once the camera has booted, so its messages are not relay checked
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false}, {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false}, {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, 2, 8, .check_relay = false}, {MAZDA_CRZ_CTRL, 2, 8, .check_relay = false},
    // radar tracks and status, for the camera
    {0x361, 2, 8, .check_relay = false}, {0x362, 2, 8, .check_relay = false}, {0x363, 2, 8, .check_relay = false},
    {0x364, 2, 8, .check_relay = false}, {0x365, 2, 8, .check_relay = false}, {0x366, 2, 8, .check_relay = false},
    {0x499, 2, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // the radar's CRZ_CTRL goes away once it is silenced
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) : BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
