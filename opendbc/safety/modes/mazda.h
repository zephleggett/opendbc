#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/longitudinal.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
#define MAZDA_RADAR_DIAG    0x764U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

static bool mazda_longitudinal = false;

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (!mazda_longitudinal && (msg->addr == MAZDA_CRZ_CTRL)) {
      // Stock ACC mode: cruise engagement from radar's CRZ_CTRL
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 17U);
    }

    // Longitudinal mode: button-based engagement (Honda Bosch pattern)
    // CRZ_CTRL is suppressed by radar programming session, so we use
    // CRZ_BTNS from the stock stalk for engagement tracking.
    if (mazda_longitudinal && (msg->addr == MAZDA_CRZ_BTNS)) {
      // CAN_OFF (cancel) = bit 0, RES = bit 2, SET_P = bit 4, SET_M = bit 5
      bool cancel = msg->data[0] & 0x01U;
      bool engage = (msg->data[0] & 0x34U) != 0U;  // RES | SET_P | SET_M

      if (engage && !cancel) {
        controls_allowed = true;
      }
      if (cancel) {
        controls_allowed = false;
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      brake_pressed = (msg->data[0] & 0x10U);
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rate_up = 15,
    .max_rate_down = 38,
    .max_rt_delta = 450,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // CRZ_INFO ACCEL_CMD limits in DBC physical units (~0.001294 m/s^2 per unit)
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 1545,   // ~2.0 m/s^2
    .min_accel = -2704,  // ~-3.5 m/s^2
    .inactive_accel = 0,
  };

  bool tx = true;
  // Check if msg is sent on the main BUS
  if (msg->bus == (unsigned char)MAZDA_MAIN) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
        tx = false;
      }
    }

    // cruise buttons check (only in stock ACC mode)
    if (msg->addr == MAZDA_CRZ_BTNS) {
      if (!mazda_longitudinal) {
        // allow resume spamming while controls allowed, but
        // only allow cancel while controls not allowed
        bool cancel_cmd = (msg->data[0] == 0x1U);
        if (!controls_allowed && !cancel_cmd) {
          tx = false;
        }
      }
    }

    if (mazda_longitudinal) {
      // CRZ_INFO accel limit check
      if (msg->addr == MAZDA_CRZ_INFO) {
        // ACCEL_CMD: 13-bit big-endian at DBC start bit 17, unsigned, offset -4096
        // Byte 2 bits [1:0] (2 MSBs) | Byte 3 bits [7:0] (8 bits) | Byte 4 bits [7:5] (3 LSBs)
        int raw_accel = ((msg->data[2] & 0x03U) << 11) | (msg->data[3] << 3) | ((msg->data[4] >> 5) & 0x07U);
        int accel_cmd = raw_accel - 4096;

        if (longitudinal_accel_checks(accel_cmd, MAZDA_LONG_LIMITS)) {
          tx = false;
        }
      }

      // Tester-present to radar: only allow exact payload
      if (msg->addr == MAZDA_RADAR_DIAG) {
        if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
          tx = false;
        }
      }
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
  };

  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = true},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = true},
    {MAZDA_RADAR_DIAG, 0, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // Longitudinal rx_checks: CRZ_CTRL is suppressed by radar programming session,
  // so we exclude it and use CRZ_BTNS for engagement tracking instead.
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, 1U);

  if (mazda_longitudinal) {
    // ACC main switch state comes from CRZ_CTRL which is suppressed;
    // set true since driver must have ACC main on to use cruise buttons.
    acc_main_on = true;
    return BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS);
  }
  return BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
