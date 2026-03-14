from opendbc.car.mazda.values import Buttons, MazdaFlags

# CRZ_CTRL mode template bytes (proven on-car by testkit)
CRZ_CTRL_STANDBY = bytes.fromhex("02010b0000000000")
CRZ_CTRL_ENGAGED_CRUISE = bytes.fromhex("0a018b2000001000")
CRZ_CTRL_ENGAGED_FOLLOW = bytes.fromhex("0a018b4000001000")
CRZ_CTRL_STOP_GO = bytes.fromhex("0a018b6000001000")
CRZ_CTRL_MODE_BYTE_INDEXES = (0, 2, 3, 6)


def create_crz_info(packer, accel_cmd, acc_active, ctr):
  """Build CRZ_INFO (0x21B) — the main longitudinal command to the PCM.

  accel_cmd: physical DBC value (ACCEL_CMD signal, offset -4096). 0 = no accel request.
  acc_active: 1 when cruise actively controlling, 0 otherwise.
  ctr: 8-bit rolling counter (mod 256).
  """
  values = {
    "ACCEL_CMD": accel_cmd,
    "ACC_ACTIVE": int(acc_active),
    "ACC_SET_ALLOWED": 1,
    "CTR1": ctr % 256,
    "ERROR_STATUS": 0,
    "CRZ_ENDED": 0,
  }
  msg = packer.make_can_msg("CRZ_INFO", 0, values)
  # Fix checksum: inverted sum of bytes 0-6
  dat = bytearray(msg[1])
  dat[7] = (0xFF - (sum(dat[i] for i in range(7)) & 0xFF)) & 0xFF
  return (msg[0], bytes(dat), msg[2])


def create_crz_ctrl(template_bytes, raw_21c_base=None):
  """Build CRZ_CTRL (0x21C) — the cruise state/mode gate message.

  template_bytes: one of CRZ_CTRL_ENGAGED_CRUISE/FOLLOW/STOP_GO/STANDBY
  raw_21c_base: optional base bytes to overlay mode onto (preserves non-mode bytes)
  """
  if raw_21c_base is not None:
    dat = bytearray(raw_21c_base)
    for i in CRZ_CTRL_MODE_BYTE_INDEXES:
      dat[i] = template_bytes[i]
    return (0x21C, bytes(dat), 0)
  return (0x21C, template_bytes, 0)


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": inc,
      "SET_P_INV": (inc + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": dec,
      "SET_M_INV": (dec + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
