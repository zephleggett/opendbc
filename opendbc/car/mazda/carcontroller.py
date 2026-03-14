from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, structs
from opendbc.car.common.numpy_fast import clip
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons, MAZDA_RADAR_ADDR, MazdaFlags

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Empirical scale from Phase 0 analysis: ~0.001294 m/s^2 per DBC unit
ACCEL_SCALE = 0.001294


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.crz_info_ctr = 0
    self.last_accel = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params)

    # --- Longitudinal control ---
    if self.CP.openpilotLongitudinalControl:
      # Tester-present keepalive to hold radar in programming session (~every 500ms at 100Hz)
      if self.frame % 50 == 0:
        can_sends.append(make_tester_present_msg(MAZDA_RADAR_ADDR, 0, suppress_response=True))

      # Convert m/s^2 to DBC ACCEL_CMD units
      accel = clip(CC.actuators.accel, self.params.ACCEL_MIN * ACCEL_SCALE, self.params.ACCEL_MAX * ACCEL_SCALE)
      accel_cmd = int(round(accel / ACCEL_SCALE))
      accel_cmd = clip(accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

      if CC.longActive:
        # Choose CRZ_CTRL mode based on state
        if CS.out.standstill:
          crz_ctrl_template = mazdacan.CRZ_CTRL_STOP_GO
        else:
          crz_ctrl_template = mazdacan.CRZ_CTRL_ENGAGED_CRUISE
        acc_active = True
      else:
        accel_cmd = 0
        crz_ctrl_template = mazdacan.CRZ_CTRL_STANDBY
        acc_active = False

      # Send CRZ_INFO + CRZ_CTRL at 50Hz (every other frame at 100Hz)
      if self.frame % 2 == 0:
        can_sends.append(mazdacan.create_crz_info(self.packer, accel_cmd, acc_active, self.crz_info_ctr))
        can_sends.append(mazdacan.create_crz_ctrl(crz_ctrl_template))
        self.crz_info_ctr = (self.crz_info_ctr + 1) % 256

      self.last_accel = accel
    else:
      # Stock ACC mode — button-based cruise control
      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    # Intelligent Cruise Button Management (only in stock ACC mode)
    if not self.CP.openpilotLongitudinalControl:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    if self.CP.openpilotLongitudinalControl:
      new_actuators.accel = self.last_accel

    self.frame += 1
    return new_actuators, can_sends
