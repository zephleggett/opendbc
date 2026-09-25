import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.accel = 0.

  def update(self, CC, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

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

    if self.CP.openpilotLongitudinalControl:
      # the radar is silent in a programming session and stays there while it hears tester present
      if self.frame % 50 == 0:
        if CS.radar_disabled:
          can_sends.append(make_tester_present_msg(mazdacan.RADAR_ADDR, 0, suppress_response=True))
        elif CS.radar_request:
          can_sends.append(mazdacan.create_radar_session_request(0))

      # the camera faults without the radar's messages, and the panda doesn't forward ours to it
      if CS.radar_disabled and not CS.out.accFaulted:
        if self.frame % 2 == 0:
          self.accel = 0.
          if CC.longActive:
            self.accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
            # the radar relaxes its command once Auto Hold holds the car
            if CS.brake_hold:
              self.accel = -0.001

          stopping = CC.actuators.longControlState == LongCtrlState.stopping
          # Auto Hold lets go on RESUME_UNLATCHING
          resume = CC.longActive and not stopping and CS.brake_hold
          available = CS.out.cruiseState.available
          for bus in (0, 2):
            can_sends.append(mazdacan.create_acc_command(self.packer, bus, self.frame // 2, self.accel, CC.enabled, available,
                                                         stopping and not CS.brake_hold, resume))
            can_sends.append(mazdacan.create_crz_ctrl(self.packer, bus, CC.enabled, available, CC.hudControl.leadDistanceBars,
                                                      CS.brake_hold, CS.cam_laneinfo["BIT2"]))

        if self.frame % 10 == 0:
          can_sends.extend(mazdacan.create_radar_frames(2, self.frame // 10))

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

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    if self.CP.openpilotLongitudinalControl:
      new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
