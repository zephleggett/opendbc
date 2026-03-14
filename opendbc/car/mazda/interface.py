#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, MAZDA_RADAR_ADDR, MazdaFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)

    ret.enableBsm = 0x477 in fingerprint[0]

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022,):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    # Longitudinal control for CX-5 2022+
    if candidate in (CAR.MAZDA_CX5_2022,):
      ret.alphaLongitudinalAvailable = True
      if alpha_long:
        ret.openpilotLongitudinalControl = True
        ret.pcmCruise = False
        ret.radarUnavailable = True  # radar tracks lost during programming session
        ret.flags |= MazdaFlags.RADAR_DISABLED
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda, 1)]  # param=1 enables long

        ret.longitudinalActuatorDelay = 0.5  # conservative start, tune later
        ret.longitudinalTuning.kpBP = [0.]
        ret.longitudinalTuning.kpV = [0.5]
        ret.longitudinalTuning.kiBP = [0., 35.]
        ret.longitudinalTuning.kiV = [0.5, 0.3]
        ret.stoppingDecelRate = 0.6
        ret.startingState = True
        ret.startAccel = 1.0

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send, communication_control=None):
    """Suppress the radar ECU by entering programming session.

    Unlike Honda Bosch which uses COMMUNICATION_CONTROL (0x28),
    the 2022 CX-5 radar does not support that service (NRC 0x11).
    Instead, entering programming session and holding tester-present
    suppresses CRZ_INFO, CRZ_CTRL, and radar tracks.
    """
    if CP.flags & MazdaFlags.RADAR_DISABLED:
      from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
      session_req = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL,
                           uds.SESSION_TYPE.PROGRAMMING])
      session_resp = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40])
      for _ in range(10):
        try:
          query = IsoTpParallelQuery(can_send, can_recv, 0, [(MAZDA_RADAR_ADDR, None)],
                                     [session_req], [session_resp])
          query.get_data(0.1)
          return True
        except Exception:
          pass
      return False

  @staticmethod
  def deinit(CP, can_recv, can_send):
    """Return radar to default session."""
    if CP.flags & MazdaFlags.RADAR_DISABLED:
      from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
      try:
        session_req = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL,
                             uds.SESSION_TYPE.DEFAULT])
        session_resp = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40])
        query = IsoTpParallelQuery(can_send, can_recv, 0, [(MAZDA_RADAR_ADDR, None)],
                                   [session_req], [session_resp])
        query.get_data(0.1)
      except Exception:
        pass

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret
