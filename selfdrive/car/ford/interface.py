#!/usr/bin/env python3
from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car.ford.values import CAR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "ford"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.ford)]
    ret.dashcamOnly = False
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate == CAR.F150:
      ret.mass = 4770. * CV.LB_TO_KG + STD_CARGO_KG
      ret.steerRatio = 17
      ret.wheelbase = 3.68
      ret.steerActuatorDelay = 0.1
      ret.openpilotLongitudinalControl = False
    else:
      raise ValueError(f"Unsupported car: {candidate}")

    ret.centerToFront = ret.wheelbase * 0.44
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    ret.events = self.create_common_events(ret).to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
