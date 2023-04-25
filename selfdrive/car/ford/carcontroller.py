from common.numpy_fast import interp, clip
from selfdrive.car.ford.fordcan import spam_cancel_button, ParkAid_Data, EngVehicleSpThrottle2, BrakeSysFeatures
from selfdrive.car.ford.values import CarControllerParams, TSS2_RADAR_INIT
from opendbc.can.packer import CANPacker
from common.conversions import Conversions as CV
from selfdrive.car import make_can_msg

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams
    self.steer_delay = 1
    self.steer_enabled = False
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    #*** TSS2 Radar Init  ***
    for (addr, bus, fr_step, vl) in TSS2_RADAR_INIT:
      if self.frame % fr_step == 0:
        can_sends.append(make_can_msg(addr, vl, bus))
        #print(addr, vl, bus)

    # Send cancel button if requested
    if CC.cruiseControl.cancel:
      can_sends.append(spam_cancel_button(self.packer))

    if True:
      apply_speed = CS.out.vEgoRaw * CV.MS_TO_KPH
      apply_steer = CS.out.steeringAngleDeg

      if CC.enabled:
        apply_speed = 0
        if self.steer_delay <= 150:
          self.steer_delay += 1
        else:
          self.steer_enabled = True
          steer = interp(actuators.steer, [-1, 1], [-20, 20])
          steer = clip(steer, -10, 10)
          apply_steer = CS.out.steeringAngleDeg + steer
          print(actuators.steer, CS.out.steeringAngleDeg - apply_steer)
      else:
        self.steer_delay = 1
        self.steer_enabled = False

      can_sends.append(BrakeSysFeatures(self.packer, self.frame, apply_speed))
      can_sends.append(EngVehicleSpThrottle2(self.packer, self.frame, apply_speed, CS.out.gearShifter))
      can_sends.append(ParkAid_Data(self.packer, self.steer_enabled, apply_steer, CS.sappControlState, CS.out.standstill))
      self.frame += 1

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_steer

    self.frame += 1
    return new_actuators, can_sends
