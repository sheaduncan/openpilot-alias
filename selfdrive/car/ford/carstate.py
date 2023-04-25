from cereal import car
from opendbc.can.parser import CANParser
from common.conversions import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.ford.values import DBC

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.toggle_last = False
    self.debug_cruise = False

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # steering wheel
    self.offset = cp_cam.vl["Steering_Sensor"]['offset']
    ret.steeringAngleDeg = cp_cam.vl["Steering_Sensor"]['StePinRelInit_An_Sns'] - self.offset

    ret.genericToggle = bool(cp.vl["Steering_Wheel_Data2_FD1"]['SteWhlSwtchOk_B_Stat'])

    self.vSpeed = cp.vl["BrakeSysFeatures"]['Veh_V_ActlBrk']
    self.sappControlState = cp_cam.vl["EPAS_INFO"]['SAPPAngleControlStat1']

    ret.vEgoRaw = self.vSpeed * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = cp.vl["DesiredTorqBrk"]['VehStop_D_Stat'] == 1

    ret.gasPressed = cp.vl["EngVehicleSpThrottle"]['ApedPos_Pc_ActlArb'] / 100. > 1e-6
    ret.brakePressed = cp.vl["EngBrakeData"]['BpedDrvAppl_D_Actl'] == 2

    ret.cruiseState.enabled = cp.vl["EngBrakeData"]['CcStat_D_Actl'] == 5
    ret.cruiseState.available = cp.vl["EngBrakeData"]['CcStat_D_Actl'] != 0
    ret.cruiseState.speed = cp.vl["EngBrakeData"]['Veh_V_DsplyCcSet'] * CV.MPH_TO_MS

    ret.steeringTorque = cp_cam.vl["EPAS_INFO"]['SteeringColumnTorque']
    ret.steeringPressed = False

    gear = cp.vl["TransGearData"]['GearLvrPos_D_Actl']
    if gear == 0:
      ret.gearShifter = GearShifter.park
    elif gear == 1:
      ret.gearShifter = GearShifter.reverse
    elif gear == 2:
      ret.gearShifter = GearShifter.neutral
    elif gear == 3:
      ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = GearShifter.unknown

    ret.doorOpen = any([cp.vl["BodyInfo_3_FD1"]['DrStatDrv_B_Actl'],cp.vl["BodyInfo_3_FD1"]['DrStatPsngr_B_Actl'], cp.vl["BodyInfo_3_FD1"]['DrStatRl_B_Actl'], cp.vl["BodyInfo_3_FD1"]['DrStatRr_B_Actl']])
    ret.seatbeltUnlatched = cp.vl["RCMStatusMessage2_FD1"]['FirstRowBuckleDriver'] == 2

    ret.leftBlinker = cp.vl["Steering_Buttons"]['Left_Turn_Light'] != 0
    ret.rightBlinker = cp.vl["Steering_Buttons"]['Right_Turn_Light'] != 0

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [ # On OP 0.8.13 we no longer have the third argument
      ("Veh_V_ActlBrk", "BrakeSysFeatures"),
      ("ApedPos_Pc_ActlArb", "EngVehicleSpThrottle"),
      ("BpedDrvAppl_D_Actl", "EngBrakeData"),
      ("SteWhlRelInit_An_Sns", "BrakeSnData_5"),
      ("SteWhlSwtchOk_B_Stat", "Steering_Wheel_Data2_FD1"),
      ("Veh_V_DsplyCcSet", "EngBrakeData"),
      ("CcStat_D_Actl", "EngBrakeData"),
      ("GearLvrPos_D_Actl", "TransGearData"),
      ("DrStatDrv_B_Actl", "BodyInfo_3_FD1"),
      ("DrStatPsngr_B_Actl", "BodyInfo_3_FD1"),
      ("DrStatRl_B_Actl", "BodyInfo_3_FD1"),
      ("DrStatRr_B_Actl", "BodyInfo_3_FD1"),
      ("Left_Turn_Light", "Steering_Buttons"),
      ("Right_Turn_Light", "Steering_Buttons"),
      ("FirstRowBuckleDriver", "RCMStatusMessage2_FD1"),
      ("VehStop_D_Stat", "DesiredTorqBrk"),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("SteeringColumnTorque", "EPAS_INFO"),
      ("SteMdule_D_Stat", "EPAS_INFO"),
      ("SAPPAngleControlStat1", "EPAS_INFO"),
      ("StePinRelInit_An_Sns", "Steering_Sensor"),
      ("offset", "Steering_Sensor"),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2, enforce_checks=False)
