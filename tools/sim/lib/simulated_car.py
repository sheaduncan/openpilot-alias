import datetime
import cereal.messaging as messaging

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from openpilot.selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp
from openpilot.selfdrive.car import crc8_pedal
from openpilot.tools.sim.lib.common import SimulatorState


class SimulatedCar:
  """Simulates a honda civic 2016 (panda state + can messages) to OpenPilot"""
  dbc= "ford_lincoln_base_pt"
  rdbc= "FORD_CADS"
  packer = CANPacker(dbc)
  rpacker = CANPacker(rdbc)

  KEEP_TORQUE_STEPS = 0.15 * 100
  KEEP_BLINKER_STEPS = 5 * 100

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams'])
    self.cp = self.get_car_can_parser()
    self.idx = 0
    self.last_cruise_button = 0
    self.cruise_speed = 31

    self.blinker_steps_left = 0
    self.blinker_value = 0

    self.torque_steps_left = 0
    self.torque_value = 0


  @staticmethod
  def get_car_can_parser():
    dbc_f = SimulatedCar.dbc
    checks = [
      ("Yaw_Data_FD1", 100),
      ("EngBrakeData", 10),
    ]
    return CANParser(dbc_f, checks, 0)

  def send_can_messages(self, simulator_state: SimulatorState):
    if not simulator_state.valid:
      return

    msg = []

    msg.append(self.packer.make_can_msg("DesiredTorqBrk", 0, {}))
    msg.append(self.packer.make_can_msg("RCMStatusMessage2_FD1", 0, {}))
    msg.append(self.packer.make_can_msg("BodyInfo_3_FD1", 0, {}))
    msg.append(self.packer.make_can_msg("Engine_Clutch_Data", 0, {}))
    msg.append(self.packer.make_can_msg("BCM_Lamp_Stat_FD1", 0, {}))
                                        
    # *** powertrain bus ***

    # This is not a hybrid
    msg.append(self.packer.make_can_msg("VehicleOperatingModes", 0, {"TrnAinTq_D_Qf": 1}))
    # set the NonAdaptive to false
    msg.append(self.packer.make_can_msg("Cluster_Info1_FD1", 0, {"AccEnbl_B_RqDrv": 1}))

    # Steering errors
    msg.append(self.packer.make_can_msg("Lane_Assist_Data3_FD1", 0, {"LatCtlSte_D_Stat": 1}))


    speed = simulator_state.speed * 3.6 # convert m/s to kph
    msg.append(self.packer.make_can_msg("BrakeSysFeatures", 0, {"Veh_V_ActlBrk": speed}))

    if simulator_state.cruise_button > 0:
      print(simulator_state.cruise_button)
      self.last_cruise_button = simulator_state.cruise_button

    if simulator_state.cruise_button == 3:
      self.cruise_speed -= 1
    elif simulator_state.cruise_button == 4:
      self.cruise_speed += 1

    if self.cruise_speed < 0:
      self.cruise_speed = 0
    if self.cruise_speed > 75:
      self.cruise_speed = 75

    msg.append(self.packer.make_can_msg("EngBrakeData", 0, {
      "CcStat_D_Actl": 4 if self.last_cruise_button in (3, 4) else 0,
      "Veh_V_DsplyCcSet": self.cruise_speed,
      "BpedDrvAppl_D_Actl": 2 if simulator_state.user_brake > 0 else 0
      }))


    # # values = {
    # #   "COUNTER_PEDAL": self.idx & 0xF,
    # #   "INTERCEPTOR_GAS": simulator_state.user_gas * 2**12,
    # #   "INTERCEPTOR_GAS2": simulator_state.user_gas * 2**12,
    # # }
    # # checksum = crc8_pedal(self.packer.make_can_msg("GAS_SENSOR", 0, values)[2][:-1])
    # # values["CHECKSUM_PEDAL"] = checksum
    # # msg.append(self.packer.make_can_msg("GAS_SENSOR", 0, values))
    msg.append(self.packer.make_can_msg("EngVehicleSpThrottle", 0, {"ApedPos_Pc_ActlArb": simulator_state.user_gas * 100}))

    msg.append(self.packer.make_can_msg("Gear_Shift_by_Wire_FD1", 0, {"TrnRng_D_RqGsm": 3}))
    # # msg.append(self.packer.make_can_msg("GAS_PEDAL_2", 0, {}))
    # msg.append(self.packer.make_can_msg("RCMStatusMessage2_FD1", 0, {"FirstRowBuckleDriver": 2}))

    if abs(simulator_state.user_torque) > 0:
      print("torque_value")
      self.torque_value = 1.1 if simulator_state.user_torque > 0 else -1.1
      self.torque_steps_left = self.KEEP_TORQUE_STEPS

    if self.torque_steps_left > 0:
      self.torque_steps_left -= 1
    else:
      self.torque_value = 0

    msg.append(self.packer.make_can_msg("EPAS_INFO", 0, {"SteeringColumnTorque": self.torque_value}))
    
    msg.append(self.packer.make_can_msg("SteeringPinion_Data", 0, {
      "StePinComp_An_Est": simulator_state.steering_angle,
      "StePinCompAnEst_D_Qf": 3
      }))
    msg.append(self.packer.make_can_msg("BrakeSnData_4", 0, {}))
    # # msg.append(self.packer.make_can_msg("STANDSTILL", 0, {"WHEELS_MOVING": 1 if simulator_state.speed >= 1.0 else 0}))
    # msg.append(self.packer.make_can_msg("DesiredTorqBrk", 0, {"PrkBrkStatus": 0, "VehStop_D_Stat": 0 if simulator_state.speed >= 1.0 else 1}))
    # # msg.append(self.packer.make_can_msg("STEER_MOTOR_TORQUE", 0, {}))
    # msg.append(self.packer.make_can_msg("BodyInfo_3_FD1", 0, {}))
    # # msg.append(self.packer.make_can_msg("CRUISE_PARAMS", 0, {}))

    # Let's handle the blinkers for lane change
    if simulator_state.left_blinker > 0 or simulator_state.right_blinker > 0:
      self.blinker_value = 1 if simulator_state.left_blinker > 0 else 2 if simulator_state.right_blinker > 0 else 0
      self.blinker_steps_left = self.KEEP_BLINKER_STEPS
    
    if self.blinker_steps_left > 0:
      self.blinker_steps_left -= 1
    else:
      self.blinker_value = 0

    msg.append(self.packer.make_can_msg("Steering_Data_FD1", 0,
                                        { 
                                          "TurnLghtSwtch_D_Stat": self.blinker_value
                                        }))
    
    # if simulator_state.left_blinker > 0 or simulator_state.right_blinker > 0:
    #   msg.append(self.packer.make_can_msg("EPAS_INFO", 0, {"SteeringColumnTorque": 5}))

    # msg.append(self.packer.make_can_msg("EngBrakeData", 0,
    #                                     {
    #                                       "Veh_V_DsplyCcSet": 100,
    #                                       # "CcStat_D_Actl": 4 if simulator_state.cruise_button == 1 else 0, 
    #                                       "CcStat_D_Actl": 4, 
    #                                       "BpedDrvAppl_D_Actl": simulator_state.user_brake > 0
    #                                     }))

    # avoid errors    
    msg.append(self.packer.make_can_msg("Yaw_Data_FD1", 0, {}))
    msg.append(self.packer.make_can_msg("ACCDATA", 2, {}))
    msg.append(self.packer.make_can_msg("ACCDATA_2", 2, {}))
    msg.append(self.packer.make_can_msg("ACCDATA_3", 2, {}))
    msg.append(self.packer.make_can_msg("IPMA_Data", 2, {}))
    msg.append(self.packer.make_can_msg("IPMA_Data2", 2, {}))
    msg.append(self.packer.make_can_msg("Side_Detect_L_Stat", 2, {}))
    msg.append(self.packer.make_can_msg("Side_Detect_R_Stat", 2, {}))
    
    # # *** radar bus ***
    # if self.idx % 5 == 0:
    #   msg.append(self.rpacker.make_can_msg("RADAR_DIAGNOSTIC", 1, {"RADAR_STATE": 0x79}))
    #   for i in range(16):
    #     msg.append(self.rpacker.make_can_msg("TRACK_%d" % i, 1, {"LONG_DIST": 255.5}))

    self.pm.send('can', can_list_to_can_capnp(msg))

  def send_panda_state(self, simulator_state):
    self.sm.update(0)
    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': simulator_state.ignition,
      'pandaType': "tres",
      'controlsAllowed': True,
      'safetyModel': 'ford',
      'safetyParam': 3,
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
    }
    self.pm.send('pandaStates', dat)

  def update(self, simulator_state: SimulatorState):
    self.send_can_messages(simulator_state)

    if self.idx % 50 == 0: # only send panda states at 2hz
      self.send_panda_state(simulator_state)

    self.idx += 1