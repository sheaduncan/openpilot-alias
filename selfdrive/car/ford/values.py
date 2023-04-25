from selfdrive.car import dbc_dict
from collections import namedtuple
from cereal import car
from typing import Dict, List, Union
from selfdrive.car.docs_definitions import CarInfo
from selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
Ecu = car.CarParams.Ecu

AngleRateLimit = namedtuple('AngleRateLimit', ['speed_points', 'max_angle_diff_points'])
AngleLimit = namedtuple('AngleLimit', ['speed_points', 'max_angle_points'])

class CarControllerParams:

  # These rate limits are also enforced by the Panda safety code.
  RATE_LIMIT_UP = AngleRateLimit(speed_points=[0., 5., 15.], max_angle_diff_points=[7.5, 1.2, .225])
  RATE_LIMIT_DOWN = AngleRateLimit(speed_points=[0., 5., 15.], max_angle_diff_points=[7.5, 5.25, 0.6])

  APA_STEP = 2 # 50hz
  # Angle limits
  
  ANGLE_LIMIT = AngleLimit(speed_points=[0., 5., 15.], max_angle_points=[180., 90., 45.])

class CAR:
  F150 = 'F150'

CAR_INFO: Dict[str, Union[CarInfo, List[CarInfo]]] = {
  CAR.F150: CarInfo("Ford F150", "All"),
}

FW_VERSIONS = {
  CAR.F150: {
    (Ecu.engine, 0x7e0, None): [
      b'GL3A-14C204-JD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}


DBC = {
  CAR.F150: dbc_dict('ford_f150', 'toyota_tss2_adas'),
}

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
    ),
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=4,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=4,
    ),
  ],
)

# addr: (bus, 1/freq*100, vl)
TSS2_RADAR_INIT = [
  # CAN Bus 0 (Chassis)
  # CAN Bus 1 (Radar)
  # CAN Bus 2 (IPMA)
  # TODO: Only send what is needed and for as long as it is needed. This spams can
  #(0x128, 0,   100, b'\xf4\x01\x90\x83\x00\x37'),
  #(0x141, 0,   100, b'\x00\x00\x00\x46'),
  #(0x160, 0,   100, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  #(0x161, 0,   100, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x128, 1,   100, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, 1,   100, b'\x00\x00\x00\x46'),
  (0x160, 1,   100, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, 1,   100, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x128, 1,   100, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, 1,   100, b'\x00\x00\x00\x46'),
  (0x160, 1,   100, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, 1,   100, b'\x00\x1e\x00\x00\x00\x80\x07'),

  #(0x283, 0,   100, b'\x00\x00\x00\x00\x00\x00\x8c'),
  #(0x344, 0,   100, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  #(0x365, 0,  100, b'\x00\x00\x00\x80\xfc\x00\x08'),
  #(0x366, 0,  100, b'\x00\x72\x07\xff\x09\xfe\x00'),
  #(0x4CB, 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
  (0x283, 1,   100, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x344, 1,   100, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, 1,  100, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, 1,  100, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, 1, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
  (0x283, 1,   100, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x344, 1,   100, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, 1,  100, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, 1,  100, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, 1, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]
