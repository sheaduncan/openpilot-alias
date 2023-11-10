from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum, StrEnum
from typing import Dict, List, Union

from cereal import car
from openpilot.selfdrive.car import AngleRateLimit, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarFootnote, CarHarness, CarInfo, CarParts, Column, \
                                           Device
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 5        # LateralMotionControl, 20Hz
  LKA_STEP = 3          # Lane_Assist_Data1, 33Hz
  ACC_CONTROL_STEP = 2  # ACCDATA, 50Hz
  LKAS_UI_STEP = 100    # IPMA_Data, 1Hz
  ACC_UI_STEP = 20      # ACCDATA_3, 5Hz
  BUTTONS_STEP = 5      # Steering_Data_FD1, 10Hz, but send twice as fast

  CURVATURE_MAX = 0.02  # Max curvature for steering command, m^-1
  STEER_DRIVER_ALLOWANCE = 1.0  # Driver intervention threshold, Nm

  # Curvature rate limits
  # The curvature signal is limited to 0.003 to 0.009 m^-1/sec by the EPS depending on speed and direction
  # Limit to ~2 m/s^3 up, ~3 m/s^3 down at 75 mph
  # Worst case, the low speed limits will allow 4.3 m/s^3 up, 4.9 m/s^3 down at 75 mph
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.0002, 0.0001])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.000225, 0.00015])
  CURVATURE_ERROR = 0.002  # ~6 degrees at 10 m/s, ~10 degrees at 35 m/s

  ACCEL_MAX = 2.0               # m/s^2 max acceleration
  ACCEL_MIN = -3.5              # m/s^2 max deceleration
  MIN_GAS = -0.5
  INACTIVE_GAS = -5.0

  def __init__(self, CP):
    pass


class CAR(StrEnum):
  BRONCO_SPORT_MK1 = "FORD BRONCO SPORT 1ST GEN"
  ESCAPE_MK4 = "FORD ESCAPE 4TH GEN"
  EXPLORER_MK6 = "FORD EXPLORER 6TH GEN"
  F_150_MK14 = "FORD F-150 14TH GEN"
  FOCUS_MK4 = "FORD FOCUS 4TH GEN"
  MAVERICK_MK1 = "FORD MAVERICK 1ST GEN"


CANFD_CAR = {CAR.F_150_MK14}


class RADAR:
  DELPHI_ESR = 'ford_fusion_2018_adas'
  DELPHI_MRR = 'FORD_CADS'


DBC: Dict[str, Dict[str, str]] = defaultdict(lambda: dbc_dict("ford_lincoln_base_pt", RADAR.DELPHI_MRR))

# F-150 radar is not yet supported
#DBC[CAR.F_150_MK14] = dbc_dict("ford_lincoln_base_pt", None)


class Footnote(Enum):
  FOCUS = CarFootnote(
    "Refers only to the Focus Mk4 (C519) available in Europe/China/Taiwan/Australasia, not the Focus Mk3 (C346) in " +
    "North and South America/Southeast Asia.",
    Column.MODEL,
  )


@dataclass
class FordCarInfo(CarInfo):
  package: str = "Co-Pilot360 Assist+"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.ford_q3]))

  def init_make(self, CP: car.CarParams):
    if CP.carFingerprint in (CAR.BRONCO_SPORT_MK1, CAR.MAVERICK_MK1):
      self.car_parts = CarParts([Device.threex_angled_mount, CarHarness.ford_q3])


CAR_INFO: Dict[str, Union[CarInfo, List[CarInfo]]] = {
  CAR.BRONCO_SPORT_MK1: FordCarInfo("Ford Bronco Sport 2021-22"),
  CAR.ESCAPE_MK4: [
    FordCarInfo("Ford Escape 2020-22"),
    FordCarInfo("Ford Kuga 2020-22", "Adaptive Cruise Control with Lane Centering"),
  ],
  CAR.EXPLORER_MK6: [
    FordCarInfo("Ford Explorer 2020-22"),
    FordCarInfo("Lincoln Aviator 2020-21", "Co-Pilot360 Plus"),
  ],
  CAR.F_150_MK14: FordCarInfo("Ford F-150 2023", "Co-Pilot360 Active 2.0"),
  CAR.FOCUS_MK4: FordCarInfo("Ford Focus 2018", "Adaptive Cruise Control with Lane Centering", footnotes=[Footnote.FOCUS]),
  CAR.MAVERICK_MK1: [
    FordCarInfo("Ford Maverick 2022", "LARIAT Luxury"),
    FordCarInfo("Ford Maverick 2023", "Co-Pilot360 Assist"),
  ],
}

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # CAN and CAN FD queries are combined.
    # FIXME: For CAN FD, ECUs respond with frames larger than 8 bytes on the powertrain bus
    # TODO: properly handle auxiliary requests to separate queries and add back whitelists
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      # whitelist_ecus=[Ecu.engine],
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      # whitelist_ecus=[Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.shiftByWire],
      bus=0,
      auxiliary=True,
    ),
  ],
  extra_ecus=[
    (Ecu.shiftByWire, 0x732, None),
  ],
)

FW_VERSIONS = {
  CAR.BRONCO_SPORT_MK1: {
    (Ecu.eps, 0x730, None): [
      b'LX6C-14D003-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-14D003-AK\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'LX6C-2D053-RD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-RE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'M1PT-14F397-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'M1PA-14C204-GF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'N1PA-14C204-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'N1PA-14C204-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.ESCAPE_MK4: {
    (Ecu.eps, 0x730, None): [
      b'LX6C-14D003-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-14D003-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'LX6C-2D053-NS\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-NT\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-NY\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-SA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-SD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'LJ6T-14F397-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LJ6T-14F397-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LV4T-14F397-GG\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'LX6A-14C204-BJV\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6A-14C204-BJX\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6A-14C204-CNG\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6A-14C204-DPK\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6A-14C204-ESG\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'MX6A-14C204-BEF\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'MX6A-14C204-BEJ\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'MX6A-14C204-CAB\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NX6A-14C204-BLE\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.EXPLORER_MK6: {
    (Ecu.eps, 0x730, None): [
      b'L1MC-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'L1MC-14D003-AK\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'L1MC-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'M1MC-14D003-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'M1MC-14D003-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'L1MC-2D053-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'L1MC-2D053-BA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'L1MC-2D053-BB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'L1MC-2D053-BF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'L1MC-2D053-KB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'LB5T-14F397-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LB5T-14F397-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LB5T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LC5T-14F397-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'LB5A-14C204-ATJ\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LB5A-14C204-AUJ\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LB5A-14C204-AZL\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LB5A-14C204-BUJ\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LB5A-14C204-EAC\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'MB5A-14C204-MD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'MB5A-14C204-RC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NB5A-14C204-AZD\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NB5A-14C204-HB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.F_150_MK14: {
    [
    #2023 F150 4X4 5.0L (Comma3 + Red Panda)
    {
	    369: 8, 512: 8, 514: 8, 516: 8, 517: 8, 560: 8, 130: 8, 126: 8, 972: 8, 464740368: 8, 818: 8, 524: 8, 819: 8, 535: 8, 806: 8, 65: 8, 66: 8, 76: 8, 145: 8, 146: 8, 71: 8, 359: 8, 1200: 8, 119: 8, 73: 8, 125: 8, 531: 8, 532: 8, 534: 8, 1045: 8, 1150: 8, 391: 8, 390: 8, 394: 8, 936: 8, 937: 8, 943: 8, 976: 8, 934: 8, 981: 8, 935: 8, 982: 8, 938: 8, 1010: 8, 939: 8, 942: 8, 983: 8, 1113: 8, 1233: 8, 1234: 8, 1237: 8, 357: 8, 92: 8, 342: 8, 358: 8, 374: 8, 376: 8, 377: 8, 380: 8, 515: 8, 1055: 8, 1057: 8, 1060: 8, 1069: 8, 1071: 8, 1085: 8, 1086: 8, 1042: 8, 970: 8, 1100: 8, 578: 8, 1067: 8, 1111: 8, 1068: 8, 932: 8, 1034: 8, 1070: 8, 1093: 8, 1146: 8, 1072: 8, 778: 8, 792: 8, 1076: 8, 931: 8, 985: 8, 611: 8, 1186: 8, 131: 8, 1044: 8, 1046: 8, 879: 8, 954: 8, 90: 8, 1429: 8, 132: 8, 878: 8, 382: 8, 1090: 8, 1087: 8, 909: 8, 953: 8, 1003: 8, 1084: 8, 994: 8, 949: 8, 1438: 8, 1239: 8, 824: 8, 550: 8, 810: 8, 1160: 8, 1245: 8, 563: 8, 530: 8, 639: 8, 1242: 8, 1116: 8, 997: 8, 947: 8, 963: 8, 1049: 8, 811: 8, 551: 8, 651: 8, 529: 8, 1009: 8, 533: 8, 739: 8, 986: 8, 402: 8, 129: 8, 1043: 8, 360: 8, 817: 8, 1120: 8, 1121: 8, 1054: 8, 1056: 8, 1441: 8, 118: 8, 1091: 8, 140: 8, 395: 8, 549: 8, 765: 8, 791: 8, 968: 8, 992: 8, 1114: 8, 912: 8, 874: 8, 1006: 8, 1106: 8, 1108: 8, 1461: 8, 1047: 8, 862: 8, 1050: 8, 1235: 8, 1236: 8, 393: 64, 929: 8, 930: 8, 973: 8, 984: 8, 1105: 8, 1229: 8, 1238: 8, 1248: 8, 1249: 8, 1250: 8, 1251: 8, 1252: 8, 1253: 8, 1254: 8, 1255: 8, 1436: 8, 1122: 8, 1123: 8, 1124: 8, 820: 8, 1430: 8, 1840: 8, 1848: 24}, 5: {33: 16, 34: 16, 35: 8, 256: 8, 257: 32, 261: 8, 264: 32, 265: 12, 266: 64, 288: 64, 289: 64, 290: 64, 291: 64, 292: 64, 293: 64, 294: 64, 295: 64, 296: 64, 297: 64, 298: 64, 299: 64, 300: 64, 301: 64, 9: 12, 21: 8, 22: 8, 24: 8, 515: 8, 1273: 20, 1275: 8, 302: 64, 303: 64, 304: 64, 305: 64, 32: 7, 306: 64, 513: 8, 307: 64, 308: 64, 309: 24, 416: 8, 417: 8, 418: 8, 419: 8, 420: 16, 421: 8, 4: 8, 271: 16, 512: 8, 5: 20, 8: 8, 673: 8}, 6: {369: 8, 512: 8, 514: 8, 516: 8, 517: 8, 560: 8, 130: 8, 126: 8, 972: 8, 464740368: 8, 818: 8, 524: 8, 819: 8, 535: 8, 806: 8, 65: 8, 66: 8, 76: 8, 145: 8, 146: 8, 71: 8, 359: 8, 1200: 8, 119: 8, 73: 8, 125: 8, 531: 8, 532: 8, 534: 8, 1045: 8, 1150: 8, 391: 8, 390: 8, 394: 8, 936: 8, 937: 8, 943: 8, 976: 8, 934: 8, 981: 8, 935: 8, 982: 8, 938: 8, 1010: 8, 939: 8, 942: 8, 983: 8, 1113: 8, 1233: 8, 1234: 8, 1237: 8, 357: 8, 92: 8, 342: 8, 358: 8, 374: 8, 376: 8, 377: 8, 380: 8, 515: 8, 1055: 8, 1057: 8, 1060: 8, 1069: 8, 1071: 8, 1085: 8, 1086: 8, 1042: 8, 970: 8, 1100: 8, 578: 8, 1067: 8, 1111: 8, 1068: 8, 932: 8, 1034: 8, 1070: 8, 1093: 8, 1146: 8, 1072: 8, 778: 8, 792: 8, 1076: 8, 931: 8, 985: 8, 611: 8, 1186: 8, 131: 8, 1044: 8, 1046: 8, 879: 8, 954: 8, 90: 8, 1429: 8, 132: 8, 878: 8, 382: 8, 1090: 8, 1087: 8, 909: 8, 953: 8, 1003: 8, 1084: 8, 994: 8, 949: 8, 1438: 8, 1239: 8, 824: 8, 550: 8, 810: 8, 1160: 8, 1245: 8, 563: 8, 530: 8, 639: 8, 1242: 8, 1116: 8, 997: 8, 947: 8, 963: 8, 1049: 8, 811: 8, 551: 8, 651: 8, 529: 8, 1009: 8, 533: 8, 739: 8, 986: 8, 402: 8, 129: 8, 1043: 8, 360: 8, 817: 8, 1120: 8, 1121: 8, 1054: 8, 1056: 8, 1441: 8, 118: 8, 1091: 8, 140: 8, 395: 8, 549: 8, 765: 8, 791: 8, 968: 8, 992: 8, 1114: 8, 912: 8, 874: 8, 1006: 8, 1106: 8, 1108: 8, 1461: 8, 1047: 8, 862: 8, 1050: 8, 1235: 8, 1236: 8, 393: 64, 929: 8, 930: 8, 973: 8, 984: 8, 1105: 8, 1229: 8, 1238: 8, 1248: 8, 1249: 8, 1250: 8, 1251: 8, 1252: 8, 1253: 8, 1254: 8, 1255: 8, 1436: 8, 1122: 8, 1123: 8, 1124: 8, 820: 8, 1430: 8, 1840: 8, 1848: 24
    }],
    (Ecu.eps, 0x730, None): [
      b'ML3V-14D003-BC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'PL34-2D053-CA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'ML3T-14D049-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'PJ6T-14H102-ABJ\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'PL3A-14C204-BRB\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FOCUS_MK4: {
    (Ecu.eps, 0x730, None): [
      b'JX6C-14D003-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'JX61-2D053-CJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'JX7T-14D049-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'JX7T-14F397-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'JX6A-14C204-BPL\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.MAVERICK_MK1: {
    (Ecu.eps, 0x730, None): [
      b'NZ6C-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'NZ6C-2D053-AG\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PZ6C-2D053-ED\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'NZ6T-14D049-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'NZ6T-14F397-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'NZ6A-14C204-AAA\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NZ6A-14C204-PA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NZ6A-14C204-ZA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PZ6A-14C204-BE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PZ6A-14C204-JC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PZ6A-14C204-JE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}
