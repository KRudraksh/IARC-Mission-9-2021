
"use strict";

let AttitudeTarget = require('./AttitudeTarget.js');
let HilSensor = require('./HilSensor.js');
let GPSRTK = require('./GPSRTK.js');
let HilControls = require('./HilControls.js');
let State = require('./State.js');
let Param = require('./Param.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let WaypointList = require('./WaypointList.js');
let WaypointReached = require('./WaypointReached.js');
let ESCStatus = require('./ESCStatus.js');
let ManualControl = require('./ManualControl.js');
let LandingTarget = require('./LandingTarget.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let ExtendedState = require('./ExtendedState.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let VehicleInfo = require('./VehicleInfo.js');
let StatusText = require('./StatusText.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let FileEntry = require('./FileEntry.js');
let Waypoint = require('./Waypoint.js');
let Vibration = require('./Vibration.js');
let ESCInfo = require('./ESCInfo.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let VFR_HUD = require('./VFR_HUD.js');
let Mavlink = require('./Mavlink.js');
let PositionTarget = require('./PositionTarget.js');
let BatteryStatus = require('./BatteryStatus.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let CommandCode = require('./CommandCode.js');
let RTCM = require('./RTCM.js');
let Trajectory = require('./Trajectory.js');
let Altitude = require('./Altitude.js');
let LogData = require('./LogData.js');
let MountControl = require('./MountControl.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let RCIn = require('./RCIn.js');
let GPSRAW = require('./GPSRAW.js');
let DebugValue = require('./DebugValue.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let ActuatorControl = require('./ActuatorControl.js');
let Thrust = require('./Thrust.js');
let HilGPS = require('./HilGPS.js');
let RCOut = require('./RCOut.js');
let RadioStatus = require('./RadioStatus.js');
let RTKBaseline = require('./RTKBaseline.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let LogEntry = require('./LogEntry.js');
let HomePosition = require('./HomePosition.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let ParamValue = require('./ParamValue.js');
let CamIMUStamp = require('./CamIMUStamp.js');

module.exports = {
  AttitudeTarget: AttitudeTarget,
  HilSensor: HilSensor,
  GPSRTK: GPSRTK,
  HilControls: HilControls,
  State: State,
  Param: Param,
  OverrideRCIn: OverrideRCIn,
  ADSBVehicle: ADSBVehicle,
  WaypointList: WaypointList,
  WaypointReached: WaypointReached,
  ESCStatus: ESCStatus,
  ManualControl: ManualControl,
  LandingTarget: LandingTarget,
  OpticalFlowRad: OpticalFlowRad,
  ESCStatusItem: ESCStatusItem,
  CompanionProcessStatus: CompanionProcessStatus,
  ExtendedState: ExtendedState,
  TimesyncStatus: TimesyncStatus,
  VehicleInfo: VehicleInfo,
  StatusText: StatusText,
  ESCInfoItem: ESCInfoItem,
  FileEntry: FileEntry,
  Waypoint: Waypoint,
  Vibration: Vibration,
  ESCInfo: ESCInfo,
  OnboardComputerStatus: OnboardComputerStatus,
  VFR_HUD: VFR_HUD,
  Mavlink: Mavlink,
  PositionTarget: PositionTarget,
  BatteryStatus: BatteryStatus,
  WheelOdomStamped: WheelOdomStamped,
  PlayTuneV2: PlayTuneV2,
  CommandCode: CommandCode,
  RTCM: RTCM,
  Trajectory: Trajectory,
  Altitude: Altitude,
  LogData: LogData,
  MountControl: MountControl,
  EstimatorStatus: EstimatorStatus,
  RCIn: RCIn,
  GPSRAW: GPSRAW,
  DebugValue: DebugValue,
  HilStateQuaternion: HilStateQuaternion,
  ActuatorControl: ActuatorControl,
  Thrust: Thrust,
  HilGPS: HilGPS,
  RCOut: RCOut,
  RadioStatus: RadioStatus,
  RTKBaseline: RTKBaseline,
  HilActuatorControls: HilActuatorControls,
  LogEntry: LogEntry,
  HomePosition: HomePosition,
  GlobalPositionTarget: GlobalPositionTarget,
  ParamValue: ParamValue,
  CamIMUStamp: CamIMUStamp,
};
