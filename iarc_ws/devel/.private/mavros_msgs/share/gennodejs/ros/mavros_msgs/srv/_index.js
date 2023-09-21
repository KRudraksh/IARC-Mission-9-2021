
"use strict";

let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let WaypointClear = require('./WaypointClear.js')
let ParamPull = require('./ParamPull.js')
let WaypointPull = require('./WaypointPull.js')
let FileOpen = require('./FileOpen.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileTruncate = require('./FileTruncate.js')
let FileList = require('./FileList.js')
let ParamPush = require('./ParamPush.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let CommandInt = require('./CommandInt.js')
let CommandLong = require('./CommandLong.js')
let WaypointPush = require('./WaypointPush.js')
let ParamSet = require('./ParamSet.js')
let MessageInterval = require('./MessageInterval.js')
let ParamGet = require('./ParamGet.js')
let CommandBool = require('./CommandBool.js')
let FileRename = require('./FileRename.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let SetMode = require('./SetMode.js')
let FileWrite = require('./FileWrite.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let FileChecksum = require('./FileChecksum.js')
let LogRequestData = require('./LogRequestData.js')
let FileMakeDir = require('./FileMakeDir.js')
let CommandHome = require('./CommandHome.js')
let FileClose = require('./FileClose.js')
let LogRequestList = require('./LogRequestList.js')
let CommandTOL = require('./CommandTOL.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileRemove = require('./FileRemove.js')
let StreamRate = require('./StreamRate.js')
let FileRead = require('./FileRead.js')
let MountConfigure = require('./MountConfigure.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let SetMavFrame = require('./SetMavFrame.js')

module.exports = {
  CommandTriggerInterval: CommandTriggerInterval,
  WaypointClear: WaypointClear,
  ParamPull: ParamPull,
  WaypointPull: WaypointPull,
  FileOpen: FileOpen,
  CommandVtolTransition: CommandVtolTransition,
  FileTruncate: FileTruncate,
  FileList: FileList,
  ParamPush: ParamPush,
  WaypointSetCurrent: WaypointSetCurrent,
  CommandInt: CommandInt,
  CommandLong: CommandLong,
  WaypointPush: WaypointPush,
  ParamSet: ParamSet,
  MessageInterval: MessageInterval,
  ParamGet: ParamGet,
  CommandBool: CommandBool,
  FileRename: FileRename,
  LogRequestEnd: LogRequestEnd,
  SetMode: SetMode,
  FileWrite: FileWrite,
  VehicleInfoGet: VehicleInfoGet,
  FileChecksum: FileChecksum,
  LogRequestData: LogRequestData,
  FileMakeDir: FileMakeDir,
  CommandHome: CommandHome,
  FileClose: FileClose,
  LogRequestList: LogRequestList,
  CommandTOL: CommandTOL,
  FileRemoveDir: FileRemoveDir,
  FileRemove: FileRemove,
  StreamRate: StreamRate,
  FileRead: FileRead,
  MountConfigure: MountConfigure,
  CommandTriggerControl: CommandTriggerControl,
  SetMavFrame: SetMavFrame,
};
