
"use strict";

let KortexError = require('./KortexError.js');
let ApiOptions = require('./ApiOptions.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ErrorCodes = require('./ErrorCodes.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let StepResponse = require('./StepResponse.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let AxisOffsets = require('./AxisOffsets.js');
let TorqueOffset = require('./TorqueOffset.js');
let PositionCommand = require('./PositionCommand.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let LoopSelection = require('./LoopSelection.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let ControlLoop = require('./ControlLoop.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let Servoing = require('./Servoing.js');
let AxisPosition = require('./AxisPosition.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let CommandMode = require('./CommandMode.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let RampResponse = require('./RampResponse.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let StatusFlags = require('./StatusFlags.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let SequenceInformation = require('./SequenceInformation.js');
let UserEvent = require('./UserEvent.js');
let TransformationRow = require('./TransformationRow.js');
let SystemTime = require('./SystemTime.js');
let SequenceHandle = require('./SequenceHandle.js');
let ControllerNotification = require('./ControllerNotification.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let ChangeTwist = require('./ChangeTwist.js');
let ServoingMode = require('./ServoingMode.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let ActionNotification = require('./ActionNotification.js');
let WaypointList = require('./WaypointList.js');
let JointLimitation = require('./JointLimitation.js');
let JointAngles = require('./JointAngles.js');
let Timeout = require('./Timeout.js');
let JointTorques = require('./JointTorques.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let SoundType = require('./SoundType.js');
let NavigationDirection = require('./NavigationDirection.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let IPv4Information = require('./IPv4Information.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let ActionEvent = require('./ActionEvent.js');
let UserNotificationList = require('./UserNotificationList.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let Twist = require('./Twist.js');
let BridgeList = require('./BridgeList.js');
let Finger = require('./Finger.js');
let ControllerInputType = require('./ControllerInputType.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let GpioBehavior = require('./GpioBehavior.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let JointAngle = require('./JointAngle.js');
let TwistLimitation = require('./TwistLimitation.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let WifiInformation = require('./WifiInformation.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let LimitationType = require('./LimitationType.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let SignalQuality = require('./SignalQuality.js');
let ControllerList = require('./ControllerList.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let IKData = require('./IKData.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let Mapping = require('./Mapping.js');
let BridgeResult = require('./BridgeResult.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let JointTorque = require('./JointTorque.js');
let NetworkNotification = require('./NetworkNotification.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let ControllerType = require('./ControllerType.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let SequenceTasks = require('./SequenceTasks.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let MapHandle = require('./MapHandle.js');
let Gripper = require('./Gripper.js');
let GripperMode = require('./GripperMode.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let GripperRequest = require('./GripperRequest.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let JointSpeed = require('./JointSpeed.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let Map = require('./Map.js');
let BackupEvent = require('./BackupEvent.js');
let MapEvent_events = require('./MapEvent_events.js');
let Waypoint = require('./Waypoint.js');
let MapElement = require('./MapElement.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let ControllerEvent = require('./ControllerEvent.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let PasswordChange = require('./PasswordChange.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let Query = require('./Query.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let Point = require('./Point.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let ActionList = require('./ActionList.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let EmergencyStop = require('./EmergencyStop.js');
let MapEvent = require('./MapEvent.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let BridgeType = require('./BridgeType.js');
let WifiInformationList = require('./WifiInformationList.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let GripperCommand = require('./GripperCommand.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let Wrench = require('./Wrench.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let Admittance = require('./Admittance.js');
let SequenceTask = require('./SequenceTask.js');
let OperatingMode = require('./OperatingMode.js');
let Pose = require('./Pose.js');
let ControllerHandle = require('./ControllerHandle.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let RequestedActionType = require('./RequestedActionType.js');
let UserProfileList = require('./UserProfileList.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let ActionHandle = require('./ActionHandle.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let SafetyEvent = require('./SafetyEvent.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let NetworkType = require('./NetworkType.js');
let ControllerEventType = require('./ControllerEventType.js');
let WrenchMode = require('./WrenchMode.js');
let Delay = require('./Delay.js');
let ActionType = require('./ActionType.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let Sequence = require('./Sequence.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let SequenceList = require('./SequenceList.js');
let FactoryNotification = require('./FactoryNotification.js');
let FullUserProfile = require('./FullUserProfile.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let MapGroup = require('./MapGroup.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let UserProfile = require('./UserProfile.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let WrenchCommand = require('./WrenchCommand.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let MapGroupList = require('./MapGroupList.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let LedState = require('./LedState.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let FactoryEvent = require('./FactoryEvent.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let ShapeType = require('./ShapeType.js');
let Orientation = require('./Orientation.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let NetworkHandle = require('./NetworkHandle.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let ControllerState = require('./ControllerState.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let GpioAction = require('./GpioAction.js');
let Action = require('./Action.js');
let SnapshotType = require('./SnapshotType.js');
let ControllerElementState = require('./ControllerElementState.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let UserList = require('./UserList.js');
let ProtectionZone = require('./ProtectionZone.js');
let BridgeStatus = require('./BridgeStatus.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let ZoneShape = require('./ZoneShape.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let RobotEvent = require('./RobotEvent.js');
let Base_Position = require('./Base_Position.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let Base_Stop = require('./Base_Stop.js');
let BridgeConfig = require('./BridgeConfig.js');
let ChangeWrench = require('./ChangeWrench.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let UserNotification = require('./UserNotification.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let Snapshot = require('./Snapshot.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let GpioCommand = require('./GpioCommand.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let GpioEvent = require('./GpioEvent.js');
let MappingHandle = require('./MappingHandle.js');
let MapList = require('./MapList.js');
let TwistCommand = require('./TwistCommand.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let NetworkEvent = require('./NetworkEvent.js');
let Faults = require('./Faults.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let MappingList = require('./MappingList.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let Ssid = require('./Ssid.js');
let RFConfiguration = require('./RFConfiguration.js');
let WifiEnableState = require('./WifiEnableState.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let BaseFeedback = require('./BaseFeedback.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let UARTParity = require('./UARTParity.js');
let Empty = require('./Empty.js');
let Unit = require('./Unit.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let NotificationOptions = require('./NotificationOptions.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let DeviceHandle = require('./DeviceHandle.js');
let DeviceTypes = require('./DeviceTypes.js');
let UARTSpeed = require('./UARTSpeed.js');
let Timestamp = require('./Timestamp.js');
let NotificationType = require('./NotificationType.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let SafetyHandle = require('./SafetyHandle.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let UARTWordLength = require('./UARTWordLength.js');
let Permission = require('./Permission.js');
let Connection = require('./Connection.js');
let SafetyNotification = require('./SafetyNotification.js');
let NotificationHandle = require('./NotificationHandle.js');
let ArmState = require('./ArmState.js');
let CountryCode = require('./CountryCode.js');
let UARTStopBits = require('./UARTStopBits.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let AngularTwist = require('./AngularTwist.js');
let GravityVector = require('./GravityVector.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let LinearTwist = require('./LinearTwist.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let KinematicLimits = require('./KinematicLimits.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let CartesianTransform = require('./CartesianTransform.js');
let PayloadInformation = require('./PayloadInformation.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let SafetyInformation = require('./SafetyInformation.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let RunMode = require('./RunMode.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let SafetyEnable = require('./SafetyEnable.js');
let RunModes = require('./RunModes.js');
let CalibrationResult = require('./CalibrationResult.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let SafetyStatus = require('./SafetyStatus.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let CalibrationElement = require('./CalibrationElement.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let Calibration = require('./Calibration.js');
let PartNumber = require('./PartNumber.js');
let IPv4Settings = require('./IPv4Settings.js');
let CalibrationItem = require('./CalibrationItem.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let MACAddress = require('./MACAddress.js');
let ModelNumber = require('./ModelNumber.js');
let DeviceType = require('./DeviceType.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let SerialNumber = require('./SerialNumber.js');
let RebootRqst = require('./RebootRqst.js');
let DeviceHandles = require('./DeviceHandles.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let MotorFeedback = require('./MotorFeedback.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let MotorCommand = require('./MotorCommand.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let GPIOState = require('./GPIOState.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CMode = require('./I2CMode.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let UARTPortId = require('./UARTPortId.js');
let I2CDevice = require('./I2CDevice.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let GPIOMode = require('./GPIOMode.js');
let GPIOValue = require('./GPIOValue.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let GPIOPull = require('./GPIOPull.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let EthernetDevice = require('./EthernetDevice.js');
let I2CData = require('./I2CData.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let ArmLaterality = require('./ArmLaterality.js');
let VisionModuleType = require('./VisionModuleType.js');
let ModelId = require('./ModelId.js');
let WristType = require('./WristType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let BrakeType = require('./BrakeType.js');
let EndEffectorType = require('./EndEffectorType.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let BaseType = require('./BaseType.js');
let FocusPoint = require('./FocusPoint.js');
let SensorSettings = require('./SensorSettings.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let BitRate = require('./BitRate.js');
let Resolution = require('./Resolution.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let Sensor = require('./Sensor.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let OptionInformation = require('./OptionInformation.js');
let FocusAction = require('./FocusAction.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let ManualFocus = require('./ManualFocus.js');
let OptionValue = require('./OptionValue.js');
let Option = require('./Option.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let FrameRate = require('./FrameRate.js');
let VisionNotification = require('./VisionNotification.js');
let VisionEvent = require('./VisionEvent.js');
let TranslationVector = require('./TranslationVector.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');

module.exports = {
  KortexError: KortexError,
  ApiOptions: ApiOptions,
  SubErrorCodes: SubErrorCodes,
  ErrorCodes: ErrorCodes,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  StepResponse: StepResponse,
  VectorDriveParameters: VectorDriveParameters,
  AxisOffsets: AxisOffsets,
  TorqueOffset: TorqueOffset,
  PositionCommand: PositionCommand,
  CustomDataSelection: CustomDataSelection,
  CommandModeInformation: CommandModeInformation,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  TorqueCalibration: TorqueCalibration,
  ControlLoopSelection: ControlLoopSelection,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  CustomDataIndex: CustomDataIndex,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  LoopSelection: LoopSelection,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  ControlLoop: ControlLoop,
  FrequencyResponse: FrequencyResponse,
  Servoing: Servoing,
  AxisPosition: AxisPosition,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  CommandMode: CommandMode,
  ControlLoopParameters: ControlLoopParameters,
  RampResponse: RampResponse,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  CommandFlags: CommandFlags,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  StatusFlags: StatusFlags,
  SequenceTaskHandle: SequenceTaskHandle,
  Gen3GpioPinId: Gen3GpioPinId,
  WifiSecurityType: WifiSecurityType,
  SequenceInformation: SequenceInformation,
  UserEvent: UserEvent,
  TransformationRow: TransformationRow,
  SystemTime: SystemTime,
  SequenceHandle: SequenceHandle,
  ControllerNotification: ControllerNotification,
  CartesianWaypoint: CartesianWaypoint,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  BridgeIdentifier: BridgeIdentifier,
  MappingInfoNotificationList: MappingInfoNotificationList,
  ChangeTwist: ChangeTwist,
  ServoingMode: ServoingMode,
  ArmStateInformation: ArmStateInformation,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  ActionNotification: ActionNotification,
  WaypointList: WaypointList,
  JointLimitation: JointLimitation,
  JointAngles: JointAngles,
  Timeout: Timeout,
  JointTorques: JointTorques,
  ControllerConfiguration: ControllerConfiguration,
  SoundType: SoundType,
  NavigationDirection: NavigationDirection,
  ProtectionZoneInformation: ProtectionZoneInformation,
  ControllerNotificationList: ControllerNotificationList,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  IPv4Information: IPv4Information,
  SwitchControlMapping: SwitchControlMapping,
  ActionEvent: ActionEvent,
  UserNotificationList: UserNotificationList,
  IPv4Configuration: IPv4Configuration,
  Twist: Twist,
  BridgeList: BridgeList,
  Finger: Finger,
  ControllerInputType: ControllerInputType,
  OperatingModeNotification: OperatingModeNotification,
  ActionExecutionState: ActionExecutionState,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  TrajectoryInfoType: TrajectoryInfoType,
  GpioBehavior: GpioBehavior,
  JointNavigationDirection: JointNavigationDirection,
  JointAngle: JointAngle,
  TwistLimitation: TwistLimitation,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  WifiInformation: WifiInformation,
  ControllerConfigurationMode: ControllerConfigurationMode,
  LimitationType: LimitationType,
  ControllerBehavior: ControllerBehavior,
  SignalQuality: SignalQuality,
  ControllerList: ControllerList,
  ProtectionZoneHandle: ProtectionZoneHandle,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  IKData: IKData,
  OperatingModeInformation: OperatingModeInformation,
  Mapping: Mapping,
  BridgeResult: BridgeResult,
  ConstrainedJointAngles: ConstrainedJointAngles,
  JointTorque: JointTorque,
  NetworkNotification: NetworkNotification,
  TransformationMatrix: TransformationMatrix,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  ControllerType: ControllerType,
  MapGroupHandle: MapGroupHandle,
  AngularWaypoint: AngularWaypoint,
  TrajectoryErrorElement: TrajectoryErrorElement,
  SequenceTasks: SequenceTasks,
  NetworkNotificationList: NetworkNotificationList,
  MapHandle: MapHandle,
  Gripper: Gripper,
  GripperMode: GripperMode,
  ProtectionZoneEvent: ProtectionZoneEvent,
  GripperRequest: GripperRequest,
  TrajectoryErrorReport: TrajectoryErrorReport,
  JointSpeed: JointSpeed,
  ServoingModeNotification: ServoingModeNotification,
  Map: Map,
  BackupEvent: BackupEvent,
  MapEvent_events: MapEvent_events,
  Waypoint: Waypoint,
  MapElement: MapElement,
  ConstrainedPose: ConstrainedPose,
  ActuatorInformation: ActuatorInformation,
  BluetoothEnableState: BluetoothEnableState,
  WifiConfigurationList: WifiConfigurationList,
  ControllerEvent: ControllerEvent,
  ProtectionZoneList: ProtectionZoneList,
  SafetyNotificationList: SafetyNotificationList,
  PasswordChange: PasswordChange,
  CartesianLimitationList: CartesianLimitationList,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  Query: Query,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  Point: Point,
  WrenchLimitation: WrenchLimitation,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  ActionList: ActionList,
  WaypointValidationReport: WaypointValidationReport,
  ProtectionZoneNotification: ProtectionZoneNotification,
  ActivateMapHandle: ActivateMapHandle,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  EmergencyStop: EmergencyStop,
  MapEvent: MapEvent,
  WifiConfiguration: WifiConfiguration,
  BridgeType: BridgeType,
  WifiInformationList: WifiInformationList,
  Base_CapSenseMode: Base_CapSenseMode,
  GripperCommand: GripperCommand,
  AppendActionInformation: AppendActionInformation,
  ControllerConfigurationList: ControllerConfigurationList,
  Wrench: Wrench,
  WifiEncryptionType: WifiEncryptionType,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  Admittance: Admittance,
  SequenceTask: SequenceTask,
  OperatingMode: OperatingMode,
  Pose: Pose,
  ControllerHandle: ControllerHandle,
  ConstrainedJointAngle: ConstrainedJointAngle,
  SequenceTasksRange: SequenceTasksRange,
  RequestedActionType: RequestedActionType,
  UserProfileList: UserProfileList,
  Base_JointSpeeds: Base_JointSpeeds,
  ActionHandle: ActionHandle,
  SequenceInfoNotification: SequenceInfoNotification,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  SafetyEvent: SafetyEvent,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  NetworkType: NetworkType,
  ControllerEventType: ControllerEventType,
  WrenchMode: WrenchMode,
  Delay: Delay,
  ActionType: ActionType,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  Sequence: Sequence,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  ControllerElementHandle: ControllerElementHandle,
  OperatingModeNotificationList: OperatingModeNotificationList,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  SequenceList: SequenceList,
  FactoryNotification: FactoryNotification,
  FullUserProfile: FullUserProfile,
  Base_ControlModeNotification: Base_ControlModeNotification,
  ControllerElementEventType: ControllerElementEventType,
  MapGroup: MapGroup,
  ArmStateNotification: ArmStateNotification,
  ConstrainedOrientation: ConstrainedOrientation,
  ChangeJointSpeeds: ChangeJointSpeeds,
  ControlModeNotificationList: ControlModeNotificationList,
  UserProfile: UserProfile,
  FullIPv4Configuration: FullIPv4Configuration,
  WrenchCommand: WrenchCommand,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  MapGroupList: MapGroupList,
  Base_ServiceVersion: Base_ServiceVersion,
  Base_CapSenseConfig: Base_CapSenseConfig,
  LedState: LedState,
  TrajectoryErrorType: TrajectoryErrorType,
  FactoryEvent: FactoryEvent,
  AdmittanceMode: AdmittanceMode,
  ShapeType: ShapeType,
  Orientation: Orientation,
  GpioConfigurationList: GpioConfigurationList,
  Base_ControlModeInformation: Base_ControlModeInformation,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  ControllerNotification_state: ControllerNotification_state,
  NetworkHandle: NetworkHandle,
  ServoingModeNotificationList: ServoingModeNotificationList,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  ControllerState: ControllerState,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  GpioAction: GpioAction,
  Action: Action,
  SnapshotType: SnapshotType,
  ControllerElementState: ControllerElementState,
  TrajectoryInfo: TrajectoryInfo,
  UserList: UserList,
  ProtectionZone: ProtectionZone,
  BridgeStatus: BridgeStatus,
  Action_action_parameters: Action_action_parameters,
  Base_ControlMode: Base_ControlMode,
  ZoneShape: ZoneShape,
  Base_GpioConfiguration: Base_GpioConfiguration,
  RobotEvent: RobotEvent,
  Base_Position: Base_Position,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  GpioPinConfiguration: GpioPinConfiguration,
  RobotEventNotification: RobotEventNotification,
  FirmwareComponentVersion: FirmwareComponentVersion,
  Base_RotationMatrix: Base_RotationMatrix,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  CartesianSpeed: CartesianSpeed,
  Base_Stop: Base_Stop,
  BridgeConfig: BridgeConfig,
  ChangeWrench: ChangeWrench,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  FirmwareBundleVersions: FirmwareBundleVersions,
  UserNotification: UserNotification,
  ServoingModeInformation: ServoingModeInformation,
  JointsLimitationsList: JointsLimitationsList,
  ActionNotificationList: ActionNotificationList,
  Snapshot: Snapshot,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  CartesianLimitation: CartesianLimitation,
  GpioCommand: GpioCommand,
  MappingInfoNotification: MappingInfoNotification,
  GpioEvent: GpioEvent,
  MappingHandle: MappingHandle,
  MapList: MapList,
  TwistCommand: TwistCommand,
  SequenceTasksPair: SequenceTasksPair,
  NetworkEvent: NetworkEvent,
  Faults: Faults,
  ConstrainedPosition: ConstrainedPosition,
  MappingList: MappingList,
  RobotEventNotificationList: RobotEventNotificationList,
  Ssid: Ssid,
  RFConfiguration: RFConfiguration,
  WifiEnableState: WifiEnableState,
  BridgePortConfig: BridgePortConfig,
  BaseFeedback: BaseFeedback,
  ActuatorCustomData: ActuatorCustomData,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  BaseCyclic_Command: BaseCyclic_Command,
  ActuatorCommand: ActuatorCommand,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  UARTDeviceIdentification: UARTDeviceIdentification,
  UARTParity: UARTParity,
  Empty: Empty,
  Unit: Unit,
  SafetyStatusValue: SafetyStatusValue,
  NotificationOptions: NotificationOptions,
  CountryCodeIdentifier: CountryCodeIdentifier,
  DeviceHandle: DeviceHandle,
  DeviceTypes: DeviceTypes,
  UARTSpeed: UARTSpeed,
  Timestamp: Timestamp,
  NotificationType: NotificationType,
  UserProfileHandle: UserProfileHandle,
  SafetyHandle: SafetyHandle,
  CartesianReferenceFrame: CartesianReferenceFrame,
  UARTWordLength: UARTWordLength,
  Permission: Permission,
  Connection: Connection,
  SafetyNotification: SafetyNotification,
  NotificationHandle: NotificationHandle,
  ArmState: ArmState,
  CountryCode: CountryCode,
  UARTStopBits: UARTStopBits,
  UARTConfiguration: UARTConfiguration,
  AngularTwist: AngularTwist,
  GravityVector: GravityVector,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  LinearTwist: LinearTwist,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  KinematicLimits: KinematicLimits,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  ToolConfiguration: ToolConfiguration,
  KinematicLimitsList: KinematicLimitsList,
  ControlConfigurationNotification: ControlConfigurationNotification,
  CartesianTransform: CartesianTransform,
  PayloadInformation: PayloadInformation,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  ControlConfigurationEvent: ControlConfigurationEvent,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  ControlConfig_Position: ControlConfig_Position,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  DesiredSpeeds: DesiredSpeeds,
  SafetyInformation: SafetyInformation,
  CalibrationStatus: CalibrationStatus,
  RunMode: RunMode,
  BootloaderVersion: BootloaderVersion,
  FirmwareVersion: FirmwareVersion,
  CapSenseRegister: CapSenseRegister,
  SafetyInformationList: SafetyInformationList,
  SafetyConfiguration: SafetyConfiguration,
  SafetyEnable: SafetyEnable,
  RunModes: RunModes,
  CalibrationResult: CalibrationResult,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  SafetyStatus: SafetyStatus,
  SafetyConfigurationList: SafetyConfigurationList,
  CalibrationElement: CalibrationElement,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  Calibration: Calibration,
  PartNumber: PartNumber,
  IPv4Settings: IPv4Settings,
  CalibrationItem: CalibrationItem,
  CalibrationParameter_value: CalibrationParameter_value,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  PartNumberRevision: PartNumberRevision,
  SafetyThreshold: SafetyThreshold,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  CalibrationParameter: CalibrationParameter,
  MACAddress: MACAddress,
  ModelNumber: ModelNumber,
  DeviceType: DeviceType,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  SerialNumber: SerialNumber,
  RebootRqst: RebootRqst,
  DeviceHandles: DeviceHandles,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  MotorFeedback: MotorFeedback,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_Command: GripperCyclic_Command,
  MotorCommand: MotorCommand,
  GPIOIdentification: GPIOIdentification,
  GPIOState: GPIOState,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CMode: I2CMode,
  I2CConfiguration: I2CConfiguration,
  GPIOIdentifier: GPIOIdentifier,
  UARTPortId: UARTPortId,
  I2CDevice: I2CDevice,
  EthernetConfiguration: EthernetConfiguration,
  GPIOMode: GPIOMode,
  GPIOValue: GPIOValue,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  GPIOPull: GPIOPull,
  EthernetDuplex: EthernetDuplex,
  I2CWriteParameter: I2CWriteParameter,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  EthernetSpeed: EthernetSpeed,
  I2CDeviceIdentification: I2CDeviceIdentification,
  EthernetDevice: EthernetDevice,
  I2CData: I2CData,
  I2CReadParameter: I2CReadParameter,
  I2CDeviceAddressing: I2CDeviceAddressing,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  ArmLaterality: ArmLaterality,
  VisionModuleType: VisionModuleType,
  ModelId: ModelId,
  WristType: WristType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  BrakeType: BrakeType,
  EndEffectorType: EndEffectorType,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  InterfaceModuleType: InterfaceModuleType,
  BaseType: BaseType,
  FocusPoint: FocusPoint,
  SensorSettings: SensorSettings,
  SensorFocusAction: SensorFocusAction,
  ExtrinsicParameters: ExtrinsicParameters,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  BitRate: BitRate,
  Resolution: Resolution,
  OptionIdentifier: OptionIdentifier,
  DistortionCoefficients: DistortionCoefficients,
  IntrinsicParameters: IntrinsicParameters,
  Sensor: Sensor,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  OptionInformation: OptionInformation,
  FocusAction: FocusAction,
  SensorIdentifier: SensorIdentifier,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  ManualFocus: ManualFocus,
  OptionValue: OptionValue,
  Option: Option,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  FrameRate: FrameRate,
  VisionNotification: VisionNotification,
  VisionEvent: VisionEvent,
  TranslationVector: TranslationVector,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
};
