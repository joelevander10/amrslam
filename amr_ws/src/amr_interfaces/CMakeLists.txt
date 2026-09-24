cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelVelocities.msg"
  "msg/WheelStates.msg"
  "msg/DriveStatus.msg"
  "msg/SafetyFieldStatus.msg"
  "msg/StationDetection.msg"
  "msg/MappingState.msg"
  "msg/LocalizationState.msg"
  "msg/MotionPermit.msg"
  "msg/RunState.msg"
  "msg/PanelState.msg"
  "msg/ModeState.msg"
  "msg/ControlLease.msg"
  "msg/MuxState.msg"
  "msg/ManualCommand.msg"
  "msg/OperationState.msg"
  "msg/IoImage.msg"
  "msg/Event.msg"
  "msg/CommissioningState.msg"
  "msg/PpMove.msg"
  "msg/PpStatus.msg"
  "msg/LineTrack.msg"
  "msg/SurveyMoveState.msg"
  "srv/RunMission.srv"
  "srv/StartSurvey.srv"
  "srv/SaveMap.srv"
  "srv/SetPose2D.srv"
  "srv/AddObstacle.srv"
  "srv/RequestMode.srv"
  "srv/RequestSurvey.srv"
  "srv/GetOperation.srv"
  "srv/PlanCommissioning.srv"
  "srv/SurveyMove.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
