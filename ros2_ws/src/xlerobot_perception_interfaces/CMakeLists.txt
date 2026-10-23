cmake_minimum_required(VERSION 3.8)
project(xlerobot_perception_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InstanceMask.msg" "msg/InstanceMaskArray.msg"
  "msg/TrackedObject.msg" "msg/TrackedObjectArray.msg"
  "srv/SetVocabulary.srv" "srv/QueryObjects.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
