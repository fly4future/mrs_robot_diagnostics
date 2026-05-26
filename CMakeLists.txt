cmake_minimum_required(VERSION 3.28)
project(mrs_robot_diagnostics)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_robot_diagnostics]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  tf2_ros
  pluginlib
  diagnostic_msgs
)

set(LIBRARIES
  MrsRobotDiagnostics_MrsRobotDiagnostics
  MrsRobotDiagnostics_StateMonitor
  MrsRobotDiagnostics_SensorPlugins
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                      Shared Library                     |
## --------------------------------------------------------------

# Shared library for the plugin API
add_library(MrsRobotDiagnostics_MrsRobotDiagnostics SHARED
  src/sensor_handler.cpp
)
add_library(mrs_robot_diagnostics::mrs_robot_diagnostics ALIAS MrsRobotDiagnostics_MrsRobotDiagnostics)

set_target_properties(MrsRobotDiagnostics_MrsRobotDiagnostics
  PROPERTIES
    EXPORT_NAME mrs_robot_diagnostics
    OUTPUT_NAME mrs_robot_diagnostics
)

target_sources(MrsRobotDiagnostics_MrsRobotDiagnostics
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_robot_diagnostics/enums/robot_type.h"
    "include/mrs_robot_diagnostics/enums/tracker_state.h"
    "include/mrs_robot_diagnostics/enums/uav_state.h"
    "include/mrs_robot_diagnostics/enums/enum_helpers.h"
    "include/mrs_robot_diagnostics/enums/enum_macros.h"
    "include/mrs_robot_diagnostics/sensor_handler.h"  
    "include/mrs_robot_diagnostics/create_main_subscriber.tpp"
)

target_link_libraries(MrsRobotDiagnostics_MrsRobotDiagnostics
  PUBLIC
    rclcpp::rclcpp
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
    tf2_ros::tf2_ros
    ${diagnostic_msgs_TARGETS}
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

## | ------------ MrsRobotDiagnostics_StateMonitor ------------ |

add_library(MrsRobotDiagnostics_StateMonitor SHARED
  src/state_monitor.cpp
  src/preflight_checker.cpp
  src/utils/flight_timer.cpp
  src/utils/wh_drained_integrator.cpp
  src/utils/host_stats.cpp
)

target_sources(MrsRobotDiagnostics_StateMonitor
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_robot_diagnostics/state_monitor.h"
    "include/mrs_robot_diagnostics/preflight_checker.h"
    "include/mrs_robot_diagnostics/utils/rate_tracker.h"
    "include/mrs_robot_diagnostics/utils/flight_timer.h"
    "include/mrs_robot_diagnostics/utils/wh_drained_integrator.h"
    "include/mrs_robot_diagnostics/utils/host_stats.h"
)

target_link_libraries(MrsRobotDiagnostics_StateMonitor
  PRIVATE
    mrs_robot_diagnostics::mrs_robot_diagnostics  
)

rclcpp_components_register_nodes(MrsRobotDiagnostics_StateMonitor "mrs_robot_diagnostics::state_monitor::StateMonitor")

target_compile_definitions(MrsRobotDiagnostics_StateMonitor PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# ## | ----------- MrsRobotDiagnostics_CameraHandler ------------ |
#
# add_library(MrsRobotDiagnostics_CameraHandler SHARED
#   src/sensor_handler_plugins/camera_handler.cpp
# )
#
# target_sources(MrsRobotDiagnostics_CameraHandler
#   PUBLIC FILE_SET HEADERS BASE_DIRS "include"
#   FILES
#     "include/mrs_robot_diagnostics/sensor_plugins/camera_handler.h"
# )
#
# target_link_libraries(MrsRobotDiagnostics_CameraHandler
#   PRIVATE
#     mrs_robot_diagnostics::mrs_robot_diagnostics 
# )
#
# target_compile_definitions(MrsRobotDiagnostics_CameraHandler PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## | -------------- MrsRobotDiagnostics_SensorPlugins --------------- |

add_library(MrsRobotDiagnostics_SensorPlugins SHARED
  src/sensor_handler_plugins/gps_handler.cpp
  src/sensor_handler_plugins/magnetometer_handler.cpp
  src/sensor_handler_plugins/generic_handler.cpp
  src/sensor_handler_plugins/remote_controller.cpp
)

target_sources(MrsRobotDiagnostics_SensorPlugins
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_robot_diagnostics/sensor_plugins/gps_handler.h"
    "include/mrs_robot_diagnostics/sensor_plugins/magnetometer_handler.h"
    "include/mrs_robot_diagnostics/sensor_plugins/generic_handler.h"
    "include/mrs_robot_diagnostics/sensor_plugins/remote_controller.h"
)

target_link_libraries(MrsRobotDiagnostics_SensorPlugins
  PRIVATE
    mrs_robot_diagnostics::mrs_robot_diagnostics
)

target_compile_definitions(MrsRobotDiagnostics_SensorPlugins PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

pluginlib_export_plugin_description_file(mrs_robot_diagnostics plugins.xml)

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
