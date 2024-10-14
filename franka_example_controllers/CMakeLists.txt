#cmake版本
cmake_minimum_required(VERSION 3.4)

#软件包名
project(franka_example_controllers)

#设置变量
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/) 

#查找编译依赖的其他CMake/Catkin包
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  joint_limits_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  urdf
  visualization_msgs
  OpenCV REQUIRED         
  cv_bridge
  image_transport
)

#查找编译依赖的库
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

#消息生成器
add_message_files(FILES
  JointTorqueComparison.msg
  paramForDebug.msg
  admittanceControllerMsg.msg
  nocontactImpedanceMsg.msg
)
#调用消息生成
generate_messages()

#动态参数生成器,调用动态参数生成
generate_dynamic_reconfigure_options(
  cfg/admittance_controller.cfg
  cfg/nullspace_impedance_controller.cfg
  cfg/impedance_controller.cfg
  cfg/robust_controller.cfg
  cfg/nocontact_impedance_controller.cfg
)

#指定包编译信息导出：它对依赖于此功能包的其他功能包来说具有重要作用
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_example_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    joint_limits_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
    visualization_msgs
    sensor_msgs
    cv_bridge
    image_transport
  DEPENDS Franka
)
set(THIRD_PARTY_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_x64)
set(THIRD_PARTY_LIB ${THIRD_PARTY_LIB_DIR}/libhps_ftc.so)

message("+++++++++++++++++")
message(STATUS "THIRD_PARTY_LIB: ${THIRD_PARTY_LIB}")
# message(${CMAKE_CURRENT_SOURCE_DIR})

#生成库
add_library(franka_example_controllers
  src/admittance_controller/admittance_controller.cpp
  src/admittance_controller/admittance_dyn_controller.cpp
  src/nullSpace_impedance/nullspace_impedance_controller.cpp
  src/nullSpace_impedance/nullspace_impedance_EBobserver_controller.cpp
  src/nullSpace_impedance/nullspace_impedance_MBobserver_controller.cpp
  src/robust_controller/robust_controller.cpp
  src/impedance_controller/joint_impedance_controller.cpp
  src/impedance_controller/cartesian_impedance_controller.cpp
  src/nocontact_impedance/nocontact_impedance_controller.cpp
  src/trajectory.cpp
  src/pinocchino_interactive.cpp
  src/hps_sensor.cpp
)

#定义目标文件（franka_example_controllers）依赖其他的目标文件，确保其他目标已被构建
add_dependencies(franka_example_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

#将目标文件（franka_example_controllers）与库文件进行链接
target_link_libraries(franka_example_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  pinocchio::pinocchio
  pthread 
  ${THIRD_PARTY_LIB}
)
# 添加头文件地址
target_include_directories(franka_example_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_include_directories(franka_example_controllers PUBLIC
  include
)

## Installation
install(TARGETS franka_example_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_example_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_example_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_example_controllers
    FILES ${SOURCES}
    DEPENDS franka_example_controllers
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()
