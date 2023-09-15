#cmake版本
cmake_minimum_required(VERSION 3.4)

#软件包名
project(panda_controller)

#设置变量
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  paramForDebug.msg
)
#调用消息生成
generate_messages()

#动态参数生成器,调用动态参数生成
generate_dynamic_reconfigure_options(
  cfg/pandaController.cfg
)

#指定包编译信息导出：它对依赖于此功能包的其他功能包来说具有重要作用
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES panda_controller
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
  DEPENDS Franka
)

#生成库
add_library(panda_controller
  src/algorithm/pandaDynLibManager.cpp
  src/communication/communication.cpp
  src/robotController/pandaController.cpp
  src/rosController/pandaRosController.cpp
)


#定义目标文件（panda_controller）依赖其他的目标文件，确保其他目标已被构建
add_dependencies(panda_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)
#将目标文件（panda_controller）与库文件进行链接
target_link_libraries(panda_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)
# 添加头文件地址
target_include_directories(panda_controller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(panda_controller PUBLIC
  include
  ${PROJECT_SOURCE_DIR}/src
  /home/wd/workSpace/WDcontroller/h
)
message(${Franka_INCLUDE_DIRS})
message(${EIGEN3_INCLUDE_DIRS})
message(${catkin_INCLUDE_DIRS})
message(${PROJECT_SOURCE_DIR})

## Installation
install(TARGETS panda_controller
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
install(FILES panda_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# catkin_install_python(
#   PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
  add_format_target(panda_controller FILES ${SOURCES} ${HEADERS})
  add_tidy_target(panda_controller
    FILES ${SOURCES}
    DEPENDS panda_controller
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()
