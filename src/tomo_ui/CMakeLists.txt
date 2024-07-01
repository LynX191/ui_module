cmake_minimum_required(VERSION 3.5)
project(tomo_ui)

if (POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()
if (POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options()
endif()

include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  system
  thread
  date_time
)

# catkin_python_setup()


option(UseQt5 "UseQt5" ON)
if (UseQt5)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Quick Gui Widgets Concurrent Network OpenGL)
  # set variable names already used with Qt4
  set(QT_LIBRARIES Qt5::Widgets Qt5::Core Qt5::Quick Qt5::Concurrent Qt5::OpenGL)
  set(QTVERSION ${Qt5Widgets_VERSION})
else()
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)
  include(${QT_USE_FILE})
endif()

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy

  tomo_action
  tomo_msgs
  tomo_utils
  tomo_comm
  tomo_devices_carton
  tomo_camera
  tomo_rviz
)

catkin_package(
  INCLUDE_DIRS
	src
  ${EXPORT_HEADER_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  tomo_rviz
  tomo_comm
  tomo_devices_carton
)

find_package(yaml-cpp REQUIRED)
# set(Libssh_DIR "/usr/local/lib/cmake/libssh/libssh-config.cmake")
find_package(libssh REQUIRED)
include_directories(${LIBSSH_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${tomo_rviz_INCLUDE_DIRS}
  ../tomo_rviz/include/
)
link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

set(SOURCES_FILES
  src/AICore/station_core.cpp

  src/Dialog/modal_dialogbox.cpp

  src/Script/Define/struct_def.h
  src/Script/Config/cfg_app.cpp
  src/Script/Utilities/utilities.cpp

  src/Model/TomoModel/mode_model.cpp
  src/Model/TomoModel/state_model.cpp
  src/Model/image_model.cpp
  src/Model/login_model.cpp
  src/Model/notify_model.cpp

  src/ViewModel/DashboardVM/dash_board_vm.cpp
  src/ViewModel/TomoVM/Feature/mode_tomo_vm.cpp
  src/ViewModel/TomoVM/Feature/state_tomo_vm.cpp
  src/ViewModel/TomoVM/tomo_vm.cpp
  src/ViewModel/HeadVM/head_vm.cpp
  src/ViewModel/ShipperVM/shipper_vm.cpp
  src/ViewModel/PimVM/pim_vm.cpp
  src/ViewModel/controlbar_vm.cpp
  src/ViewModel/image_view_vm.cpp
  src/ViewModel/image_writer.cpp
  src/ViewModel/login_vm.cpp
  src/ViewModel/main_vm.cpp
  src/ViewModel/master_app.cpp
  src/ViewModel/systembar_vm.cpp
  src/ViewModel/track_vm.cpp
  src/ViewModel/SettingVM/setting_vm.cpp
  src/ViewModel/SystemLogVM/base_terminal.cpp
  src/ViewModel/SystemLogVM/system_log_vm.cpp
  src/ViewModel/IoVM/io_control_vm.cpp
  src/ViewModel/ImagingVM/imaging_client_vm.cpp
  src/ViewModel/StationVM/station_vm.cpp
  src/ViewModel/ProductionVM/production_vm.cpp
  src/ViewModel/NotifyVM/notify_vm.cpp

  src/main.cpp
  src/qml.qrc
)

add_executable(${PROJECT_NAME}
    ${SOURCES_FILES}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  depthai::opencv
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML2_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${tomo_rviz_LIBRARIES}
  ${LIBSSH_LIBRARIES}
  ssh

	-ldl
	-lutil
	-lcups
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
