cmake_minimum_required(VERSION 3.5)
project(tomo_rviz)

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

if(UNIX AND NOT APPLE)
  find_package(X11 REQUIRED)
endif()

find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  system
  thread
  date_time
)

find_package(urdfdom_headers REQUIRED)

find_package(PkgConfig REQUIRED)

find_package(ASSIMP QUIET)

if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()

if (ASSIMP_FOUND)
  if( NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150" )
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  #catkin_lint: ignore_once link_directory
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
endif()

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif(NOT OGRE_OV_FOUND)

if(NOT DEFINED OGRE_PLUGIN_PATH)
  execute_process(COMMAND
    ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
    OUTPUT_VARIABLE OGRE_PLUGIN_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif(NOT DEFINED OGRE_PLUGIN_PATH)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})

if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
  unset(OGRE_OV_LIBRARIES_ABS_TMP)
  foreach(_lib ${OGRE_OV_LIBRARIES})
    set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")

    find_library(${OGRE_OV_LIB_TAG}
            NAMES ${_lib}
            HINTS ${OGRE_OV_LIBRARY_DIRS}
            PATHS ${OGRE_OV_LIBRARY_DIRS}
    )

    set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
  endforeach(_lib)

  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
        CACHE FILEPATH "Pathname of library ${_lib}")
endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)

find_package(OpenGL REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  #catkin_lint: ignore_once deprecated_cmake_module
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


# catkin_python_setup()

if(NOT OGRE_OV_LIBRARIES_ABS)
  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES})
endif()

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
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rviz
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
  sensor_msgs
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  message_filters
  message_generation
  nav_msgs
  pluginlib
  resource_retriever
  rosconsole
  roslib
  rospy
  tomo_comm
  std_srvs
  tf
  urdf
  visualization_msgs
  tomo_devices_carton
  tomo_action
  tomo_utils
)

set(EXPORT_HEADER_DIR "${CATKIN_DEVEL_PREFIX}/include")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}")

catkin_package(
  INCLUDE_DIRS
  src
  ${EXPORT_HEADER_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${OGRE_OV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  ${OGRE_OV_LIBRARIES_ABS}
  ${rviz_ADDITIONAL_LIBRARIES}
  ${OPENGL_LIBRARIES}
  CATKIN_DEPENDS
	geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  message_filters
  message_runtime
  nav_msgs
  pluginlib
  resource_retriever
  roscpp
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  tf
  urdf
  visualization_msgs
  tomo_comm
  tomo_devices_carton
)

find_package(yaml-cpp REQUIRED)

include_directories(${YAML_CPP_INCLUDE_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OGRE_OV_INCLUDE_DIRS}
  ${OPENGL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}

)

link_directories(${catkin_LIBRARY_DIRS})

find_package(GTK2)
include_directories(${GTK2_INCLUDE_DIRS})
include_directories(/usr/include/gdk-pixbuf-2.0)
add_definitions(-DQT_NO_KEYWORDS)
include_directories(src ${EXPORT_HEADER_DIR})

qt5_wrap_cpp(MOC_FILES
  include/tomo_rviz/visualizer_app.h
  include/tomo_rviz/visualization_frame.h
  include/tomo_rviz/displays_panel.h
  include/tomo_rviz/panel_dock_widget.h
  include/tomo_rviz/panel_factory.h
  include/tomo_rviz/render_panel.h
)

set(RVIZ_SOURCES_FILES
	src/visualization_frame.cpp
	src/visualizer_app.cpp
	src/panel_factory.cpp
	src/displays_panel.cpp
	src/panel_dock_widget.cpp
)
add_library(${PROJECT_NAME}
	${MOC_FILES}
	${RVIZ_SOURCES_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OGRE_OV_LIBRARIES_ABS}
  ${OpenCV_LIBS}
  ${OPENGL_LIBRARIES}
  ${TinyXML2_LIBRARIES}
  ${X11_X11_LIB}
  ${ASSIMP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(main "src/main.cpp" ${RVIZ_SOURCES_FILES} ${MOC_FILES})
target_link_libraries(main ${catkin_LIBRARIES} ${QT_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  rosdoc.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
