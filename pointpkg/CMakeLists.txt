cmake_minimum_required(VERSION 3.0.2)
project(pointpkg)

string(APPEND CMAKE_CXX_FLAGS " /W4")#onnx


## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES remotepkg
  #CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  src
  src/not_used
  ${catkin_INCLUDE_DIRS}
)

##################################################################
###############           CoppeliaSim            #################
##################################################################
add_library(
   extApi_ SHARED
   remoteApi/extApi.c
   remoteApi/extApiPlatform.c
 )
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

##################################################################
###############              OpenCV              #################
##################################################################
set(OpenCV_DIR "../opencv/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

##################################################################
###############               PCL                #################
##################################################################
set(PCL_DIR "C:/Users/y4236/Documents/pcws/src/pointpkg/PCL1.11.1/cmake")
#set(Boost_DEBUG ON) #コンパイルエラー用
find_package(PCL 1.11 REQUIRED) #すべて
#find_package(PCL 1.3 REQUIRED COMPONENTS common io kdtree geometry search octree) #指定
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message("## PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")

##################################################################
###############             RealSense            #################
##################################################################
set(REALSENSE_INCLUDE_DIRS "C:/Program Files (x86)/Intel RealSense SDK 2.0/include")
set(REALSENSE_LIBS "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64/realsense2.lib")
include_directories(${REALSENSE_INCLUDE_DIRS})
link_directories(${REALSENSE_LIBS})

##################################################################
###############               ONNX               #################
##################################################################
#https://github.com/microsoft/onnxruntime-inference-examples/blob/main/c_cxx/CMakeLists.txt
option(onnxruntime_USE_CUDA "Build with CUDA support" ON)
option(onnxruntime_USE_OPENVINO "Build with OpenVINO support" ON)
option(onnxruntime_USE_NNAPI_BUILTIN "Build with builtin NNAPI lib for Android NNAPI support" OFF)
option(onnxruntime_USE_DNNL "Build with DNNL support" ON)
option(onnxruntime_USE_NUPHAR "Build with Nuphar" ON)
option(onnxruntime_USE_TENSORRT "Build with TensorRT support" ON)
option(LIBPNG_ROOTDIR "libpng root dir")
option(ONNXRUNTIME_ROOTDIR "onnxruntime root dir")
if(onnxruntime_USE_CUDA)
  add_definitions(-DUSE_CUDA)
endif()
if(onnxruntime_USE_OPENVINO)
  add_definitions(-DUSE_OPENVINO)
endif()
if(onnxruntime_USE_NNAPI_BUILTIN)
  add_definitions(-DUSE_NNAPI)
endif()
if(onnxruntime_USE_DNNL)
  add_definitions(-DUSE_DNNL)
endif()
if(onnxruntime_USE_NUPHAR)
  add_definitions(-DUSE_NUPHAR)
endif()
if(onnxruntime_USE_TENSORRT)
  add_definitions(-DUSE_TENSORRT)
endif()
if(onnxruntime_USE_DML)
  message("Enabling DML")
  add_definitions(-DUSE_DML)
endif()
set(ONNXRUNTIME_ROOTDIR "C:/Users/y4236/Documents/pcws/src/pointpkg/onnxruntime-win-x64-gpu-1.12.1")
include_directories("${ONNXRUNTIME_ROOTDIR}/include")#"${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/session")
link_directories("${ONNXRUNTIME_ROOTDIR}/lib/onnxruntime.lib")
#set(ONNXRUNTIME_LIBS "${ONNXRUNTIME_ROOTDIR}/lib/")

##################################################################
####################   excutables and link   #####################
##################################################################
add_executable(pointer
src/main.cpp
src/manager.cpp
src/gui.cpp
src/ros_interface.cpp
src/coppeliasim_interface.cpp
src/pcl2.cpp
src/labview_interface.cpp
#src/params.cpp
src/ros_param.cpp

src/realsense_interface.cpp

src/yolo_detector.cpp
src/yolo_utils.cpp


#src/not_used/client.cpp
#src/not_used/manager.cpp


)
target_compile_options(pointer PUBLIC -O2)

target_link_libraries(pointer
   ${catkin_LIBRARIES}
   extApi_
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${REALSENSE_LIBS}
   ${ONNXRUNTIME_LIBS}
)

add_executable(tdmaker_for_pc
   src/tdmaker_for_pc.cpp
)

target_link_libraries(tdmaker_for_pc
   ${catkin_LIBRARIES}
)


add_executable(pcd_maker
src/cad/pcd_maker.cpp
)
target_link_libraries(pcd_maker
   ${catkin_LIBRARIES}
)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


## Declare a C++ library
 



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/remotepkg_node.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_remotepkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(extApi remoteApi/extApi.c)
#target_link_libraries(extApi ${catkin_LIBRARIES})

#add_executable(extApiPlatform remoteApi/extApiPlatform.c)
#target_link_libraries(extApiPlatform ${catkin_LIBRARIES})

