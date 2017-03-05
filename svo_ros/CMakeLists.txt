SET(PROJECT_NAME svo_ros)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()


# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
  roscpp
  cmake_modules
  nav_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf
  svo
  svo_msgs
  vikit_common
  vikit_ros
)

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)

# Describe catkin Project
catkin_package(
  DEPENDS Eigen OpenCV Sophus Boost
  CATKIN_DEPENDS roscpp nav_msgs std_msgs visualization_msgs sensor_msgs image_transport cv_bridge tf svo svo_msgs vikit_common vikit_ros
  INCLUDE_DIRS include
  LIBRARIES svo_visualizer
)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Create visualizer library
ADD_LIBRARY(svo_visualizer src/visualizer.cpp)
TARGET_LINK_LIBRARIES(svo_visualizer ${LINK_LIBS})
                                 
# Create Executables                      
ADD_EXECUTABLE(vo src/vo_node.cpp)
TARGET_LINK_LIBRARIES(vo svo_visualizer)
ADD_EXECUTABLE(benchmark src/benchmark_node.cpp)
TARGET_LINK_LIBRARIES(benchmark svo_visualizer)
