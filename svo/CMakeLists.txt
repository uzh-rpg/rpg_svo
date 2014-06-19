################################################################################
# user build settings

SET(TRACE TRUE)
SET(HAVE_G2O FALSE) 
SET(USE_ROS TRUE) # Set FALSE if you want to build this package without Catkin
SET(DEBUG_OUTPUT FALSE) # Only relevant if build without ROS

################################################################################

SET(PROJECT_NAME svo)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DSVO_TRACE)
ENDIF()
IF(HAVE_G2O)
  ADD_DEFINITIONS(-DUSE_BUNDLE_ADJUSTMENT)
ENDIF()
IF(USE_ROS)
  ADD_DEFINITIONS(-DSVO_USE_ROS)
ENDIF()
IF(DEBUG_OUTPUT)
  ADD_DEFINITIONS(-DSVO_DEBUG_OUTPUT)
ENDIF()

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(fast REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp roslib cmake_modules vikit_common vikit_ros)
  catkin_package(
      DEPENDS Eigen OpenCV Sophus Boost fast
      CATKIN_DEPENDS roscpp roslib vikit_common vikit_ros
      INCLUDE_DIRS include
      LIBRARIES svo
  )
ELSE()
  FIND_PACKAGE(vikit_common REQUIRED)
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${fast_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${fast_LIBRARIES}
)

IF(USE_ROS)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ELSE()
  INCLUDE_DIRECTORIES(${vikit_common_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${vikit_common_LIBRARIES})
ENDIF()

# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_base.cpp
  src/frame.cpp
  src/point.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/initialization.cpp
  src/matcher.cpp
  src/reprojector.cpp
  src/feature_alignment.cpp
  src/feature_detection.cpp
  src/depth_filter.cpp
  src/config.cpp
  src/sparse_img_align.cpp)

# Add g2o if available
IF(HAVE_G2O)
  FIND_PACKAGE(G2O REQUIRED)
  INCLUDE_DIRECTORIES(
    ${G2O_INCLUDE_DIR}
    /usr/include/suitesparse # for cholmod
  )
  LIST(APPEND LINK_LIBS
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_TYPES_SBA} cholmod cxsparse
  )
  LIST(APPEND SOURCEFILES src/bundle_adjustment.cpp)
ENDIF(HAVE_G2O)

# Create svo library
ADD_LIBRARY(svo SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(svo ${LINK_LIBS})

################################################################################
# TESTS

ADD_EXECUTABLE(test_feature_align test/test_feature_alignment.cpp)
TARGET_LINK_LIBRARIES(test_feature_align svo)

ADD_EXECUTABLE(test_pipeline test/test_pipeline.cpp)
TARGET_LINK_LIBRARIES(test_pipeline svo)

ADD_EXECUTABLE(test_matcher test/test_matcher.cpp)
TARGET_LINK_LIBRARIES(test_matcher svo)

ADD_EXECUTABLE(test_feature_detection test/test_feature_detection.cpp)
TARGET_LINK_LIBRARIES(test_feature_detection svo)

ADD_EXECUTABLE(test_depth_filter test/test_depth_filter.cpp)
TARGET_LINK_LIBRARIES(test_depth_filter svo)

ADD_EXECUTABLE(test_sparse_img_align test/test_sparse_img_align.cpp)
TARGET_LINK_LIBRARIES(test_sparse_img_align svo)

ADD_EXECUTABLE(test_pose_optimizer test/test_pose_optimizer.cpp)
TARGET_LINK_LIBRARIES(test_pose_optimizer svo) 
