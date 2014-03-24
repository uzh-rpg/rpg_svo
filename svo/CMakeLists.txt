################################################################################
# user build settings

SET(DEBUG FALSE)
SET(TRACE FALSE)
SET(HAVE_G2O FALSE) 

################################################################################

SET(PROJECT_NAME svo)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set ARM_ARCHITECTURE environment variable on Odroid
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(IS_ARM TRUE)
#  ADD_DEFINITIONS(-DHAVE_FAST_NEON)
ELSE()
  SET(IS_ARM FALSE)
ENDIF()

# Set definitions
IF(DEBUG)
  ADD_DEFINITIONS(-DSVO_DEBUG)
ENDIF()
IF(TRACE)
  ADD_DEFINITIONS(-DSVO_TRACE)
ENDIF()
IF(HAVE_G2O)
  ADD_DEFINITIONS(-DUSE_BUNDLE_ADJUSTMENT)
ENDIF()
ADD_DEFINITIONS(-DUSE_ROS)

# Set build flags
SET(CMAKE_CXX_FLAGS "-Wall -Werror -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
IF(IS_ARM)
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a -fomit-frame-pointer -fsee -funsafe-loop-optimizations -fno-signed-zeros -fno-math-errno -funroll-loops")
ELSE()
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer -fsee -funsafe-loop-optimizations -fno-signed-zeros -fno-math-errno -funroll-loops")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp vikit_common vikit_ros)

# Add plain cmake packages 
FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(fast REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread signals)

# Describe catkin Project
catkin_package(
    DEPENDS Eigen OpenCV Sophus Boost fast
    CATKIN_DEPENDS roscpp vikit_common vikit_ros
    INCLUDE_DIRS include
    LIBRARIES svo
)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${fast_INCLUDE_DIRS}
)
IF(HAVE_G2O)
  INCLUDE_DIRECTORIES(
    ${G2O_ROOT}/include
    /usr/include/suitesparse # for cholmod
  )
ENDIF()

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${fast_LIBRARIES}
)
IF(HAVE_G2O)
  LIST(APPEND LINK_LIBS
    g2o_core g2o_stuff g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_types_sba
    cholmod cxsparse
  )
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
  src/reprojection.cpp
  src/feature_alignment.cpp
  src/feature_detection.cpp
  src/depth_filter.cpp
  src/config.cpp
  src/sparse_img_align.cpp)
IF(HAVE_G2O)
  LIST(APPEND SOURCEFILES src/bundle_adjustment.cpp)
ENDIF(HAVE_G2O)

# Create svo library
ADD_LIBRARY(svo SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(svo ${LINK_LIBS})


################################################################################
# TESTS

IF(DEFINED ENV{SVO_DATASET_DIR})
  ADD_DEFINITIONS(-DTEST_DATA_DIR=\"$ENV{SVO_DATASET_DIR}\")
ELSE()
  ADD_DEFINITIONS(-DTEST_DATA_DIR=\"${PROJECT_SOURCE_DIR}/test/data\")
ENDIF()
ADD_DEFINITIONS(-DTEST_TRACE_DIR=\"${PROJECT_SOURCE_DIR}/test/results\")

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
