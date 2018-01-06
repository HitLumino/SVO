################################################################################
# user build settings

SET(TRACE TRUE)
SET(HAVE_G2O FALSE) 
SET(DEBUG_OUTPUT FALSE) # Only relevant if build without ROS

################################################################################

SET(PROJECT_NAME svo)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
#SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
message(${CMAKE_MODULE_PATH})

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
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system filesystem)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Thirdparty/vikit_common/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Thirdparty/fast/include)

#FIND_PACKAGE(vikit_common REQUIRED)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include dirs
INCLUDE_DIRECTORIES(
  	include
  	${Eigen_INCLUDE_DIRS}
  	${OpenCV_INCLUDE_DIRS}
  	${Sophus_INCLUDE_DIRS}
  	${Boost_INCLUDE_DIRS}
  	${Pangolin_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  	${OpenCV_LIBS}
  	${Sophus_LIBRARIES}
  	${Boost_LIBRARIES}
	${Pangolin_LIBRARIES}
)

IF(Pangolin_FOUND)
	MESSAGE("--- FOUND Pangolin, compiling the svo pangolin program")
	INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIRS})
	
	LIST(APPEND LINK_LIBS 
  	${Pangolin_LIBRARIES})
  	message("Pangolin path: ${Pangolin_LIBRARIES}")
ELSE()
	MESSAGE("--- Pangolin Not Found! ---")
ENDIF()

# Pack the Vikit as a library
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/Thirdparty/vikit_common/)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/Thirdparty/fast/)

INCLUDE_DIRECTORIES(${vikit_common_INCLUDE_DIRS})
LIST(APPEND LINK_LIBS ${vikit_common_LIBRARIES})

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
  	src/viewer.cpp
  	src/map_drawer.cpp
  	src/sparse_img_align.cpp
	src/IMU_handler.cpp
)

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
SET(LIBRARIES svo 
	vikit_common
	fast)

ADD_EXECUTABLE(test_feature_align test/test_feature_alignment.cpp)
TARGET_LINK_LIBRARIES(test_feature_align ${LIBRARIES})

ADD_EXECUTABLE(test_pipeline 
	test/test_pipeline.cpp
)

TARGET_LINK_LIBRARIES(test_pipeline 
	${LIBRARIES}
	${LINK_LIBS}
)

ADD_EXECUTABLE(test_matcher test/test_matcher.cpp)
TARGET_LINK_LIBRARIES(test_matcher ${LIBRARIES})

ADD_EXECUTABLE(test_feature_detection test/test_feature_detection.cpp)
TARGET_LINK_LIBRARIES(test_feature_detection ${LIBRARIES})

ADD_EXECUTABLE(test_depth_filter test/test_depth_filter.cpp)
TARGET_LINK_LIBRARIES(test_depth_filter ${LIBRARIES})

ADD_EXECUTABLE(test_sparse_img_align test/test_sparse_img_align.cpp)
TARGET_LINK_LIBRARIES(test_sparse_img_align ${LIBRARIES})

ADD_EXECUTABLE(test_pose_optimizer test/test_pose_optimizer.cpp)
TARGET_LINK_LIBRARIES(test_pose_optimizer ${LIBRARIES}) 
