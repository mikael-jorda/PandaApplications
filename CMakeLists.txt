cmake_minimum_required (VERSION 2.8.12)
project (PandaApplications)
SET(CMAKE_CXX_FLAGS "-std=c++11")
# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")

# set common source
set (PANDA_APPLICATIONS_COMMON_SOURCE
	)
include_directories(${PROJECT_SOURCE_DIR}/src)

# set common dependencies
# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# # - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-MODEL
FIND_PACKAGE(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS
FIND_PACKAGE(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - SAI2-PRIMITIVES
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)


set(PANDA_APPLICATIONS_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${SAI2-PRIMITIVES_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	)

# add apps
set (PANDA_APPLICATIONS_BINARY_DIR                  ${PROJECT_SOURCE_DIR}/bin)

# add_subdirectory(00-calibration_for_camera)
# add_subdirectory(00-experiment_joint_control)
# add_subdirectory(demo-cs225a)
# add_subdirectory(00-force_sensor_calibration)
# add_subdirectory(01-joint_motion)
# add_subdirectory(02-cartesian_motion)
# add_subdirectory(03-open_loop_force_control)
# add_subdirectory(04-surface_alignment)
# add_subdirectory(05-contact_driven_demo)
# add_subdirectory(06-pick_and_place)
# add_subdirectory(07-dual_arm_tray)
# add_subdirectory(08-simulation_electric_cables)
# add_subdirectory(09-haptic_painting)
# add_subdirectory(10-construction_painting)
# add_subdirectory(11-metaGraspTesting)
add_subdirectory(12-metaGraspTwoRobots)
