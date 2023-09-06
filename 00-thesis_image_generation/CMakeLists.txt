# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PANDA_APPLICATIONS_BINARY_DIR}/00-thesis_image_generation)
ADD_EXECUTABLE (panda_contact panda_contact.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (panda_redundancy panda_redundancy.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (panda_joint_limit panda_joint_limit.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (panda_obstacle_avoidance panda_obstacle_avoidance.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (panda_obstacle_avoidance_elbow panda_obstacle_avoidance_elbow.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (panda_contact ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (panda_redundancy ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (panda_joint_limit ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (panda_obstacle_avoidance ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (panda_obstacle_avoidance_elbow ${PANDA_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_panda_contact.urdf world_panda_redundancy.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY world_panda_joint_limit.urdf world_panda_obstacle_avoidance.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf panda_arm_hand.urdf DESTINATION ${APP_RESOURCE_DIR})
