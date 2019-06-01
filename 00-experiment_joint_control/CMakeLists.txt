# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PANDA_APPLICATIONS_BINARY_DIR}/00-experiment_joint_control)
ADD_EXECUTABLE (experiment_joint_control controller.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_experiment_joint_control simviz.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (experiment_joint_control ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_experiment_joint_control ${PANDA_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_hand.urdf DESTINATION ${APP_RESOURCE_DIR})