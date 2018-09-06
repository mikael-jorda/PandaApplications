# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PANDA_APPLICATIONS_BINARY_DIR}/00-test_jacobians)
ADD_EXECUTABLE (simviz_jacobians simviz.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (jacobians jacobians.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (jacobians_singular_values jacobian_singular_values.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_jacobians ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (jacobians ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (jacobians_singular_values ${PANDA_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${PANDA_APPLICATIONS_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/00-test_jacobians)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf panda_arm_hand.urdf world.urdf DESTINATION ${APP_RESOURCE_DIR})
