# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PANDA_APPLICATIONS_BINARY_DIR}/00-model_update_tests)
ADD_EXECUTABLE (measure_robot_data measure_robot_data.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (model_computation_speed model_computation_speed.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
# ADD_EXECUTABLE (controller01 controller.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_measure_data simviz.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (measure_robot_data ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (model_computation_speed ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
# TARGET_LINK_LIBRARIES (controller01 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_measure_data ${PANDA_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
# FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY models/panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY models/two_pandas_mobile.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY models/humanoid.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY models/two_pandas_mobile_hands.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY models/humanoid_hands.urdf DESTINATION ${APP_RESOURCE_DIR})
# FILE(COPY models/Allegrohand.urdf DESTINATION ${APP_RESOURCE_DIR})
# FILE(COPY models/panda_arm_Allegrohand_new.urdf DESTINATION ${APP_RESOURCE_DIR})
