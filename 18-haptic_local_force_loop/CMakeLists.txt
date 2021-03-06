# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PANDA_APPLICATIONS_BINARY_DIR}/18-haptic_local_force_loop)
ADD_EXECUTABLE (app18 app.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (app_new_pf18 app_new_pf.cpp ForceSpaceParticleFilter_weight_mem.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
# ADD_EXECUTABLE (app_new_fsensor18 app_new_fsensor.cpp ForceSpaceParticleFilter_weight_mem.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
# ADD_EXECUTABLE (simviz_sphere18 simviz_sphere.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_panda18 simviz_panda.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (viz_panda18 viz_panda.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
# ADD_EXECUTABLE (control_sphere18 controller_sphere.cpp ForceSpaceParticleFilter.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (control_panda_loc_unified18 controller_panda_loc_unified.cpp ForceSpaceParticleFilter_weight_mem.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (control_panda_loc_impedance18 controller_panda_loc_impedance.cpp ForceSpaceParticleFilter_weight_mem.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (control_panda_classical18 controller_panda_classical.cpp ForceSpaceParticleFilter_weight_mem.cpp ${PANDA_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (app18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (app_new_pf18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
# TARGET_LINK_LIBRARIES (app_new_fsensor18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
# TARGET_LINK_LIBRARIES (simviz_sphere18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_panda18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (viz_panda18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
# TARGET_LINK_LIBRARIES (control_sphere18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (control_panda_loc_unified18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (control_panda_loc_impedance18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (control_panda_classical18 ${PANDA_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_sphere.urdf world_panda.urdf world_particles.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf sphere.urdf DESTINATION ${APP_RESOURCE_DIR})
