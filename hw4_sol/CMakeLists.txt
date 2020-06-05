# create an executable
ADD_EXECUTABLE (hw4-p1-sol-interfaces p1-main-sol-interfaces.cpp)
ADD_EXECUTABLE (hw4-p2-sol p2-main-sol.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw4-p1-sol-interfaces ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw4-p2-sol  ${CS327A_COMMON_LIBRARIES})

# copy example interface to output
FILE(COPY hw4-p1-interfaces.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS327A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/hw4_sol)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_1_puma.urdf world_2_puma.urdf object.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy in sai2-interfaces 
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch script
FILE(COPY p1-launch.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})