# create an executable
ADD_EXECUTABLE (hw2-p1 p1-main.cpp)
ADD_EXECUTABLE (hw2-p2 p2-main.cpp)
ADD_EXECUTABLE (hw2-p3 p3-main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw2-p1 ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw2-p2 ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw2-p3 ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS327A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/hw2)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_1_puma.urdf world_2_iiwa.urdf world_3_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})
