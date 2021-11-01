cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project( landmarkVersion6 )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )

# OpenCV
find_package( OpenCV REQUIRED )

# Pylon 
find_package(Pylon REQUIRED)

#UR_RTDE
FIND_PACKAGE( ur_rtde REQUIRED COMPONENTS
	spin_msgs
	spin_srvs
	geometry_msgs
	tf2_geometry_msgs
	tf2
)

include_directories(${Pylon_INCLUDE_DIRS})


add_executable( main main.cpp camera.cpp camera.h cameraSettings.h cameraCalibration.cpp cameraCalibration.h detectionObject.cpp detectionObject.h workspaceCalibration.cpp workspaceCalibration.h detectionMarker.cpp detectionMarker.h moveArm.cpp moveArm.h)
target_link_libraries( main 
	${OpenCV_LIBS} 
	${Pylon_LIBRARIES} 
	${ADDITIONAL_LINK_LIBRARIES}
	${Boost_SYSTEM_LIBRARY}
        ${Boost_FILESYSTEM_LIBRARY}
)
TARGET_LINK_LIBRARIES(main ur_rtde::rtde )
