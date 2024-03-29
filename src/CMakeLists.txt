cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project( landmarkVersion6 )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_AUTOMOC ON)

list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )

include(FindPkgConfig)
pkg_check_modules(CURLPP REQUIRED curlpp)

# OpenCV
find_package( OpenCV REQUIRED )

# Pylon 
find_package(Pylon REQUIRED)

# RobWork
find_package(RobWork REQUIRED)

find_package(Qt5Widgets REQUIRED)

find_package(Qt5PrintSupport REQUIRED)

find_package(Qt5Network REQUIRED)

find_package(VISP REQUIRED)

#UR_RTDE
FIND_PACKAGE( ur_rtde REQUIRED COMPONENTS)


FIND_PACKAGE( Boost COMPONENTS system program_options filesystem thread REQUIRED )
FIND_PACKAGE(CURL)

IF(CURL_FOUND)
  INCLUDE_DIRECTORIES(${CURL_INCLUDE_DIR})
  SET(requiredlibs ${requiredlibs} ${CURL_LIBRARIES} )
ELSE(CURL_FOUND)
  #MESSAGE(FATAL_ERROR "Could not find the CURL library and development files.")
ENDIF(CURL_FOUND)



include_directories(
	${Pylon_INCLUDE_DIRS}
        ${ROBWORK_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${VISP_INCLUDE_DIRS})


add_definitions(${Qt5Widgets_DEFINITIONS})

LINK_DIRECTORIES(
	${ROBWORK_LIBRARY_DIRS})

set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")


add_executable( main
    main.cpp
    camera.cpp
    camera.h
    cameraConfirguration.h
    cameraCalibration.cpp
    cameraCalibration.h
    workspaceCalibration.cpp
    workspaceCalibration.h
    detectionCheckerboard.cpp
    detectionCheckerboard.h
    moveRobot.cpp
    moveRobot.h
    socket.cpp
    socket.h
    mytcpserver.cpp
    mytcpserver.h
    )

target_link_libraries( main 
        #${OpenCV_LIBS}
	${Pylon_LIBRARIES} 
	${ADDITIONAL_LINK_LIBRARIES}
        ${ROBWORK_LIBRARIES}
        ${VISP_LIBRARIES}
        )

TARGET_LINK_LIBRARIES(main ur_rtde::rtde )

target_link_libraries(main Qt5::Network)
