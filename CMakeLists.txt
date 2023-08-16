CMAKE_MINIMUM_REQUIRED(VERSION 3.22.1)
PROJECT(realsense_rtdi)

#SET(CMAKE_CXX_STANDARD 17)
#SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#set(CMAKE_PREFIX_PATH "/usr/local/lib" ${CMAKE_PREFIX_PATH})

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(ur_rtde REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} include)

SET(LIB_SOURCES
    src/KalmanFilter.cpp
    src/ObjectTracker.cpp
    src/RealSenseCam.cpp
    src/RobotController.cpp
    src/KalmanTesting.cpp
    src/Utility.cpp
)

ADD_LIBRARY(SourceLib ${LIB_SOURCES})

ADD_EXECUTABLE(realsense_rtdi src/main.cpp)

message(STATUS "Realsense package found: ${realsense2_FOUND}")

TARGET_LINK_LIBRARIES(realsense_rtdi SourceLib)
TARGET_LINK_LIBRARIES(realsense_rtdi ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(realsense_rtdi Eigen3::Eigen)
TARGET_LINK_LIBRARIES(realsense_rtdi ur_rtde::rtde)
TARGET_LINK_LIBRARIES(realsense_rtdi realsense2::realsense2)