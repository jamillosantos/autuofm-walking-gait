cmake_minimum_required(VERSION 3.6)
project(walking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
	src/main.cpp

	src/data/servo.cpp
	src/data/servo.h

	src/engine/robot.cpp
	src/engine/robot.h

	src/data/vector.cpp
	src/data/vector.h

	src/engine/part.cpp
	src/engine/part.h
	src/engine/ik.cpp src/engine/ik.h src/data/configuration.cpp src/data/configuration.h src/data/angle.cpp src/data/angle.h src/sensors/imu.cpp src/sensors/imu.h src/sensors/gyro.cpp src/sensors/gyro.h)
add_executable(walking ${SOURCE_FILES})