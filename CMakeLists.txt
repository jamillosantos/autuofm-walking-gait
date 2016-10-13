cmake_minimum_required(VERSION 3.4)
project(walking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

message("Configuration")
set(THIRD_PARTY_LIBRARIES "${CMAKE_SOURCE_DIR}/libs" CACHE FILEPATH "Third party libraries directory")
set(ENABLE_TEST ON CACHE BOOL "Enable testing for this build")
set(TEST_OUTPUT ${CMAKE_BINARY_DIR} CACHE FILEPATH "Output directory of tests")
set(TEST_RESOURCE_DIR ${CMAKE_SOURCE_DIR}/test/resources CACHE FILEPATH "Resource folder for tests")
set(COVERAGE OFF CACHE BOOL "Enables coverage configuration")
set(PROFILING OFF CACHE BOOL "Enables profiling configuration")
add_definitions(-DTEST_RESOURCE_DIR="${TEST_RESOURCE_DIR}")
add_definitions(-DBOOST_LOG_DYN_LINK=1)
add_definitions(-DDEFAULT_CONFIG_FILE="${CMAKE_SOURCE_DIR}/config/vision.json")
if (${ENABLE_TEST})
	message("-- Tests enabled")
	message("--   Tests output: ${TEST_OUTPUT}")
	message("--   Resources at: ${TEST_RESOURCE_DIR}")
else()
	message("-- Tests disabled")
endif()

# libboost
set(BOOST_VERSION "1.62.0"
	CACHE STRING "GTest library's version")
string(REPLACE "."
	"_" BOOST_VERSION_UNDERSCORE
	${BOOST_VERSION})
set(BOOST_DIR "${THIRD_PARTY_LIBRARIES}/boost_${BOOST_VERSION_UNDERSCORE}"
	CACHE STRING "Boost destiny folder")
set(BOOST_DOWNLOAD_MD5 "5fb94629535c19e48703bdb2b2e9490f"
	CACHE STRING "Boost library download MD5 hash for integrity verification")
set(BOOST_REQUIRED_COMPONENTS "system;filesystem;regex;log_setup;log;program_options"
	CACHE STRING "Boost library components")

# JSONCPP
set(JSONCPP_VERSION "1.7.7"
	CACHE STRING "JSON-CPP library's version")
set(JSONCPP_URL "https://github.com/open-source-parsers/jsoncpp/archive/${JSONCPP_VERSION}.tar.gz"
	CACHE STRING "JSON-CPP library's git repository URL")
set(JSONCPP_DIR "${THIRD_PARTY_LIBRARIES}/jsoncpp-${JSONCPP_VERSION}"
	CACHE STRING "JSON-CPP destiny folder")
set(JSONCPP_MD5 "9b51c65c563463220a8cb5fa33d525f8"
	CACHE STRING "JSON-CPP md5")

# GTest
set(GTEST_VERSION "release-1.7.0"
	CACHE STRING "GTest library's version")
set(GTEST_URL "https://github.com/google/googletest/archive/${GTEST_VERSION}.tar.gz"
	CACHE STRING "GTest library's git repository URL")
set(GTEST_MD5 "4ff6353b2560df0afecfbda3b2763847"
	CACHE STRING "Google Test download MD5 hash for integrity verification")
set(GTEST_DIR "${THIRD_PARTY_LIBRARIES}/googletest-${GTEST_VERSION}"
	CACHE STRING "GTest destiny folder")

function(download result url md5 file)
	if (EXISTS ${file})
		message("${file} exists")
		file(MD5 ${file} file_md5)
		if (NOT ${file_md5} STREQUAL ${md5})
			message("MD5 does not match")
			message("     Got: ${file_md5}")
			message("Expected: ${md5}")
			file(REMOVE ${file})
			message("Downloading ${url}")
			file(DOWNLOAD ${url} ${file} SHOW_PROGRESS)
		endif()
	else()
		message("Downloading ${url}")
		file(DOWNLOAD ${url} ${file} SHOW_PROGRESS)
	endif()
	file(MD5 ${file} file_md5)
	if (${file_md5} STREQUAL ${md5})
		set(${result} ${file} PARENT_SCOPE)
	else()
		set(${result} "ERROR" PARENT_SCOPE)
		message(WARNING "MD5 verification failed.")
		message(WARNING "     Got: ${file_md5}.")
		message(WARNING "Expected: ${md5}.")
	endif()
endfunction(download)

message("JSONCPP")
if (NOT EXISTS ${JSONCPP_DIR})
	set(JSONCPP_TMP_FILE /tmp/jsoncpp.tar.gz)
	download(JSONCPP_RESULT ${JSONCPP_URL} ${JSONCPP_MD5} ${JSONCPP_TMP_FILE})
	if ("${JSONCPP_RESULT}" STREQUAL "ERROR")
		message(FATAL_ERROR "Cannot download JSON-CPP.")
	else()
		message("-- Extracting JSON-CPP to '${JSONCPP_DIR}'")
		execute_process(                                       # checkout to v2.0
			COMMAND tar xf ${JSONCPP_TMP_FILE}
			WORKING_DIRECTORY ${THIRD_PARTY_LIBRARIES}
			RESULT_VARIABLE JSON_RESULT
			ERROR_VARIABLE JSON_ERROR
			)
		if (${JSON_RESULT} EQUAL 0)
			message("-- CMaking ...")
			file(MAKE_DIRECTORY "${JSONCPP_DIR}/build")
			file(MAKE_DIRECTORY "${JSONCPP_DIR}/build/debug")
			file(MAKE_DIRECTORY "${JSONCPP_DIR}/install")
			file(MAKE_DIRECTORY "${JSONCPP_DIR}/install/debug")
			execute_process(
				COMMAND cmake -DCMAKE_INSTALL_PREFIX=../../install/debug -DBUILD_SHARED_LIBS=ON -config Debug ../..
				WORKING_DIRECTORY "${JSONCPP_DIR}/build/debug"
				OUTPUT_VARIABLE CMAKE_OUTPUT
				RESULT_VARIABLE CMAKE_RESULT
			)
			if (${CMAKE_RESULT} EQUAL 0)
				message("-- Making ...")
				execute_process(
					COMMAND make install
					WORKING_DIRECTORY "${JSONCPP_DIR}/build/debug"
					OUTPUT_VARIABLE MAKE_OUTPUT
					RESULT_VARIABLE MAKE_RESULT
				)
				if (MAKE_RESULT GREATER 0)
					file(REMOVE_RECURSE ${JSONCPP_DIR})
					message(FATAL_ERROR "Cannot make JSON-CPP.")
				else()
					message("-- Success!")
				endif()
			else()
				file(REMOVE_RECURSE ${JSONCPP_DIR})
				message(FATAL_ERROR "Cannot cmake JSON-CPP.")
			endif()
		else()
			message(FATAL_ERROR "Cannot extract JSON-CPP.")
		endif()
	endif()
endif()

find_library(jsoncpp jsoncpp
	HINTS "${JSONCPP_DIR}/install/debug/lib"
	NO_DEFAULT_PATH
	)
if (${jsoncpp} STREQUAL "jsoncpp-NOTFOUND")
	# tries to find the default installation.
	find_library(jsoncpp jsoncpp)
endif()
if (${jsoncpp} STREQUAL "jsoncpp-NOTFOUND")
	message(FATAL_ERROR "Cannot find jsoncpp at '${JSONCPP_DIR}/install/debug/lib'.")
else()
	message("-- JSONCPP found at ${jsoncpp}")
	set(JSONCPP_INCLUDE_DIR "${JSONCPP_DIR}/install/debug/include")
	message("-- Include directory: ${JSONCPP_INCLUDE_DIR}")
	include_directories(${JSONCPP_INCLUDE_DIR})
endif()
message("")

message("Boost")
find_package(Boost ${BOOST_VERSION} COMPONENTS ${BOOST_REQUIRED_COMPONENTS})
if (NOT ${Boost_FOUND})
	if (NOT EXISTS ${BOOST_DIR})
		message("-- Cannot find ${BOOST_DIR}")
		set(BOOST_FILE_URL http://downloads.sourceforge.net/project/boost/boost/${BOOST_VERSION}/boost_${BOOST_VERSION_UNDERSCORE}.tar.bz2)
		set(BOOST_TMP_FILE /tmp/libboost.tar.bz2)
		download(BOOST_RESULT ${BOOST_FILE_URL} ${BOOST_DOWNLOAD_MD5} ${BOOST_TMP_FILE})
		if (${BOOST_RESULT} STREQUAL "ERROR")
			message(FATAL_ERROR "Cannot get ${BOOST_FILE_URL}.")
		else()
			message("-- OK")
		endif()
		message("-- Extracting to ${THIRD_PARTY_LIBRARIES} ...")
		execute_process(
			COMMAND tar xf ${BOOST_TMP_FILE}
			WORKING_DIRECTORY ${THIRD_PARTY_LIBRARIES}
			RESULT_VARIABLE TAR_RESULT
			OUTPUT_VARIABLE TARGET_OUTPUT
		)
		if (${TAR_RESULT} EQUAL "0")
			message("-- OK")
		else()
			file(REMOVE_RECURSE ${BOOST_DIR})
			message(FATAL_ERROR "Cannot extract libboost.")
		endif()
	endif()

	if (NOT EXISTS ${BOOST_DIR}/b2)
		message("-- Bootstrapping libboost")
		execute_process(
			COMMAND sh bootstrap.sh
			WORKING_DIRECTORY ${BOOST_DIR}
			RESULT_VARIABLE BOOST_RESULT
		)
		if (${BOOST_RESULT} EQUAL 0)
			message("-- Ok")
		else()
			file(REMOVE_RECURSE "${BOOST_DIR}/install")
			message(FATAL_ERROR "Cannot extract libboost.")
		endif()
	endif()
	if (NOT EXISTS ${BOOST_DIR}/install)
		message("-- Compiling and installing libboost")
		file(MAKE_DIRECTORY ${BOOST_DIR}/install)
		execute_process(
			COMMAND "${BOOST_DIR}/b2" "--prefix=${BOOST_DIR}/install" install
			WORKING_DIRECTORY ${BOOST_DIR}
			RESULT_VARIABLE BOOST_RESULT
		)
		if (${BOOST_RESULT} GREATER 0)
			file(REMOVE_RECURSE "${BOOST_DIR}/install")
			message(FATAL_ERROR "Cannot build libboost. ${BOOST_OUTPUT}")
		endif()
	endif()
	set(BOOST_ROOT "${BOOST_DIR}/install/")
	find_package(Boost ${BOOST_VERSION} REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})
	message("-- Include dir: ${Boost_INCLUDE_DIRS}")
	include_directories(${Boost_INCLUDE_DIRS})
	message("-- Libraries: ${Boost_LIBRARIES}")
endif()
message("")

message("GTest")
if (NOT EXISTS ${GTEST_DIR})
	message("-- Downloading GTest (${GTEST_URL})")
	set(GTEST_TMP_FILE /tmp/gtest.tar.gz)
	download(GTEST_RESULT ${GTEST_URL} ${GTEST_MD5} ${GTEST_TMP_FILE})
	if (${GTEST_RESULT} EQUAL "ERROR")
		message(FATAL_ERROR "Cannot download GTest.")
	else()
		message("-- Extracting Google Test to ${GTEST_DIR} ...")
		execute_process(                                         # checkout to v2.0
			COMMAND tar xf ${GTEST_TMP_FILE}
			WORKING_DIRECTORY ${THIRD_PARTY_LIBRARIES}
			RESULT_VARIABLE GTEST_RESULT
			ERROR_VARIABLE GTEST_ERROR
			)
		if (${GTEST_RESULT} GREATER 0)                           # git checkout failed
			file(REMOVE_RECURSE ${GTEST_DIR})
			message(FATAL_ERROR "Cannot extract GTest: ${GTEST_ERROR}.")
		else()                                                   # git checkout success
			file(MAKE_DIRECTORY "${GTEST_DIR}/build")
			file(MAKE_DIRECTORY "${GTEST_DIR}/build/debug")
			message("-- CMaking ...")
			execute_process(
				COMMAND cmake -config Debug ../..
				WORKING_DIRECTORY "${GTEST_DIR}/build/debug"
				OUTPUT_VARIABLE CMAKE_OUTPUT
				RESULT_VARIABLE CMAKE_RESULT
			)
			if (${CMAKE_RESULT} EQUAL 0)
				message("-- Making ...")
				execute_process(
					COMMAND make
					WORKING_DIRECTORY "${GTEST_DIR}/build/debug"
					OUTPUT_VARIABLE MAKE_OUTPUT
					RESULT_VARIABLE MAKE_RESULT
				)
				if (MAKE_RESULT GREATER 0)
					file(REMOVE_RECURSE ${GTEST_DIR})
					message(FATAL_ERROR "Cannot make GTest.")
				else()
					message("-- Success!")
				endif()
			else()
				file(REMOVE_RECURSE ${GTEST_DIR})
				message(FATAL_ERROR "Cannot cmake GTest.")
			endif()
		endif()
	endif()
endif()
message("")

set(Walking_LIBRARIES
	jsoncpp
	pthread
	${Boost_LIBRARIES}
)

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

	src/engine/ik.cpp
	src/engine/ik.h

	src/configuration/configuration.cpp
	src/configuration/configuration.h

	src/data/angle.cpp
	src/data/angle.h

	src/sensors/imu.cpp
	src/sensors/imu.h

	src/sensors/gyro.cpp
	src/sensors/gyro.h

	src/sensors/mpu.cpp
	src/sensors/mpu.h

	src/motors/servo.cpp
	src/motors/servo.h
	src/consts.h
)
add_executable(walking ${SOURCE_FILES})
target_link_libraries(walking
	${Walking_LIBRARIES}
)

if (${ENABLE_TEST})
	message("Tests")
	enable_testing()
	include_directories(${CMAKE_SOURCE_DIR}/src)
	file(GLOB_RECURSE TEST_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} test/src/*_test.cpp)

	add_executable(
		run_tests

		# test/src/resources.h
		# test/src/resources.cpp

		${TEST_SRCS}
		${SOURCE_FILES}
	)
	set_target_properties(run_tests PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT}
		)
	target_link_libraries(run_tests
		gtest gtest_main
		${Walking_LIBRARIES}
		)
	foreach (testSrc ${TEST_SRCS})
		# get_filename_component(testName ${testSrc} NAME_WE)
		string(REPLACE "test/src/" "" testPackage "${testSrc}")
		string(REPLACE "_test.cpp" "" testPackage "${testPackage}")
		string(REPLACE "/" "_" testPackage "${testPackage}")
		message("-- ${testPackage}: ${testSrc}")
		add_test(
			NAME ${testPackage}
			WORKING_DIRECTORY ${TEST_OUTPUT}
			COMMAND run_tests --gtest_filter=${testPackage}.*
		)
	endforeach()
	message("")
endif()