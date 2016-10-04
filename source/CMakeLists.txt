cmake_minimum_required(VERSION 3.0.0)

project(FastGlobalRegistration)

set(FastGlobalRegistration_VERSION_MAJOR "1")
set(FastGlobalRegistration_VERSION_MINOR "0")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# set include directories
include_directories(
	${FastGlobalRegistration_SOURCE_DIR}
	${FastGlobalRegistration_SOURCE_DIR}/External
	${FastGlobalRegistration_SOURCE_DIR}/External/Eigen
	${FastGlobalRegistration_SOURCE_DIR}/External/flann
	)

# default built type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Set OS-specific things here
if(WIN32)
	add_definitions(-DWINDOWS)
	message(STATUS "Compiling on Windows")
	if(MSVC)
		message(STATUS "Compiling with MSVC")
		add_definitions(-DNOMINMAX)
		add_definitions(-D_USE_MATH_DEFINES)
	endif(MSVC)
elseif(CYGWIN)
	message(STATUS "Compiling on Cygwin")
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCYGWIN)
elseif(APPLE)
	add_definitions(-DUNIX)
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	# compile speed optimization for clang
	add_definitions(-Os)
	# disable OpenMP since it is not currently supported on OSX
	message(STATUS "Compiling on OSX")
elseif(UNIX)
	add_definitions(-DUNIX)
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(WIN32)

add_subdirectory(FastGlobalRegistration)
