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
    ${FastGlobalRegistration_SOURCE_DIR}/FastGlobalRegistration
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
		add_definitions(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)		# suppress C4996 warning
		add_definitions(/wd4267 /wd4244 /wd4305 /wd4522)
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
	add_compile_options(-Wno-deprecated-declarations)
	add_compile_options(-Wno-unused-result)
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(WIN32)

add_subdirectory(FastGlobalRegistration)


# Matlab Mex
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/Cmake) # add FindMatlab module

add_definitions(/DMATLAB_MEX_FILE) #define matlab macros
add_definitions(/DMX_COMPAT_32)

find_package(Matlab REQUIRED)

IF(MATLAB_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")
    add_subdirectory(Matlab)
ELSE(MATLAB_FOUND)
    MESSAGE("MATLAB not found...nothing will be built.")
ENDIF(MATLAB_FOUND)
