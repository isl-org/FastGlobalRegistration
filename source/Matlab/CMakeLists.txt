# compile mex
file(GLOB files ${FastGlobalRegistration_SOURCE_DIR}/Matlab/*.cpp)
foreach(CPP_FULL_PATH ${files})
  get_filename_component(CPP_FILE ${CPP_FULL_PATH} NAME_WE)

  set(CPP_FILE_NAME ${CPP_FILE}.cpp)
  INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR})
  add_library(${CPP_FILE} SHARED ${CPP_FILE_NAME} ${FastGlobalRegistration_SOURCE_DIR}/Cmake/Matlabdef.def)
  target_link_libraries(${CPP_FILE} FastGlobalRegistrationLib ${MATLAB_LIBRARIES})

  # 32-bit or 64-bit mex
  if(WIN32)
    if (CMAKE_CL_64)
        SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw64)
    else(CMAKE_CL_64)
        SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw32)
    endif(CMAKE_CL_64)
  elseif(APPLE)
    if (CMAKE_SIZEOF_VOID_P MATCHES "8")
        SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexmaci64 PREFIX "")
    else(CMAKE_SIZEOF_VOID_P MATCHES "8")
        SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexmaci PREFIX "")
    endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
  elseif(UNIX)
    if (CMAKE_SIZEOF_VOID_P MATCHES "8")
        SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexa64 PREFIX "")
    else(CMAKE_SIZEOF_VOID_P MATCHES "8")
        SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexglx PREFIX "")
    endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
  endif(WIN32)

  # Install it directly to the Matlab folder
  set_target_properties(${CPP_FILE} PROPERTIES
           LIBRARY_OUTPUT_DIRECTORY ${FastGlobalRegistration_SOURCE_DIR}/Matlab)

endforeach()