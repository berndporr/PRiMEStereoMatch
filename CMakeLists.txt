if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

cmake_minimum_required (VERSION 3.20)
project (PRiMEStereoMatch VERSION 2.1 LANGUAGES CXX)

add_compile_options(-Wall -pedantic)

include(GNUInstallDirs)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(${OpenMP_INCLUDE_DIRS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

message(STATUS "opencv:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(.)

set(LIBSRC
	PrimeStereoMatch/CVC.cpp
	PrimeStereoMatch/DispSel.cpp
	PrimeStereoMatch/PP.cpp
	PrimeStereoMatch/CVF.cpp
	PrimeStereoMatch/fastguidedfilter.cpp
	PrimeStereoMatch/PrimeStereoMatch.cpp
)

set(LIBINCLUDE
	PrimeStereoMatch/ComFunc.h
	PrimeStereoMatch/DispSel.h
	PrimeStereoMatch/PP.h
	PrimeStereoMatch/CVC.h
	PrimeStereoMatch/fastguidedfilter.h
	PrimeStereoMatch/CVF.h
	PrimeStereoMatch/JointWMF.h
)

add_library(prime_stereo_match STATIC ${LIBSRC})
TARGET_LINK_LIBRARIES(prime_stereo_match pthread ${OpenCV_LIBS})

add_subdirectory(demo)

target_include_directories(prime_stereo_match
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/iir>
)

set_target_properties(prime_stereo_match PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  VERSION ${PROJECT_VERSION}
  PUBLIC_HEADER PrimeStereoMatch.h
  PRIVATE_HEADER "${LIBINCLUDE}")

install(TARGETS prime_stereo_match EXPORT prime_stereo_match-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PrimeStereoMatch)
