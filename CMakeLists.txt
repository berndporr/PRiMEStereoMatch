if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

cmake_minimum_required (VERSION 3.20)
project (PRiMEStereoMatch VERSION 2.1 LANGUAGES CXX)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(${OpenMP_INCLUDE_DIRS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(.)

set(LIBSRC
	prime_stereo_match/CVC.cpp
	prime_stereo_match/DispSel.cpp
	prime_stereo_match/PP.cpp
	prime_stereo_match/CVF.cpp
	prime_stereo_match/fastguidedfilter.cpp
	prime_stereo_match/Prime_stereo_match.cpp
)

set(LIBINCLUDE
	prime_stereo_match/ComFunc.h
	prime_stereo_match/DispSel.h
	prime_stereo_match/PP.h
	prime_stereo_match/CVC.h
	prime_stereo_match/fastguidedfilter.h
	prime_stereo_match/CVF.h
	prime_stereo_match/JointWMF.h
)

add_library(prime_stereo_match STATIC ${LIBSRC})
TARGET_LINK_LIBRARIES(prime_stereo_match pthread ${OpenCV_LIBS})

target_include_directories(prime_stereo_match
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/prime_stereo_match>
)

set_target_properties(prime_stereo_match PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  VERSION ${PROJECT_VERSION}
  PUBLIC_HEADER prime_stereo_match.h
  PRIVATE_HEADER "${LIBINCLUDE}")

install(TARGETS prime_stereo_match EXPORT prime_stereo_match-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/prime_stereo_match)

install(EXPORT prime_stereo_match-targets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/prime_stereo_match
  FILE prime_stereo_match.cmake
)

add_subdirectory(demo)
