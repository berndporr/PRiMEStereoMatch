cmake_minimum_required (VERSION 3.20)

add_executable(PRiMEStereoMatch PRiMEStereoMatch.cpp StereoMatch.cpp)

TARGET_LINK_LIBRARIES(PRiMEStereoMatch prime_stereo_match)
