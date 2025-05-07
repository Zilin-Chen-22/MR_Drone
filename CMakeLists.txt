cmake_minimum_required(VERSION 3.16)

# set the project name and version
project(betaflight_sim VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if(Eigen3_FOUND)
  message(STATUS "eigen3 found")
endif()

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message("build type is debug")
    # set(GNU_FLAGS "-Wall;-Werror;-Wextra;-O0;-g;-fno-omit-frame-pointer;-fsanitize=address")
    set(GNU_FLAGS "-Wall;-Wextra;-O0;-g;-fno-omit-frame-pointer;-fsanitize=address")
    set(LASAN -lasan)
else()
    message("build type is release")
    set(GNU_FLAGS "-O3")
    set(LASAN "") # no lasan lib is add in release mode
endif()

add_subdirectory(betaflight_c) 
add_subdirectory(l1_adaptive_c)


include(FetchContent)
FetchContent_Declare(
  mdss
  GIT_REPOSITORY https://github.com/LonghaoQian/mdss)
  # add tag if needed   GIT_TAG release-1.12.1
# For Windows: Prevent overriding the parent project's compiler/linker
# settings
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)
FetchContent_GetProperties(mdss)
if(NOT mdss_POPULATED)
  FetchContent_Populate(mdss)
  message(STATUS ${mdss_SOURCE_DIR})
  message(STATUS ${mdss_BUILD_DIR})
  add_subdirectory(${mdss_SOURCE_DIR} ${mdss_BUILD_DIR})
endif()

# fetch mdss repo
# add_subdirectory(thirdparty/mdss)

add_executable(test1 testfile.cpp)
target_link_libraries(test1 PUBLIC ${LASAN} pid_betaflight)
target_compile_options(test1 PUBLIC ${GNU_FLAGS})

add_executable(pid_test pid_test.cpp)
target_link_libraries(pid_test PUBLIC
                            ${LASAN}
                            pid_betaflight
                            -Wl,--whole-archive mdss -Wl,--no-whole-archive
                            utility_functions)
target_compile_options(pid_test PUBLIC ${GNU_FLAGS})

add_executable(adv_pid_test adv_pid_test.cpp)
target_link_libraries(adv_pid_test PUBLIC
                            ${LASAN}
                            advanced_rate_control
                            -Wl,--whole-archive mdss -Wl,--no-whole-archive
                            utility_functions)
target_compile_options(adv_pid_test PUBLIC ${GNU_FLAGS})
