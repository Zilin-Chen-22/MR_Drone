set(STATIC_LIB_TARGET pid_betaflight) # specify the name of the static library

add_library(${STATIC_LIB_TARGET} STATIC "") # define the static library


target_link_libraries(${STATIC_LIB_TARGET} PUBLIC ${LASAN})
target_include_directories(${STATIC_LIB_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}) # append include directories to the library

#if PUBLIC is used, then the targets that depend on this library have access to the specified directory
target_compile_options(${STATIC_LIB_TARGET} PRIVATE ${GNU_FLAGS})

add_subdirectory(fc)
add_subdirectory(flight)
add_subdirectory(sensors)
add_subdirectory(SITL)
add_subdirectory(common)
add_subdirectory(rx)
add_subdirectory(build)
add_subdirectory(simio)
add_subdirectory(pg)
add_subdirectory(drivers)
add_subdirectory(config)