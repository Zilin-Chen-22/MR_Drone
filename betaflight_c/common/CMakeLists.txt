target_include_directories(${STATIC_LIB_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ common_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${common_src_list})