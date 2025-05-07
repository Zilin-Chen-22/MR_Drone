aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ flight_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${flight_src_list})