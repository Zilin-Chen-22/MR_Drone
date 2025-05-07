aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ sensors_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${sensors_src_list})