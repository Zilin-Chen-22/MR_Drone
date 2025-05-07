aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ simio_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${simio_src_list})