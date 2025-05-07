aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ rx_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${rx_src_list})