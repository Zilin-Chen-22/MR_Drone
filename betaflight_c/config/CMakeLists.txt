aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ config_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${config_src_list})