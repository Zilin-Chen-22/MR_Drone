aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ fc_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${fc_src_list})