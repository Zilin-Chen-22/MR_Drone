aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ build_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${build_src_list})