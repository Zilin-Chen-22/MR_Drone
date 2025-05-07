aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ pg_src_list)
target_sources(${STATIC_LIB_TARGET} PRIVATE ${pg_src_list})