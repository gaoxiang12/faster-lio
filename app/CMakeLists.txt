add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        )
