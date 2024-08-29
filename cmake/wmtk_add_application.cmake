function(wmtk_add_application APP_NAME ...)

    message(STATUS "Args: ${ARGV}")
    list(REMOVE_AT ARGV 0)
    message(STATUS "SRCS: ${ARGV}")
    add_executable(${APP_NAME} ${ARGV})

    set_target_properties(${APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/applications)


    target_link_libraries(${APP_NAME} PRIVATE wmtk::toolkit CLI11::CLI11)


endfunction()
