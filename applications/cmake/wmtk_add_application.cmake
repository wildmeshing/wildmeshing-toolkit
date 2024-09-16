function(wmtk_add_application APP_NAME ...)

    list(REMOVE_AT ARGV 0)
    add_executable(${APP_NAME} ${ARGV})

    set_target_properties(${APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/applications)

    target_link_libraries(${APP_NAME} PRIVATE wmtk::toolkit wmtk::warnings CLI11::CLI11)

    # Group source files for IDEs
    file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

    set_target_properties(${APP_NAME} PROPERTIES FOLDER wmtk_applications)

    if(MSVC)
        target_compile_options(${APP_NAME} PUBLIC /MP)
        target_compile_options(${APP_NAME} PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/GL>)
        target_compile_options(${APP_NAME} PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ob2>) # inline whenever suitable
        target_compile_options(${APP_NAME} PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ot>) # favor faster code over binary size
        target_link_options(${APP_NAME} PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/LTCG>)
        target_link_options(${APP_NAME} PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/INCREMENTAL:NO>)
        target_compile_options(${APP_NAME} PUBLIC $<$<CONFIG:RELEASE>:/GL>)
        target_compile_options(${APP_NAME} PUBLIC $<$<CONFIG:RELEASE>:/Ot>)
        target_link_options(${APP_NAME} PRIVATE $<$<CONFIG:RELEASE>:/LTCG>)
        target_link_options(${APP_NAME} PRIVATE $<$<CONFIG:RELEASE>:/INCREMENTAL:NO>)
    endif()

endfunction()
