# function(add_component_test COMPONENT_TARGET_NAME ...)

#     if(NOT WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
#         return()
#     endif()
#     list(REMOVE_AT ARGV 0)

#     message(STATUS "Add test files for component ${COMPONENT_TARGET_NAME}: ${ARGV}")

#     target_sources(${WMTK_COMPONENT_TEST_TARGET} PUBLIC ${ARGV})
#     target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PRIVATE ${COMPONENT_TARGET_NAME} wmtk_test_tools)

# endfunction()

function(add_component_test COMPONENT_NAME)
    set(COMPONENT_TEST_NAME wmtk_test_${COMPONENT_NAME})

    option(WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME} "Enable wmtk component test for ${COMPONENT_NAME}" ON)
    
    IF(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
        return()
    ENDIF()

    message(STATUS "Add component test ${COMPONENT_TEST_NAME}.")

    add_executable(${COMPONENT_TEST_NAME})
    target_link_libraries(${COMPONENT_TEST_NAME} PRIVATE
        wmtk::${COMPONENT_NAME}
        Catch2::Catch2WithMain
        wmtk::warnings 
        wmtk::toolkit
    )

    # Group source files for IDEs
    file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

    set_target_properties(${COMPONENT_TEST_NAME} PROPERTIES FOLDER wmtk_components_tests)

endfunction()
