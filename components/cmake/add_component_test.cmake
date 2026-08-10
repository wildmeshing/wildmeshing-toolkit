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
    # Unit tests
    if(NOT BUILD_TESTING)
        return()
    endif()

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
        wmtk::data
    )

    # Group source files for IDEs
    file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

    set_target_properties(${COMPONENT_TEST_NAME} PROPERTIES FOLDER wmtk_components_tests)

    # Register with ctest. Without this the executable is built and never run, by CI or by
    # anyone typing `ctest` locally -- which is how two suites came to sit broken on main
    # (tetwild's surface-swap case and three shortest_edge_collapse cases), each failing
    # against behaviour that had changed underneath it while nothing was watching.
    #
    # One entry per component rather than catch_discover_tests: that helper lives in Catch2's
    # `extras` and needs the module path set up, which is done in tests/ and is not visible in
    # this directory scope. The whole set runs in a few seconds, so per-case granularity is
    # not worth the extra plumbing here.
    # --allow-running-no-tests because c1_simplification's two cases are currently commented
    # out, so its executable holds nothing and Catch2 exits non-zero on an empty run. Without
    # the flag that component alone would fail the moment these are registered.
    add_test(NAME ${COMPONENT_TEST_NAME} COMMAND ${COMPONENT_TEST_NAME} --allow-running-no-tests)
    wmtk_copy_dll(${COMPONENT_TEST_NAME})

endfunction()
