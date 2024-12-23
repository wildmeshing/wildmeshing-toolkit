function(add_component_test COMPONENT_TARGET_NAME ...)

    if(NOT WMTK_ENABLE_COMPONENT_TESTS)
        return()
    endif()
    list(REMOVE_AT ARGV 0)

    message(STATUS "Add test files for component ${COMPONENT_TARGET_NAME}: ${ARGV}")

    target_sources(${WMTK_COMPONENT_TEST_TARGET} PUBLIC ${ARGV})
    target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PRIVATE ${COMPONENT_TARGET_NAME} wmtk_test_tools)

endfunction()
