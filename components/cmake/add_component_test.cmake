function(add_component_test ...)
    
    if(NOT WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
        return()
    endif()

    message(STATUS "Add test files for component ${COMPONENT_TARGET_NAME}: ${ARGV}")

    target_sources(${WMTK_COMPONENT_TEST_TARGET} PUBLIC ${ARGV})
    target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PRIVATE ${COMPONENT_TARGET_NAME})

endfunction()
