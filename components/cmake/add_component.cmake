function(add_component COMPONENT_NAME)
    set(COMPONENT_TARGET_NAME wmtk_${COMPONENT_NAME})


    option(WMTK_ENABLE_COMPONENT_${COMPONENT_NAME} "Enable wmtk component ${COMPONENT_NAME}" ON)

    IF(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
        return()
    ENDIF()

    add_library(${COMPONENT_TARGET_NAME})
    add_library(wmtk::${COMPONENT_NAME} ALIAS ${COMPONENT_TARGET_NAME})


    target_link_libraries(${COMPONENT_TARGET_NAME} PRIVATE wmtk::warnings wmtk::toolkit)

    target_include_directories(${COMPONENT_TARGET_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/${COMPONENT_NAME}")

    add_subdirectory("${COMPONENT_NAME}/wmtk/components/${COMPONENT_NAME}")

    target_link_libraries(wildmeshing_components PUBLIC ${COMPONENT_TARGET_NAME})


    # Group source files for IDEs
    file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components/${COMPONENT_NAME}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components/${COMPONENT_NAME}/*.hpp")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components/${COMPONENT_NAME}/wmtk/components/${COMPONENT_NAME}" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

endfunction()
