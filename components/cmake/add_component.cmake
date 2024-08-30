function(add_component COMPONENT_NAME)
    set(COMPONENT_TARGET_NAME wmtk_${COMPONENT_NAME})

    option(WMTK_ENABLE_COMPONENT_${COMPONENT_NAME} "Enable wmtk component ${COMPONENT_NAME}" ON)

    IF(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
        return()
    ENDIF()

    message(STATUS "Add component wmtk::${COMPONENT_NAME}.")

    add_library(${COMPONENT_TARGET_NAME})
    add_library(wmtk::${COMPONENT_NAME} ALIAS ${COMPONENT_TARGET_NAME})


    target_link_libraries(${COMPONENT_TARGET_NAME} PRIVATE wmtk::warnings wmtk::toolkit wmtk::component_utils)
    target_link_libraries(${COMPONENT_TARGET_NAME} PUBLIC wmtk::component_utils)

    target_include_directories(${COMPONENT_TARGET_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

    # Group source files for IDEs
    file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

    set_target_properties(${COMPONENT_TARGET_NAME} PROPERTIES FOLDER wmtk_components)

endfunction()
