function(add_component WMTK_COMPONENT_PREFIX folder)
    set(CURRENT_COMPONENT_LIB_NAME "${WMTK_COMPONENT_PREFIX}_${folder}")

    option(WMTK_ENABLE_COMPONENT_${folder} "" ON)

    IF(NOT ${WMTK_ENABLE_COMPONENT_${folder}})
        return()
    ENDIF()

    add_library(${CURRENT_COMPONENT_LIB_NAME})
    add_library(${WMTK_COMPONENT_PREFIX}::${folder} ALIAS ${CURRENT_COMPONENT_LIB_NAME})


    target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtk::warnings wmtk::toolkit)
    target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE "${WMTK_COMPONENT_PREFIX}::base")
    target_include_directories(${CURRENT_COMPONENT_LIB_NAME} PUBLIC "wmtk_components/${folder}")

    add_subdirectory("wmtk_components/${folder}")

    file(APPEND "${CMAKE_CURRENT_SOURCE_DIR}/components_include.hpp"
     "#include \"wmtk_components/${folder}/${folder}.hpp\"\n")

    file(APPEND "${CMAKE_CURRENT_SOURCE_DIR}/components_map.hpp"
     "components[\"${folder}\"] = wmtk::components::${folder};\n")

    set(json_components
     "${json_components}{\"pointer\":\"/${folder}\",  \"type\": \"include\", \"spec_file\": \"${folder}_spec.json\"},\n" PARENT_SCOPE)


    file(APPEND "${CMAKE_CURRENT_SOURCE_DIR}/spec_include.hpp"
     "spec_engine.include_directories.push_back( \"${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components/${folder}\");\n")

    set(WMTKC_TEST_SOURCES "test_component_${folder}.cpp" ${WMTKC_TEST_SOURCES} PARENT_SCOPE)
    target_link_libraries(wildmeshing_components INTERFACE ${CURRENT_COMPONENT_LIB_NAME})


    # Group source files for IDEs
    file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components/${folder}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components/${folder}/*.hpp")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components/${folder}" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

endfunction()