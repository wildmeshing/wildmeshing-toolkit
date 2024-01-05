function(add_component WMTK_COMPONENT_PREFIX folder)
    set(CURRENT_COMPONENT_LIB_NAME "${WMTK_COMPONENT_PREFIX}_${folder}")

    add_library(${CURRENT_COMPONENT_LIB_NAME})
    add_library(${WMTK_COMPONENT_PREFIX}::${folder} ALIAS ${CURRENT_COMPONENT_LIB_NAME})
    target_include_directories(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtk::warnings wmtk::toolkit)
    # target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE "${WMTK_COMPONENT_PREFIX}::base")

    add_subdirectory("wmtk_components/${folder}")

    file(APPEND "${CMAKE_CURRENT_SOURCE_DIR}/components_include.hpp"
     "#include \"wmtk_components/${folder}/${folder}.hpp\"\n")

    file(APPEND "${CMAKE_CURRENT_SOURCE_DIR}/components_map.hpp"
     "components[\"${folder}\"] = wmtk::components::${folder};\n")



    target_link_libraries(wildmeshing_components INTERFACE ${CURRENT_COMPONENT_LIB_NAME})
endfunction()