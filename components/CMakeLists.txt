
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/components/cmake/recipes")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/components/cmake")



set(WMTK_COMPONENT_PREFIX "wmtkc" )


include(delaunay_psm)
include(jse)
include(stb)

include(add_component)

#library to link all components libs to
add_library(wildmeshing_components INTERFACE)
add_library(wmtk::components ALIAS wildmeshing_components)
target_include_directories(wildmeshing_components INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#commont stuff for all components
# add_library("${WMTK_COMPONENT_PREFIX}_base")
# add_library(${WMTK_COMPONENT_PREFIX}::base ALIAS ${WMTK_COMPONENT_PREFIX}_base)
# add_subdirectory("wmtk_components/base")


#clear the files
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/components_include.hpp" "")
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/components_map.hpp" "")
set(json_components "")
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/spec_include.hpp"
"spec_engine.include_directories.push_back( \"${CMAKE_CURRENT_SOURCE_DIR}\");\n"
)


add_component(${WMTK_COMPONENT_PREFIX} "input")
add_component(${WMTK_COMPONENT_PREFIX} "output")
add_component(${WMTK_COMPONENT_PREFIX} "delaunay")
add_component(${WMTK_COMPONENT_PREFIX} "isotropic_remeshing")
add_component(${WMTK_COMPONENT_PREFIX} "marching")
add_component(${WMTK_COMPONENT_PREFIX} "mesh_info")
add_component(${WMTK_COMPONENT_PREFIX} "multimesh")
add_component(${WMTK_COMPONENT_PREFIX} "regular_space")
add_component(${WMTK_COMPONENT_PREFIX} "tag_intersection")
add_component(${WMTK_COMPONENT_PREFIX} "wildmeshing")

string(LENGTH ${json_components} json_components_length)
math(EXPR json_components_length "${json_components_length}-2")
string(SUBSTRING ${json_components} 0 ${json_components_length} json_components)
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/components.json" "[\n${json_components}\n]")




# Group source files for IDEs
# file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "*.cpp" "*.h" "*.hpp")
# source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components" PREFIX "components" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})



if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    # Unit tests
    enable_testing()

    add_subdirectory(tests)


    add_executable(wmtk_app main.cpp)
    include(jse)
    include(cli11)

    # ###############################################################################
    # WMTK JSON APP
    # ###############################################################################
    target_compile_options(wmtk_app PRIVATE "-rdynamic")
    target_link_libraries(wmtk_app PUBLIC
        wmtk::toolkit
        wmtk::components
        CLI11::CLI11
        jse::jse
    )

    target_compile_definitions(wmtk_app PUBLIC WMTK_APP_INPUT_SPEC="${CMAKE_SOURCE_DIR}/components/wmtk_spec.json")
endif()






