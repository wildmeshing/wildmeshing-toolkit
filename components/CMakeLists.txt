
include(wmtk_data)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/components/cmake")

option(WMTK_COMPONENT_TESTS "Generate component unit tests." ON)

include (jse)

include(add_component)
include(add_component_test)

# list of all components
add_subdirectory("shortest_edge_collapse/wmtk/components/shortest_edge_collapse")
add_subdirectory("isotropic_remeshing/wmtk/components/isotropic_remeshing")
add_subdirectory("qslim/wmtk/components/qslim")
add_subdirectory("tetwild/wmtk/components/tetwild")
add_subdirectory("triwild/wmtk/components/triwild")
add_subdirectory("simwild/wmtk/components/simwild")
add_subdirectory("c1_simplification/wmtk/components/c1_simplification")
add_subdirectory("manifold_extraction/wmtk/components/manifold_extraction")
add_subdirectory("topological_offset/wmtk/components/topological_offset")

# Optional: links polyfem. Gated by WMTK_WITH_POLYFEM (top-level CMakeLists) so the default build
# never downloads or compiles it.
if(WMTK_WITH_POLYFEM)
    add_subdirectory("polyfem_ops/wmtk/components/polyfem_ops")
endif()

# The component registry that wmtk_app, the Python module and the integration tests compile in:
# one #include and one map entry per component add_component registered above. It is generated in
# this build's own tree because the set of components is a per-build choice (WMTK_WITH_POLYFEM,
# WMTK_ENABLE_COMPONENT_*). Generated into the source tree, it was shared by every build folder of
# a clone and the last configure decided it for all of them: a folder configured without
# polyfem_ops made the next build of a folder with it compile a registry lacking polyfem_ops.
# file(CONFIGURE) rewrites a file only when its content changes, so an unchanged reconfigure
# recompiles nothing.
set(WMTK_AUTOGEN_COMMENT "// This file was auto-generated with CMake\n")
set(WMTK_COMPONENTS_INCLUDE "${WMTK_AUTOGEN_COMMENT}")
set(WMTK_COMPONENTS_MAP "${WMTK_AUTOGEN_COMMENT}")
get_property(WMTK_REGISTERED_COMPONENTS GLOBAL PROPERTY WMTK_REGISTERED_COMPONENTS)
foreach(COMPONENT_NAME IN LISTS WMTK_REGISTERED_COMPONENTS)
    string(APPEND WMTK_COMPONENTS_INCLUDE
        "#include <wmtk/components/${COMPONENT_NAME}/${COMPONENT_NAME}.hpp>\n")
    string(APPEND WMTK_COMPONENTS_MAP
        "components_map[\"${COMPONENT_NAME}\"] = wmtk::components::${COMPONENT_NAME}::${COMPONENT_NAME};\n")
endforeach()
file(CONFIGURE OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/components_include.hpp"
    CONTENT "${WMTK_COMPONENTS_INCLUDE}" @ONLY)
file(CONFIGURE OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/components_map.hpp"
    CONTENT "${WMTK_COMPONENTS_MAP}" @ONLY)

target_include_directories(wmtk_app PRIVATE "${WMTK_GENERATED_INCLUDE_DIR}")
if(WMTK_PYBIND)
    target_include_directories(wildmeshing PRIVATE "${WMTK_GENERATED_INCLUDE_DIR}")
endif()
if(WMTK_BUILD_INTEGRATION_TESTS)
    target_include_directories(wmtk_integration_tests PUBLIC "${WMTK_GENERATED_INCLUDE_DIR}")
endif()
