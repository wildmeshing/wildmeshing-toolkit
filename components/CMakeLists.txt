
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/components/cmake/recipes")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/components/cmake")

include(add_component)
include(add_component_test)

# wmtk::component_utils
add_subdirectory(utils)

if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    add_subdirectory(tests)
endif()

# list of all components
add_subdirectory("input/wmtk/components/input")
add_subdirectory("output/wmtk/components/output")
add_subdirectory("delaunay/wmtk/components/delaunay")
add_subdirectory("to_points/wmtk/components/to_points")

add_subdirectory("multimesh_from_tag/wmtk/components/multimesh_from_tag")
add_subdirectory("triangle_insertion/wmtk/components/triangle_insertion")

add_subdirectory("marching/wmtk/components/marching")
add_subdirectory("procedural/wmtk/components/procedural")

add_subdirectory("tetwild_simplification/wmtk/components/tetwild_simplification")
add_subdirectory("isotropic_remeshing/wmtk/components/isotropic_remeshing")

# add_component("export_cache")
# add_component("import_cache")
# add_component("mesh_info")
# add_component("multimesh")
# add_component("regular_space")
# add_component("tag_intersection")
# add_component("wildmeshing")
# add_component("get_all_meshes")
# add_component("fusion")
# add_component("winding_number")
# add_component("periodic_optimization")
# add_component("shortestedge_collapse")






