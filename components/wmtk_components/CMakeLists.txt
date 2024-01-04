set(SRC_FILES
    json_utils.hpp
)
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})

add_subdirectory(adaptive_tessellation)
add_subdirectory(delaunay)
add_subdirectory(wildmeshing)
add_subdirectory(input)
add_subdirectory(isotropic_remeshing)
add_subdirectory(mesh_info)
add_subdirectory(output)
add_subdirectory(regular_space)
add_subdirectory(tag_intersection)
add_subdirectory(marching)
add_subdirectory(extreme_opt)