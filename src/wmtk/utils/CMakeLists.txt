
set(SRC_FILES
    Logger.cpp
    Logger.hpp
    edgemesh_topology_initialization.h
    edgemesh_topology_initialization.cpp
    trimesh_topology_initialization.h
    trimesh_topology_initialization.cpp
    tetmesh_topology_initialization.h
    tetmesh_topology_initialization.cpp
    getRSS.cpp
    getRSS.h
    Rational.hpp
    Rational.cpp
    mesh_utils.hpp
    mesh_utils.cpp

    filter_pointers_to_derived.hpp

    orient.hpp
    orient.cpp

    triangle_areas.hpp
    triangle_areas.cpp

    primitive_range.hpp
    primitive_range.cpp

    random_seed.hpp

    VolumeRemesherTriangleInsertion.hpp
    VolumeRemesherTriangleInsertion.cpp

    EigenMatrixWriter.hpp
    EigenMatrixWriter.cpp

    edge_insertion.hpp
    edge_insertion.cpp
    Stopwatch.hpp
    Stopwatch.cpp

    DynamicArray.hpp
    DynamicArray.hxx

    bbox_from_mesh.hpp
    bbox_from_mesh.cpp
    bvh_from_mesh.hpp
    bvh_from_mesh.cpp

    tuple_map_attribute_io.hpp
    tuple_map_attribute_io.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
