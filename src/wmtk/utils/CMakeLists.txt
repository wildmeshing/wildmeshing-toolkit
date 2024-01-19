
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
    mesh_utils.hpp
    mesh_utils.cpp
    TupleInspector.hpp

    TupleInspector.cpp
    triangle_areas.hpp
    triangle_areas.cpp

    vector_hash.hpp
    vector_hash.cpp
    #Optimization.hpp
    #Optimization.cpp

    metaprogramming/as_mesh_variant.hpp
    metaprogramming/as_mesh_variant.cpp
    metaprogramming/MeshVariantTraits.cpp
    metaprogramming/DerivedReferenceWrapperVariantTraits.hpp
    metaprogramming/MeshVariantTraits.hpp
    metaprogramming/ReferenceWrappedFunctorReturnCache.hpp
    metaprogramming/ReferenceWrappedFunctorReturnType.hpp
    metaprogramming/ReferenceWrapperVariant.hpp
    metaprogramming/as_variant.hpp
    metaprogramming/unwrap_ref.hpp

    primitive_range.hpp
    primitive_range.cpp

    random_seed.hpp


    MerkleTreeInteriorNode.hpp
    MerkleTreeInteriorNode.cpp
    Hashable.hpp
    Hashable.cpp

    merkle_tree.hpp
    merkle_tree.cpp

    merkle_tree_diff.hpp
    merkle_tree_diff.cpp

    VolumeRemesherTriangleInsertion.hpp
    VolumeRemesherTriangleInsertion.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})


add_subdirectory(metaprogramming)
