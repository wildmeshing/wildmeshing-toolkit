set(SRC_FILES
    internal/extract_subset_2d.hpp
    internal/extract_subset_2d.cpp
    internal/topology_separate_2d.cpp
    internal/topology_separate_2d.hpp
    internal/extract_subset_3d.hpp
    internal/extract_subset_3d.cpp
    internal/topology_separate_3d.cpp
    internal/topology_separate_3d.hpp
    internal/utils.hpp
    internal/utils.cpp
    extract_subset.hpp
    extract_subset.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})