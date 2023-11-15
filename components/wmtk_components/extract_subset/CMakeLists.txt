set(SRC_FILES
    internal/extract_subset_2d.hpp
    internal/extract_subset_2d.cpp
    extract_subset.hpp
    extract_subset.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})