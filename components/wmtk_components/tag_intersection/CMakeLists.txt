
set(SRC_FILES
    internal/TagIntersection.hpp
    internal/TagIntersection.cpp
    internal/TagIntersectionOptions.hpp
	tag_intersection.hpp
    tag_intersection.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
