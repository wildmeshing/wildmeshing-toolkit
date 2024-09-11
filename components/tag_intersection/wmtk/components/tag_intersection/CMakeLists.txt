
set(SRC_FILES
    internal/TagIntersection.hpp
    internal/TagIntersection.cpp
    internal/TagIntersectionOptions.hpp
    internal/TagIntersectionOptions.cpp
	tag_intersection.hpp
    tag_intersection.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})