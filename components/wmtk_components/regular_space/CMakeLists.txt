
set(SRC_FILES
    internal/RegularSpaceOptions.cpp
    internal/RegularSpaceOptions.hpp
    internal/RegularSpace.hpp
    internal/RegularSpace.cpp
	regular_space.hpp
    regular_space.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
