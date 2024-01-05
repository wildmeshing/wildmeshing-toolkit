
set(SRC_FILES
    internal/RegularSpaceOptions.cpp
    internal/RegularSpaceOptions.hpp
    internal/RegularSpace.hpp
    internal/RegularSpace.cpp
	regular_space.hpp
    regular_space.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
