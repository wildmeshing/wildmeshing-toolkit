
set(SRC_FILES
    internal/MarchingOptions.hpp
    internal/MarchingOptions.cpp
    internal/Marching.hpp
    internal/Marching.cpp
	marching.hpp
    marching.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
