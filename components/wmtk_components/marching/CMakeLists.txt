
set(SRC_FILES
    internal/MarchingOptions.hpp
    internal/MarchingOptions.cpp
    internal/Marching.hpp
    internal/Marching.cpp
	marching.hpp
    marching.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
