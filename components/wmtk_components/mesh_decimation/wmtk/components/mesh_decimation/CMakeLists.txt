
set(SRC_FILES
    internal/MeshDecimationOptions.hpp
    internal/MeshDecimationOptions.cpp
    internal/MeshDecimation.hpp
    internal/MeshDecimation.cpp
	mesh_decimation.hpp
    mesh_decimation.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
