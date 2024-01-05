
set(SRC_FILES
    internal/MeshInfoOptions.hpp
    mesh_info.hpp
    mesh_info.cpp)


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})