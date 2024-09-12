
set(SRC_FILES
GetAllMeshesOptions.hpp
    get_all_meshes.hpp
    get_all_meshes.cpp)


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})