
set(SRC_FILES
    Cache.cpp
    Cache.hpp

    HDF5Reader.hpp
    HDF5Reader.cpp
    HDF5Writer.hpp
    HDF5Writer.cpp
    MeshWriter.hpp
    MeshWriter.cpp
    MeshReader.hpp
    MeshReader.cpp
    MshReader.hpp
    MshReader.cpp
    ParaviewWriter.hpp
    ParaviewWriter.cpp

    CachedMultiMesh.hpp
    CachedMultiMesh.cpp

    read_mesh.hpp
    read_mesh.cpp
    internal/get_attribute_handles.hpp
    internal/get_attribute_handles.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
