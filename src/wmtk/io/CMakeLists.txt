
set(SRC_FILES
    HDF5Writer.hpp
    HDF5Writer.cpp
    MeshWriter.hpp
    MeshReader.hpp
    MeshReader.cpp
    ParaviewWriter.hpp
    ParaviewWriter.cpp
    EdgeMeshReader.hpp
    EdgeMeshReader.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
