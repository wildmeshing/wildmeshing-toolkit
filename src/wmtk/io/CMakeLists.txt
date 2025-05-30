
set(SRC_FILES
    Cache.cpp
    Cache.hpp

    HDF5Reader.hpp
    HDF5Reader.cpp
    HDF5Writer.hpp
    HDF5Writer.cpp
    MeshWriter.hpp
    MeshReader.hpp
    MeshReader.cpp
    MshReader.hpp
    MshReader.cpp
    ParaviewWriter.hpp
    ParaviewWriter.cpp

    read_mesh.hpp
    read_mesh.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
