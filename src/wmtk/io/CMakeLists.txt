
set(SRC_FILES
    HDF5Writer.hpp
    HDF5Writer.cpp
    MeshWriter.hpp
    MeshReader.hpp
    MeshReader.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
