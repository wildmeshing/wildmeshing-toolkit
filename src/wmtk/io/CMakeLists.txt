
set(SRC_FILES
    Cache.cpp
    Cache.hpp

    CacheStack.hpp
    CacheStack.cpp
    SubCacheHandle.hpp
    SubCacheHandle.cpp

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

    CachedMultiMesh.hpp
    CachedMultiMesh.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
