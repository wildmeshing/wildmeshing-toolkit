
set(SRC_FILES
    Energy.hpp    
    Energy.cpp
    DifferentiableEnergy.hpp
    AMIPS.hpp
    AMIPS.cpp
    TriMeshValenceEnergy.hpp
    TriMeshValenceEnergy.cpp
    AreaAccuracy.hpp
    AreaAccuracy.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)