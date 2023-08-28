
set(SRC_FILES
    Energy.hpp    
    Energy.cpp
    DifferentiableEnergy.hpp
    AMIPS.hpp
    AMIPS.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
