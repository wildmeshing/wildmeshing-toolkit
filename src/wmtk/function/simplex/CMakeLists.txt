
set(SRC_FILES
    TriangleAMIPS.cpp
    TriangleAMIPS.hpp

    AMIPS.cpp
    AMIPS.hpp

    EdgeValenceEnergy.hpp
    EdgeValenceEnergy.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
