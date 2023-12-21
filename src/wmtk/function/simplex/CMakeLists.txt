
set(SRC_FILES
    TriangleAMIPS.cpp
    TriangleAMIPS.hpp

    TetrahedronAMIPS.cpp
    TetrahedronAMIPS.hpp

    EdgeValenceEnergy.hpp
    EdgeValenceEnergy.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
