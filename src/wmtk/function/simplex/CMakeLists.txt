
set(SRC_FILES
    TriangleAMIPS.cpp
    TriangleAMIPS.hpp

    AMIPS.cpp
    AMIPS.hpp

    SYMDIR.hpp
    SYMDIR.cpp

    EdgeValenceEnergy.hpp
    EdgeValenceEnergy.cpp

    TriangleAreaAD.hpp
    TriangleAreaAD.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
