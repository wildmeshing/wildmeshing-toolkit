
set(SRC_FILES
    PerSimplexFunction.hpp
    PerSimplexFunction.cpp
    PerSimplexDifferentiableFunction.hpp
    PerSimplexDifferentiableFunction.cpp

    TriangleAMIPS.cpp
    TriangleAMIPS.hpp


    TetrahedronAMIPS.cpp
    TetrahedronAMIPS.hpp

    EdgeValenceEnergy.hpp
    EdgeValenceEnergy.cpp

    TriangleAutodiffFunction.hpp
    TriangleAutodiffFunction.cpp

    PerSimplexDifferentiableAutodiffFunction.hpp
    PerSimplexDifferentiableAutodiffFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
