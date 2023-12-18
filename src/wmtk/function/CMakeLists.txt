
set(SRC_FILES
    Function.cpp
    Function.hpp
    DifferentiableFunction.cpp
    DifferentiableFunction.hpp

    LocalFunction.hpp
    LocalFunction.cpp
    LocalDifferentiableFunction.hpp
    LocalDifferentiableFunction.cpp
    PerSimplexFunction.hpp
    PerSimplexFunction.cpp
    PerSimplexDifferentiableFunction.hpp
    PerSimplexDifferentiableFunction.cpp

    AMIPS.hpp
    AMIPS.cpp
    ValenceEnergyPerEdge.hpp
    ValenceEnergyPerEdge.cpp

    TriangleAutodiffFunction.hpp
    TriangleAutodiffFunction.cpp

    AutodiffFunction.hpp
    AutodiffFunction.cpp

    PerTriangleTextureIntegralAccuracyFunction.hpp
    PerTriangleTextureIntegralAccuracyFunction.cpp
    PositionMapAMIPS2D.hpp
    PositionMapAMIPS2D.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
