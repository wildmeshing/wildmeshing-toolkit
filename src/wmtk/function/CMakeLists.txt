
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

    TriAMIPS.cpp
    TriAMIPS.hpp
    ValenceEnergyPerEdge.hpp
    ValenceEnergyPerEdge.cpp

    TriangleAutodiffFunction.hpp
    TriangleAutodiffFunction.cpp

    AutodiffFunction.hpp
    AutodiffFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
