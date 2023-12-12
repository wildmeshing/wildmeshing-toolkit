
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

    TriangleAMIPS.cpp
    TriangleAMIPS.hpp
    EdgeValenceEnergy.hpp
    EdgeValenceEnergy.cpp

    TriangleAutodiffFunction.hpp
    TriangleAutodiffFunction.cpp

    PerSimplexDifferentiableAutodiffFunction.hpp
    PerSimplexDifferentiableAutodiffFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
