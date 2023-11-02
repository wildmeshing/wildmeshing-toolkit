
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
    AMIPS2D.hpp
    AMIPS2D.cpp
    AMIPS3D.hpp
    AMIPS3D.cpp
    # PositionMapAMIPS2D.hpp
    # PositionMapAMIPS2D.cpp
    ValenceEnergyPerEdge.hpp
    ValenceEnergyPerEdge.cpp

    AutodiffFunction.hpp
    AutodiffFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
