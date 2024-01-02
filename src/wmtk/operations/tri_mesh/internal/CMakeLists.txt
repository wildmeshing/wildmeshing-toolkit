
set(SRC_FILES
    VertexSmoothUsingDifferentiableEnergyFactory.hpp
    VertexSmoothUsingDifferentiableEnergyFactory.cpp
    VertexSmoothGradientDescent.hpp
    VertexSmoothGradientDescent.cpp
    VertexSmoothNewtonMethod.hpp
    VertexSmoothNewtonMethod.cpp
    VertexSmoothNewtonMethodWithLineSearch.hpp
    VertexSmoothNewtonMethodWithLineSearch.cpp
    SeamlessGradientDescent.hpp
    SeamlessGradientDescent.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
