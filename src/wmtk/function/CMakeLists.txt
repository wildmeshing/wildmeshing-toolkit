
set(SRC_FILES
    Function.hpp    
    Function.cpp
    DifferentiableFunction.cpp
    DifferentiableFunction.hpp

    LocallyDifferentiableFunction.hpp
    LocallyDifferentiableFunction.cpp
    PerSimplexFunction.hpp
    PerSimplexFunction.cpp
    DifferentiablePerSimplexFunction.hpp
    DifferentiablePerSimplexFunction.cpp

    AMIPS.hpp
    AMIPS.cpp
    AMIPS2D.hpp
    AMIPS2D.cpp
    AMIPS3D.hpp
    AMIPS3D.cpp
    PositionMapAMIPS2D.hpp
    PositionMapAMIPS2D.cpp

    TriMeshValenceFunction.hpp
    TriMeshValenceFunction.cpp
    AutodiffFunction.hpp
    AutodiffFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
