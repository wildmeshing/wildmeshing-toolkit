
set(SRC_FILES
    Function.hpp    
    Function.cpp
    DifferentiableFunction.cpp
    DifferentiableFunction.hpp
    AMIPS.hpp
    AMIPS.cpp
    TriMeshValenceFunction.hpp
    TriMeshValenceFunction.cpp
    AreaAccuracy.hpp
    AreaAccuracy.cpp
    AutodiffFunction.hpp
    AutodiffFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)