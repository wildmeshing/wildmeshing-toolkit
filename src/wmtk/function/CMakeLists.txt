
set(SRC_FILES
    Function.cpp
    Function.hpp

    LocalNeighborsSumFunction.hpp
    LocalNeighborsSumFunction.cpp

    PerSimplexFunction.hpp
    PerSimplexFunction.cpp

    PerSimplexAutodiffFunction.hpp
    PerSimplexAutodiffFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
add_subdirectory(simplex)
