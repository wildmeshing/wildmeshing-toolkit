
set(SRC_FILES
    Function.cpp
    Function.hpp
    DifferentiableFunction.cpp
    DifferentiableFunction.hpp

    LocalFunction.hpp
    LocalFunction.cpp
    LocalDifferentiableFunction.hpp
    LocalDifferentiableFunction.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(utils)
add_subdirectory(simplex)
