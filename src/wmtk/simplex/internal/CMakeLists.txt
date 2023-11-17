set(SRC_FILES
    SimplexLessFunctor.hpp
    SimplexEqualFunctor.hpp
    boundary_with_preserved_coface.hpp
    boundary_with_preserved_coface.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
