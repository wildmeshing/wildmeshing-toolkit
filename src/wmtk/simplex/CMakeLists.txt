set(SRC_FILES
    Simplex.hpp
    SimplexCollection.hpp
    SimplexCollection.cpp
    coface_cells.hpp
    coface_cells.cpp
    open_star.hpp
    open_star.cpp
    simplex_boundary.hpp
    simplex_boundary.cpp
    internal/SimplexEqualFunctor.hpp
    internal/SimplexLessFunctor.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
