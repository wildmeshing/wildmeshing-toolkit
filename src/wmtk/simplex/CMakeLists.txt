set(SRC_FILES
    CofaceCells.hpp
    CofaceCells.cpp
    OpenStar.hpp
    OpenStar.cpp
    Simplex.hpp
    SimplexBoundary.hpp
    SimplexBoundary.cpp
    SimplexCollection.hpp
    SimplexCollection.cpp
    SimplexEqualFunctor.hpp
    SimplexLessFunctor.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
