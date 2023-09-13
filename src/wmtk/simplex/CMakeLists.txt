set(SRC_FILES
    OpenStar.hpp
    OpenStar.cpp
    Simplex.hpp
    SimplexCollection.hpp
    SimplexCollection.cpp
    SimplexEqualFunctor.hpp
    SimplexLessFunctor.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
