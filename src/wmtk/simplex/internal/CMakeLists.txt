set(SRC_FILES
    SimplexLessFunctor.hpp
    SimplexEqualFunctor.hpp
    boundary_with_preserved_face.hpp
    boundary_with_preserved_face.cpp

    VisitedArray.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
