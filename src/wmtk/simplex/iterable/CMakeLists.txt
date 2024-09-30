set(SRC_FILES
    ClosedStarIterable.hpp
    ClosedStarIterable.cpp
    LinkIterable.hpp
    LinkIterable.cpp
    OpenStarIterable.hpp
    OpenStarIterable.cpp
    TopDimensionCofacesIterable.hpp
    TopDimensionCofacesIterable.cpp
    CofacesSingleDimensionIterable.hpp
    CofacesSingleDimensionIterable.cpp
    FacesIterable.hpp
    FacesIterable.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
