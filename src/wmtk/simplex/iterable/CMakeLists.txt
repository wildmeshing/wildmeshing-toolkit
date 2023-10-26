set(SRC_FILES
    ClosedStarIterable.hpp
    ClosedStarIterable.cpp
    LinkIterable.hpp
    LinkIterable.cpp
    OpenStarIterable.hpp
    OpenStarIterable.cpp
    TopLevelCofacesIterable.hpp
    TopLevelCofacesIterable.cpp
    FacesIterable.hpp
    FacesIterable.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
