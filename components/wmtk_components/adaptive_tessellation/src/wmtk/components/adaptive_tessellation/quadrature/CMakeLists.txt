set(SRC_FILES
    ClippedQuadrature.cpp
    ClippedQuadrature.hpp
    Quadrature.hpp
    TriangleQuadrature.cpp
    TriangleQuadrature.hpp
    PolygonClipping.cpp
    PolygonClipping.hpp
)


target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})