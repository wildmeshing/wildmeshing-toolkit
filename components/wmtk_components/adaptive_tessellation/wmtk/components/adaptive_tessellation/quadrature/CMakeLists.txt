set(SRC_FILES
    ClippedQuadrature.cpp
    ClippedQuadrature.hpp
    Quadrature.hpp
    TriangleQuadrature.cpp
    TriangleQuadrature.hpp
    PolygonClipping.cpp
    PolygonClipping.hpp
    LineQuadrature.hpp
    LineQuadrature.cpp
)

target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
