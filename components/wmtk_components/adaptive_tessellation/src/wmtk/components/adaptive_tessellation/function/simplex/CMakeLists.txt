set(SRC_FILES
    PerTriangleTextureIntegralAccuracyFunction.hpp
    PerTriangleTextureIntegralAccuracyFunction.cpp
    PerTriangleAnalyticalIntegral.cpp
    PerTriangleAnalyticalIntegral.hpp
)
target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})