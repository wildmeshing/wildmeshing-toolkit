set(SRC_FILES
    PerTriangleTextureIntegralAccuracyFunction.hpp
    PerTriangleTextureIntegralAccuracyFunction.cpp
    PerTriangleAnalyticalIntegral.cpp
    PerTriangleAnalyticalIntegral.hpp
)
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
