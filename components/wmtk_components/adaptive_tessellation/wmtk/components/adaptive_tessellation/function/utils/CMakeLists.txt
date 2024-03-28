set(SRC_FILES
    IntegralBasedAvgDistance.hpp
    Triangle2DTo3DMapping.hpp
    BarycentricTriangle.hpp

    ThreeChannelPositionMapEvaluator.hpp
    ThreeChannelPositionMapEvaluator.cpp

    PositionMapEvaluator.hpp
    PositionMapEvaluator.cpp

    AnalyticalFunctionAvgDistanceToLimit.hpp
    AnalyticalFunctionAvgDistanceToLimit.cpp

    TextureMapAvgDistanceToLimit.hpp
    TextureMapAvgDistanceToLimit.cpp

    MaxDistanceToLimit.hpp
    MaxDistanceToLimit.cpp

    UVMapEvaluator.hpp
    UVMapEvaluator.cpp

    area_barrier.hpp
)
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
