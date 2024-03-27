set(SRC_FILES
    IntegralBase.hpp
    BarycentricTriangle.hpp

    ThreeChannelPositionMapEvaluator.hpp
    ThreeChannelPositionMapEvaluator.cpp

    PositionMapEvaluator.hpp
    PositionMapEvaluator.cpp

    AnalyticalFunctionNumericalIntegral.hpp
    AnalyticalFunctionNumericalIntegral.cpp

    TextureIntegral.hpp
    TextureIntegral.cpp

    UVMapEvaluator.hpp
    UVMapEvaluator.cpp

    area_barrier.hpp
)
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
