set(SRC_FILES
    IntegralBase.hpp
    BarycentricTriangle.hpp

    ThreeChannelPositionMapEvaluator.hpp
    ThreeChannelPositionMapEvaluator.cpp

    PositionMapEvaluator.hpp
    PositionMapEvaluator.cpp

    AnalyticalFunctionTriangleQuadrature.hpp
    AnalyticalFunctionTriangleQuadrature.cpp

    TextureIntegral.hpp
    TextureIntegral.cpp

    UVMapEvaluator.hpp
    UVMapEvaluator.cpp
)
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
