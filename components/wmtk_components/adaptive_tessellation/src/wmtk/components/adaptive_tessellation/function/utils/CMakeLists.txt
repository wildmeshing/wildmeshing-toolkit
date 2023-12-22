set(SRC_FILES
    IntegralBase.hpp
    BarycentricTriangle.hpp

    ThreeChannelPositionMapEvaluator.hpp
    ThreeChannelPositionMapEvaluator.cpp

    TextureIntegral.hpp
    TextureIntegral.cpp

    UVMapEvaluator.hpp
    UVMapEvaluator.cpp
)
target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})