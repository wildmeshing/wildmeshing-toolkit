set(SRC_FILES
    PerTriangleTextureIntegralAccuracyFunction.hpp
    PerTriangleTextureIntegralAccuracyFunction.cpp
    PerTriangleAnalyticalIntegral.cpp
    PerTriangleAnalyticalIntegral.hpp

    PositionMapAMIPS.cpp
    PositionMapAMIPS.hpp

    SumEnergy.hpp
    SumEnergy.cpp
)
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
