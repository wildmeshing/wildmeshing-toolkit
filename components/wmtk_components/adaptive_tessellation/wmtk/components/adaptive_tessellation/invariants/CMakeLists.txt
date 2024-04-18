set(SRC_FILES
    RGBSplitInvariant.hpp
    RGBSplitInvariant.cpp
    RGBSwapInvariant.hpp
    RGBSwapInvariant.cpp
    MaxTriangleNumberInvariant.hpp
    MaxTriangleNumberInvariant.cpp
)


target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})