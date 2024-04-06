set(SRC_FILES
    BaseCollapseNewAttributeStrategy.hpp
    BaseSplitNewAttributeStrategy.hpp

    CollapseNewAttributeStrategy.hpp
    CollapseNewAttributeStrategy.cpp

    SplitNewAttributeStrategy.hpp
    SplitNewAttributeStrategy.cpp

    NewAttributeStrategy.hpp
    NewAttributeStrategy.cpp

    CollapseNewAttributeTopoInfo.hpp
    SplitNewAttributeTopoInfo.hpp

    HybridRationalCollapseNewAttributeStrategy.hpp
    HybridRationalCollapseNewAttributeStrategy.cpp

    HybridRationalSplitNewAttributeStrategy.hpp
    HybridRationalSplitNewAttributeStrategy.cpp
)

target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
