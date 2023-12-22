set(SRC_FILES
    EdgeOperationData.hpp
    EdgeOperationData.cpp

    CollapseNewAttributeStrategy.hpp
    CollapseNewAttributeStrategy.cpp

    SplitNewAttributeStrategy.hpp
    SplitNewAttributeStrategy.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})