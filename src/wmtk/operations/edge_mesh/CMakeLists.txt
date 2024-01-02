set(SRC_FILES
    EdgeOperationData.hpp
    EdgeOperationData.cpp

    CollapseNewAttributeTopoInfo.hpp
    CollapseNewAttributeTopoInfo.cpp

    SplitNewAttributeTopoInfo.hpp
    SplitNewAttributeTopoInfo.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})