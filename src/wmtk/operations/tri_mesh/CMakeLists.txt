set(SRC_FILES
    EdgeOperationData.hpp
    EdgeOperationData.cpp

    CollapseNewAttributeStrategy.hpp
    CollapseNewAttributeStrategy.cpp

    SplitNewAttributeStrategy.hpp
    SplitNewAttributeStrategy.cpp

    BasicSplitNewAttributeStrategy.hpp
    BasicSplitNewAttributeStrategy.cpp

    BasicCollapseNewAttributeStrategy.hpp
    BasicCollapseNewAttributeStrategy.cpp

    # TODODiscuss add me back
    # BoundaryAwareCollapseNewAttributeStrategy.hpp
    # BoundaryAwareCollapseNewAttributeStrategy.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
