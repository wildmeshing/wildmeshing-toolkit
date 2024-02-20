set(SRC_FILES
    AttributeTransferStrategy.hpp
    AttributeTransferStrategy.cpp
    AttributeTransferStrategyPosition.hpp

    AttributeTransferStrategyBase.hpp
    AttributeTransferStrategyBase.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
