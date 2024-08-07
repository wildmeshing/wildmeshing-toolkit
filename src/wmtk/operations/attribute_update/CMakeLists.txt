set(SRC_FILES
    AttributeTransferStrategy.hpp
    AttributeTransferStrategy.cpp

    AttributeTransferStrategyBase.hpp
    AttributeTransferStrategyBase.cpp

    CastAttributeTransferStrategy.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
