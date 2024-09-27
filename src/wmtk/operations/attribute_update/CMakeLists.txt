set(SRC_FILES
    AttributeTransferStrategy.hpp
    AttributeTransferStrategy.cpp

    AttributeTransferStrategyBase.hpp
    AttributeTransferStrategyBase.cpp

    TopologicalTransferStrategy.hpp
    TopologicalTransferStrategy.cpp

    CastAttributeTransferStrategy.hpp

    make_cast_attribute_transfer_strategy.cpp
    make_cast_attribute_transfer_strategy.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
