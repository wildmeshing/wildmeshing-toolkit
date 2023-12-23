#include "AttributeInitializationHandle.hpp"

namespace wmtk::attribute {
    AttributeInitializationHandleBase(
        std::shared_ptr<operations::SplitNewAttributeStrategy> split_strategy,
        std::shared_ptr<operations::CollapseNewAttributeStrategy> collapse_strategy): m_split_strategy(std::move(split_strategy), m_collapse_strategy(std::move(collapse_strategy)) {}
            AttributeInitialiationHandleBase::~AttributeInitializationHandleBase() = default;
    }
