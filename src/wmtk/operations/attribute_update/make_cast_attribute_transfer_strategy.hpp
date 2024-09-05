#pragma once

#include "AttributeTransferStrategyBase.hpp"

namespace wmtk::operations::attribute_update {


    std::shared_ptr<AttributeTransferStrategyBase> make_cast_attribute_transfer_strategy( const wmtk::attribute::MeshAttributeHandle& source, const wmtk::attribute::MeshAttributeHandle& target);

}
