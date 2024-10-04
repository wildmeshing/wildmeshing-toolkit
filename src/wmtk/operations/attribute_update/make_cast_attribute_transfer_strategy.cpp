#include "CastAttributeTransferStrategy.hpp"
#include "wmtk/operations/attribute_update/AttributeTransferStrategy.hpp"

namespace wmtk::operations::attribute_update {


std::shared_ptr<AttributeTransferStrategyBase> make_cast_attribute_transfer_strategy(
    const wmtk::attribute::MeshAttributeHandle& source,
    const wmtk::attribute::MeshAttributeHandle& target)
{
    return std::visit(
        [&](const auto& s,
            const auto& t) noexcept -> std::shared_ptr<AttributeTransferStrategyBase> {
            using SType = typename std::decay_t<decltype(s)>::Type;
            using TType = typename std::decay_t<decltype(t)>::Type;

            return std::make_shared<CastAttributeTransferStrategy<TType, SType>>(target, source);
        },
        source.handle(),
        target.handle());
}

} // namespace wmtk::operations::attribute_update
