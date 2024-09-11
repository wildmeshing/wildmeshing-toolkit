#pragma once
#include <memory>
#include <vector>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::operations {
class AttributeTransferEdge : public std::enable_shared_from_this<AttributeTransferEdge>
{
public:
    virtual ~AttributeTransferEdge() = 0;
    virtual std::vector<wmtk::attribute::MeshAttributeHandle> sources() const = 0;
    virtual std::vector<wmtk::attribute::MeshAttributeHandle> targets() const = 0;
};
} // namespace wmtk::operations
