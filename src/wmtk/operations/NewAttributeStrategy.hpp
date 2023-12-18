#pragma once
#include <wmtk/PrimitiveType.hpp>
namespace wmtk {
class Mesh;
class Tuple;
} // namespace wmtk

namespace wmtk::operations {


class NewAttributeStrategy
{
public:
    // default operation types
    enum class OpType {
        CopyTuple,
        CopyOther, // per-dimension "other" simplex option
        Mean,
        Custom,
        Default
    };
    virtual ~NewAttributeStrategy();

    virtual PrimitiveType primitive_type() const = 0;

    virtual Mesh& mesh() = 0;

    const Mesh& mesh() const;
};
} // namespace wmtk::operations
