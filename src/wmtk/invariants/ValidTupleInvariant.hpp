#pragma once

#include "MeshInvariant.hpp"

namespace wmtk {
class ValidTupleInvariant : public MeshInvariant
{
    using MeshInvariant::MeshInvariant;
    bool before(const Tuple& t) const override;
};
} // namespace wmtk
