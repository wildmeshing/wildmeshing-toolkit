#pragma once

#include "Invariant.hpp"

namespace wmtk {
class InteriorEdgeInvariant : public Invariant
{
    bool before(const Tuple& t) override;
};
} // namespace wmtk
