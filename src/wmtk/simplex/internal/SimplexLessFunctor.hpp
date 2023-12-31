#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>

namespace wmtk::simplex::internal {
struct SimplexLessFunctor
{
    const Mesh& m;

    SimplexLessFunctor(const Mesh& mm)
        : m{mm}
    {}

    bool operator()(const Simplex& s0, const Simplex& s1) const
    {
        return utils::SimplexComparisons::less(m, s0, s1);
    }
};

} // namespace wmtk::simplex::internal
