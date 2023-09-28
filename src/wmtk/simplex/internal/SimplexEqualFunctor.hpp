#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::simplex::internal {
struct SimplexEqualFunctor
{
    const Mesh& m;

    SimplexEqualFunctor(const Mesh& mm)
        : m{mm}
    {}

    bool operator()(const Simplex& s0, const Simplex& s1) const
    {
        return m.simplices_are_equal(s0, s1);
    }
};
} // namespace wmtk::simplex::internal