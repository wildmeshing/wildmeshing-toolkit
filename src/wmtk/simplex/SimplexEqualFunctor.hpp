#pragma once

#include "../Mesh.hpp"
#include "Simplex.hpp"

namespace wmtk::simplex {
struct SimplexEqualFunctor
{
    const Mesh& m;

    SimplexEqualFunctor(const Mesh& mm)
        : m{mm}
    {}

    bool operator()(const Simplex& s0, const Simplex& s1) const
    {
        return m.simplex_is_equal(s0, s1);
    }
};
}