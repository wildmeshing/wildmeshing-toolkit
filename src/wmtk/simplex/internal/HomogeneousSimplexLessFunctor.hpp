#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>

namespace wmtk::simplex::internal {
struct HomogeneousSimplexLessFunctor
{
    const Mesh& m;
    const PrimitiveType primitive_type;

    HomogeneousSimplexLessFunctor(const Mesh& mm, PrimitiveType pt)
        : m{mm}
        , primitive_type(pt)
    {}

    bool operator()(const Tuple& t0, const Tuple& t1) const
    {
        return utils::SimplexComparisons::less(m, primitive_type, t0, t1);
    }
};
} // namespace wmtk::simplex::internal
