#pragma once

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::operations::composite {

/**
 * The return tuple is the new vertex, pointing to the original vertex.
 * This operation does not set vertex positions.
 *     / | \
 *    /  |  \
 *   /  _*_  \
 *  / _< f \_ \
 *  |/_______\|
 *   \       /
 *    \     /
 *     \   /
 **/
class TriFaceSplit : public EdgeSplit, public EdgeCollapse
{
public:
    TriFaceSplit(Mesh& m);

    bool operator()(const Simplex& simplex) { return EdgeSplit::operator()(simplex); }


    PrimitiveType primitive_type() const override { return PrimitiveType::Face; }

protected:
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;
    std::vector<Simplex> execute(const Simplex& simplex) override;
};

} // namespace wmtk::operations::composite
