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

/*
    Notice!!!
    This face split has two different split operations to assign different attribute transfers
*/
class TriFaceSplit_alt : public Operation
{
public:
    TriFaceSplit_alt(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Triangle; }

    inline EdgeSplit& split_1() { return m_split_1; }
    inline EdgeSplit& split_2() { return m_split_2; }
    inline EdgeCollapse& collapse() { return m_collapse; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    EdgeSplit m_split_1;
    EdgeSplit m_split_2;
    EdgeCollapse m_collapse;
};

} // namespace wmtk::operations::composite
