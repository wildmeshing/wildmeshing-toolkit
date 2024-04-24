#pragma once

#include <wmtk/operations/EdgeCollapse.hpp>

namespace wmtk::operations::composite {

class TetTwoWayEdgeCollapse : public Operation
{
public:
    TetTwoWayEdgeCollapse(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline EdgeCollapse& collapse() { return m_collapse; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    EdgeCollapse m_collapse;
};
} // namespace wmtk::operations::composite