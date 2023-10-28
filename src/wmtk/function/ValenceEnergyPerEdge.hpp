#pragma once
#include "PerSimplexFunction.hpp"

namespace wmtk::function {

class ValenceEnergyPerEdge : public PerSimplexFunction
{
public:
    ValenceEnergyPerEdge(const TriMesh& mesh);
    double get_value(const Tuple& simplex) const override;
    using PerSimplexFunction::get_value;

protected:
    const TriMesh& tri_mesh() const;
};
} // namespace wmtk
