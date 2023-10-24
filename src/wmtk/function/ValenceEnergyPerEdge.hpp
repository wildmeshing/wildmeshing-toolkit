#pragma once
#include "PerSimplexFunction.hpp"

namespace wmtk {
namespace function {

class ValenceEnergyPerEdge : public PerSimplexFunction
{
public:
    ValenceEnergyPerEdge(const TriMesh& mesh);
    double get_value(const Simplex& simplex) const override;

protected:
    const TriMesh& mesh() const;
};
} // namespace function
} // namespace wmtk
