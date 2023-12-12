#pragma once
#include "PerSimplexFunction.hpp"

namespace wmtk::function {

class EdgeValenceEnergy : public PerSimplexFunction
{
public:
    EdgeValenceEnergy(const TriMesh& mesh);
    double get_value(const Simplex& simplex) const override;
    using PerSimplexFunction::get_value;

protected:
    const TriMesh& tri_mesh() const;
};
} // namespace wmtk::function
