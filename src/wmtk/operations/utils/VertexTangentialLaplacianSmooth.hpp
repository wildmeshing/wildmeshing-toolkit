#pragma once

#include "VertexLaplacianSmooth.hpp"

namespace wmtk::operations {

class VertexTangentialLaplacianSmooth : public VertexLaplacianSmooth
{
public:
    constexpr static double DEFAULT_DAMPING_FACTOR = 1.0;

    VertexTangentialLaplacianSmooth(
        attribute::MeshAttributeHandle& handle,
        double damping_factor = DEFAULT_DAMPING_FACTOR);

    bool operator()(Mesh& m, const simplex::Simplex& s) override;

private:
    double m_damping_factor;
};

} // namespace wmtk::operations
