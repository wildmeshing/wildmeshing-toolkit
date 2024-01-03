#pragma once

#include "VertexLaplacianSmooth.hpp"

namespace wmtk::operations {

class VertexTangentialLaplacianSmooth : public VertexLaplacianSmooth
{
public:
    VertexTangentialLaplacianSmooth(
        const MeshAttributeHandle<double>& handle,
        const double damping_factor = 1.0);

    bool operator()(Mesh& m, const simplex::Simplex& s) override;

private:
    double m_damping_factor;
};

} // namespace wmtk::operations
