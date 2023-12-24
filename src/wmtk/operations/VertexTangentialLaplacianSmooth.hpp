#pragma once

#include "VertexLaplacianSmooth.hpp"

namespace wmtk::operations {

class VertexTangentialLaplacianSmooth : public VertexLaplacianSmooth
{
public:
    VertexTangentialLaplacianSmooth(
        Mesh& m,
        const MeshAttributeHandle<double>& handle,
        const double damping_factor = 1.0);

protected:
    std::vector<Simplex> execute(const Simplex& simplex) override;

    double m_damping_factor;
};

} // namespace wmtk::operations
