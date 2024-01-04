#pragma once

#include "VertexLaplacianSmooth.hpp"

namespace wmtk::operations {

class VertexTangentialLaplacianSmooth : public VertexLaplacianSmooth
{
public:
    constexpr static double DEFAULT_DAMPING_FACTOR = 1.0;

    VertexTangentialLaplacianSmooth(
        Mesh& m,
        const attribute::TypedAttributeHandle<double>& handle,
        double damping_factor = DEFAULT_DAMPING_FACTOR);
    VertexTangentialLaplacianSmooth(
        attribute::MeshAttributeHandle& handle,
        double damping_factor = DEFAULT_DAMPING_FACTOR);

protected:
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

    double m_damping_factor;
};

} // namespace wmtk::operations
