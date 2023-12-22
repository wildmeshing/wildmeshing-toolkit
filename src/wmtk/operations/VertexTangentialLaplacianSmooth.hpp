#pragma once

#include "VertexLaplacianSmooth.hpp"

namespace wmtk::operations {

class VertexTangentialLaplacianSmooth : public VertexLaplacianSmooth
{
public:
    VertexTangentialLaplacianSmooth(Mesh& m, const MeshAttributeHandle<double>& handle);

protected:
    std::vector<Simplex> execute(const Simplex& simplex) override;
};

} // namespace wmtk::operations
