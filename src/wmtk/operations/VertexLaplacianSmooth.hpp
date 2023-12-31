#pragma once

#include "AttributesUpdateBase.hpp"

namespace wmtk::operations {

class VertexLaplacianSmooth : public AttributesUpdateBase
{
public:
    VertexLaplacianSmooth(Mesh& m, const MeshAttributeHandle<double>& handle);

protected:
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

protected:
    MeshAttributeHandle<double> m_attibute_handle;
};

} // namespace wmtk::operations
