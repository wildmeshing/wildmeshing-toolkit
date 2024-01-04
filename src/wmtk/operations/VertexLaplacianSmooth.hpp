#pragma once

#include "AttributesUpdateBase.hpp"

namespace wmtk::operations {

class VertexLaplacianSmooth : public AttributesUpdateBase
{
public:
    VertexLaplacianSmooth(Mesh& m, const attribute::TypedAttributeHandle<double>& handle);
    VertexLaplacianSmooth(attribute::MeshAttributeHandle& handle);

protected:
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

protected:
    attribute::TypedAttributeHandle<double> m_attibute_handle;
};

} // namespace wmtk::operations
