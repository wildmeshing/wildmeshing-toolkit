#pragma once

#include <wmtk/attribute/MeshAttributeHandle.hpp>


namespace wmtk::operations {

class VertexLaplacianSmooth
{
public:
    VertexLaplacianSmooth(const attribute::MeshAttributeHandle& handle);
    virtual ~VertexLaplacianSmooth() = default;

    virtual bool operator()(Mesh& m, const simplex::Simplex& s);

protected:
    attribute::MeshAttributeHandle m_attibute_handle;
};

} // namespace wmtk::operations
