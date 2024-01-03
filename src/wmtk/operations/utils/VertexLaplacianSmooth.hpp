#pragma once

#include <wmtk/attribute/MeshAttributeHandle.hpp>


namespace wmtk::operations {

class VertexLaplacianSmooth
{
public:
    VertexLaplacianSmooth(const MeshAttributeHandle<double>& handle);
    virtual ~VertexLaplacianSmooth() = default;

    virtual bool operator()(Mesh& m, const simplex::Simplex& s);

protected:
    MeshAttributeHandle<double> m_attibute_handle;
};

} // namespace wmtk::operations
