#pragma once
#include <wmtk/operations/Operation.hpp>

namespace wmtk {
class TriMesh;

namespace operations::tri_mesh {

/// @brief utility class to store the cast from mesh to tri mesh
class TriMeshOperation : public Operation
{
public:
    TriMeshOperation(Mesh& m);

protected:
    TriMesh& tri_mesh() const;

private:
    TriMesh& m_mesh;
};


} // namespace operations::tri_mesh
} // namespace wmtk
