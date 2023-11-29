#pragma once
#include <wmtk/operations/Operation.hpp>

namespace wmtk {
class EdgeMesh;

namespace operations::edge_mesh {

class EdgeMeshOperation : public virtual Operation
{
public:
    EdgeMeshOperation(EdgeMesh& m);
    // internally will try dynamic casting to check for mistakes
    EdgeMeshOperation(Mesh& m);

protected:
    EdgeMesh& mesh() const;
    Mesh& base_mesh() const override;
    Accessor<long>& hash_accessor() override;

private:
    EdgeMesh& m_mesh;
    Accessor<long> m_hash_accessor;
};


} // namespace operations::edge_mesh
} // namespace wmtk
