#pragma once
#include <wmtk/operations/Operation.hpp>

namespace wmtk {
class TriMesh;

namespace operations::tri_mesh {

class TriMeshOperation : public virtual Operation
{
public:
    TriMeshOperation(TriMesh& m);
    // internally will try dynamic casting to check for mistakes
    TriMeshOperation(Mesh& m);

protected:
    TriMesh& mesh() const;
    Mesh& base_mesh() const override;
    Accessor<long>& hash_accessor() override;

private:
    TriMesh& m_mesh;
    Accessor<long> m_hash_accessor;
};


} // namespace operations::tri_mesh
} // namespace wmtk
