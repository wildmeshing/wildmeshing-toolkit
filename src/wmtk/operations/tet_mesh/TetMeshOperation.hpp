#pragma once
#include <wmtk/operations/Operation.hpp>

namespace wmtk {
class TetMesh;

namespace operations::tet_mesh {

class TetMeshOperation : public virtual Operation
{
public:
    TetMeshOperation(TetMesh& m);
    // internally will try dynamic casting to check for mistakes
    TetMeshOperation(Mesh& m);

protected:
    TetMesh& mesh() const;
    Mesh& base_mesh() const override;
    Accessor<long>& hash_accessor() override;

private:
    TetMesh& m_mesh;
    Accessor<long> m_hash_accessor;
};


} // namespace operations::tet_mesh
} // namespace wmtk
