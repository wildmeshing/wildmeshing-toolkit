#pragma once
#include "MeshInvariant.hpp"


namespace wmtk {
class TetMesh;
// convenience for extracting a TetMesh even though the Invariant class only stores a Mesh
// internally
class TetMeshInvariant : public MeshInvariant
{
public:
    TetMeshInvariant(const TetMesh& m);
    const TetMesh& mesh() const;
};
} // namespace wmtk
