#pragma once
#include "MeshInvariant.hpp"


namespace wmtk {
// convenience for extracting a TriMesh even though the Invariant class only stores a Mesh
// internally
class TriMesh;
class TriMeshInvariant : public MeshInvariant
{
public:
    TriMeshInvariant(const TriMesh& m);
    const TriMesh& mesh() const;
};
} // namespace wmtk
