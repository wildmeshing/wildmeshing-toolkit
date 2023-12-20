#pragma once
#include "Invariant.hpp"


namespace wmtk {
// convenience for extracting a TriMesh even though the Invariant class only stores a Mesh
// internally
class TriMesh;
class TriMeshInvariant : public Invariant
{
public:
    TriMeshInvariant(const TriMesh& m);
    const TriMesh& mesh() const;
};
} // namespace wmtk
