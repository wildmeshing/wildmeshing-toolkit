#pragma once
#include "Invariant.hpp"


namespace wmtk {
class Mesh;

class MeshInvariant : public Invariant
{
public:
    MeshInvariant() = default; // TODO HACK remove this!!!
    MeshInvariant(const Mesh& m);

    const Mesh& mesh() const;

    inline static const Mesh* m_mesh; // TODO: HACK remove static and make it private
private:
};
} // namespace wmtk
