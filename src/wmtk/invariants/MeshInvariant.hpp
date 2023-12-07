#pragma once
#include "Invariant.hpp"


namespace wmtk {
class Mesh;

class MeshInvariant : public Invariant
{
public:
    MeshInvariant(const Mesh& m);
    ~MeshInvariant();

    const Mesh& mesh() const;

    virtual bool directly_modified_after(PrimitiveType type, const std::vector<Tuple>& t) const;

private:
    const Mesh& m_mesh;
};
} // namespace wmtk
