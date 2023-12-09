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

    virtual bool directly_modified_after(const std::vector<Simplex>& t) const;

private:
    const Mesh& m_mesh;
};
} // namespace wmtk
