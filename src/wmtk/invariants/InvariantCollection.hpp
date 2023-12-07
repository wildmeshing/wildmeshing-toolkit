#pragma once
#include <memory>
#include <vector>
#include "MeshInvariant.hpp"


namespace wmtk {
class InvariantCollection : public MeshInvariant
{
public:
    InvariantCollection(); // TODO HACK REMOVE THIS!!!
    InvariantCollection(const Mesh& m);
    InvariantCollection(const InvariantCollection&);
    InvariantCollection(InvariantCollection&&);
    InvariantCollection& operator=(const InvariantCollection&);
    InvariantCollection& operator=(InvariantCollection&&);
    ~InvariantCollection();
    bool before(const Simplex& t) const override;
    bool after(PrimitiveType type, const std::vector<Tuple>& t) const override;


    // pass by value so this can be internally moved
    void add(std::shared_ptr<MeshInvariant> invariant);

    const std::shared_ptr<MeshInvariant>& get(long index) const;
    long size() const;
    bool empty() const;
    const std::vector<std::shared_ptr<MeshInvariant>>& invariants() const;

private:
    std::vector<std::shared_ptr<MeshInvariant>> m_invariants;
};

class Mesh;

} // namespace wmtk
