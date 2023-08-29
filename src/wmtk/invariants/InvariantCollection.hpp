#pragma once
#include <memory>
#include <vector>
#include "Invariant.hpp"


namespace wmtk {
class InvariantCollection : public Invariant
{
public:
    InvariantCollection();
    InvariantCollection(const InvariantCollection&);
    InvariantCollection(InvariantCollection&&);
    InvariantCollection& operator=(const InvariantCollection&);
    InvariantCollection& operator=(InvariantCollection&&);
    ~InvariantCollection();
    bool before(const Tuple& t) const override;
    bool after(PrimitiveType type, const std::vector<Tuple>& t) const override;


    // pass by value so this can be internally moved
    void add(std::shared_ptr<Invariant> invariant);

    const std::shared_ptr<Invariant>& get(long index) const;
    long size() const;
    bool empty() const;
    const std::vector<std::shared_ptr<Invariant>>& invariants() const;

private:
    std::vector<std::shared_ptr<Invariant>> m_invariants;
};

class Mesh;

// An invariant collection that checks that tuples are valid and not outdated
InvariantCollection basic_invariant_collection(const Mesh& m);

} // namespace wmtk
