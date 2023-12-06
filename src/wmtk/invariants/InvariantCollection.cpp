#include "InvariantCollection.hpp"
#include <wmtk/Mesh.hpp>
#include "ValidTupleInvariant.hpp"

namespace wmtk {
InvariantCollection::InvariantCollection() = default;

InvariantCollection::InvariantCollection(const Mesh& m)
    : MeshInvariant(m)
{}
InvariantCollection::~InvariantCollection() = default;
InvariantCollection::InvariantCollection(const InvariantCollection&) = default;
InvariantCollection::InvariantCollection(InvariantCollection&&) = default;
InvariantCollection& InvariantCollection::operator=(const InvariantCollection& o)
{
    assert(o.mesh() == mesh());
    m_invariants = o.m_invariants;
    return *this;
}
InvariantCollection& InvariantCollection::operator=(InvariantCollection&& o)
{
    assert(o.mesh() == mesh());
    m_invariants = std::move(o.m_invariants);
    return *this;
}

void InvariantCollection::add(std::shared_ptr<MeshInvariant> invariant)
{
    m_invariants.emplace_back(std::move(invariant));
}
bool InvariantCollection::before(const Tuple& t) const
{
    for (const auto& invariant : m_invariants) {
        // if (&mesh() != &invariant->mesh()) {
        //     for (const Tuple& ct : mesh().map_tuples(invariant->mesh(), Simplex(type, t))) {
        //         if (!invariant->before(t)) {
        //             return false;
        //         }
        //     }
        // } else {
        //     if (!invariant->before(t)) {
        //         return false;
        //     }
        // }
        if (!invariant->before(t)) {
            return false;
        }
    }
    return true;
}
bool InvariantCollection::after(PrimitiveType type, const std::vector<Tuple>& tuples) const
{
    for (const auto& invariant : m_invariants) {
        // if (&mesh() != &invariant->mesh()) {
        //     auto mapped_tuples = mesh().map_tuples(invariant->mesh(), type, tuples);
        //     if (!invariant->after(type, mapped_tuples)) {
        //         return false;
        //     }
        // } else {
        //     if (!invariant->after(type, tuples)) {
        //         return false;
        //     }
        // }
        if (!invariant->after(type, tuples)) {
            return false;
        }
    }
    return true;
}

InvariantCollection basic_invariant_collection(const Mesh& m)
{
    InvariantCollection ic(m);
    ic.add(std::make_shared<ValidTupleInvariant>(m));
    return ic;
}

InvariantCollection basic_multimesh_invariant_collection(const Mesh& m, PrimitiveType)
{
    InvariantCollection ic(m);
    ic.add(std::make_shared<ValidTupleInvariant>(m));
    return ic;
}

const std::shared_ptr<MeshInvariant>& InvariantCollection::get(long index) const
{
    return m_invariants.at(index);
}
long InvariantCollection::size() const
{
    return m_invariants.size();
}
bool InvariantCollection::empty() const
{
    return m_invariants.empty();
}
const std::vector<std::shared_ptr<MeshInvariant>>& InvariantCollection::invariants() const
{
    return m_invariants;
}
} // namespace wmtk
