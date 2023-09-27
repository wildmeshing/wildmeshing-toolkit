#include "InvariantCollection.hpp"
#include "ValidTupleInvariant.hpp"

namespace wmtk {
InvariantCollection::InvariantCollection() = default;
InvariantCollection::~InvariantCollection() = default;
InvariantCollection::InvariantCollection(const InvariantCollection&) = default;
InvariantCollection::InvariantCollection(InvariantCollection&&) = default;
InvariantCollection& InvariantCollection::operator=(const InvariantCollection&) = default;
InvariantCollection& InvariantCollection::operator=(InvariantCollection&&) = default;

void InvariantCollection::add(std::shared_ptr<Invariant> invariant)
{
    m_invariants.emplace_back(std::move(invariant));
}
bool InvariantCollection::before(const Tuple& t) const
{
    for (const auto& invariant : m_invariants) {
        if (!invariant->before(t)) {
            return false;
        }
    }
    return true;
}
bool InvariantCollection::after(PrimitiveType type, const std::vector<Tuple>& tuples) const
{
    for (const auto& invariant : m_invariants) {
        if (!invariant->after(type, tuples)) {
            return false;
        }
    }
    return true;
}

InvariantCollection basic_invariant_collection(const Mesh& m)
{
    InvariantCollection ic;
    ic.add(std::make_shared<ValidTupleInvariant>(m));
    return ic;
}

const std::shared_ptr<Invariant>& InvariantCollection::get(long index) const
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
const std::vector<std::shared_ptr<Invariant>>& InvariantCollection::invariants() const
{
    return m_invariants;
}
} // namespace wmtk
