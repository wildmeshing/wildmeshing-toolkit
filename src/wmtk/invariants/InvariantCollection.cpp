#include "InvariantCollection.hpp"
#include <type_traits>
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
bool InvariantCollection::before(const Simplex& t) const
{
    spdlog::info("Going through an IC with {} invariants", m_invariants.size());
    for (const auto& invariant : m_invariants) {
        if (&mesh() != &invariant->mesh()) {
            for (const Tuple& ct : mesh().map_tuples(invariant->mesh(), t)) {
                if (!invariant->before(t)) {
                    return false;
                }
            }
        } else {
            if (!invariant->before(t)) {
                return false;
            }
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

std::map<Mesh const *, std::vector<std::shared_ptr<MeshInvariant>>>
InvariantCollection::get_map_mesh_to_invariants()
{
    decltype(get_map_mesh_to_invariants()) mesh_invariants_map;

    throw std::runtime_error("Untested code. Potentially wrong.");

    for (std::shared_ptr<MeshInvariant> inv : m_invariants) {
        // TODO check if that if statement is correct
        if (std::is_base_of<InvariantCollection, decltype(inv)::element_type>()) {
            // go through invariant collections
            InvariantCollection& sub_ic = static_cast<InvariantCollection&>(*inv);
            decltype(mesh_invariants_map) sub_map = sub_ic.get_map_mesh_to_invariants();

            for (const auto& [mptr, i] : sub_map) {
                auto& vec = mesh_invariants_map[mptr];
                vec.insert(vec.end(), i.begin(), i.end());
            }
        } else {
            mesh_invariants_map[&(inv->mesh())].push_back(inv);
        }
    }
    //
}

} // namespace wmtk
