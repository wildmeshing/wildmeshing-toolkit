#include "InvariantCollection.hpp"
#include <spdlog/spdlog.h>
#include <type_traits>
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {

InvariantCollection::InvariantCollection(const Mesh& m)
    : Invariant(m, true, true, true)
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

void InvariantCollection::add(std::shared_ptr<Invariant> invariant)
{
    m_invariants.emplace_back(std::move(invariant));
}
bool InvariantCollection::before(const simplex::Simplex& t) const
{
    for (const auto& invariant : m_invariants) {
        if (&mesh() != &invariant->mesh()) {
            for (const Tuple& ct : mesh().map_tuples(invariant->mesh(), t)) {
                if (!invariant->before(
                        simplex::Simplex(invariant->mesh(), t.primitive_type(), ct))) {
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
bool InvariantCollection::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    for (const auto& invariant : m_invariants) {
        if (&mesh() != &invariant->mesh()) {
            const bool invariant_uses_old_state = invariant->use_old_state_in_after();
            const bool invariant_uses_new_state = invariant->use_new_state_in_after();
            if (!(invariant_uses_old_state || invariant_uses_new_state)) {
                continue;
            }
            auto map = [&](const auto& tuples) {
                return mesh().map_tuples(invariant->mesh(), mesh().top_simplex_type(), tuples);
            };
            const std::vector<Tuple> mapped_tuples_after =
                invariant_uses_new_state ? map(top_dimension_tuples_after) : std::vector<Tuple>{};
            const std::vector<Tuple> mapped_tuples_before =
                invariant_uses_old_state ? mesh().parent_scope(map, top_dimension_tuples_before)
                                         : std::vector<Tuple>{};
            if (!invariant->after(mapped_tuples_before, mapped_tuples_after)) {
                return false;
            }
        } else {
            if (!invariant->after(top_dimension_tuples_before, top_dimension_tuples_after)) {
                return false;
            }
        }
        // assert(&mesh() != &invariant->mesh());
        // if (!invariant->after(type, tuples)) {
        //     return false;
        // }
    }
    return true;
}

bool InvariantCollection::directly_modified_after(
    const std::vector<simplex::Simplex>& simplices_before,
    const std::vector<simplex::Simplex>& simplices_after) const
{
#ifndef NDEBUG
    for (const auto& s : simplices_before) {
        mesh().parent_scope([&]() { assert(mesh().is_valid(s.tuple())); });
    }
    for (const auto& s : simplices_after) {
        assert(mesh().is_valid(s.tuple()));
    }
#endif

    for (const auto& invariant : m_invariants) {
        if (&mesh() != &invariant->mesh()) {
            auto mapped_simplices_after = mesh().map(invariant->mesh(), simplices_after);
            auto mapped_simplices_before = mesh().parent_scope(
                [&]() { return mesh().map(invariant->mesh(), simplices_before); });
#ifndef NDEBUG
            for (const auto& s : mapped_simplices_before) {
                mesh().parent_scope([&]() { assert(invariant->mesh().is_valid(s.tuple())); });
            }
            for (const auto& s : mapped_simplices_after) {
                assert(invariant->mesh().is_valid(s.tuple()));
            }
            assert(mesh().is_from_same_multi_mesh_structure(invariant->mesh()));
#endif
            if (!invariant->directly_modified_after(
                    mapped_simplices_before,
                    mapped_simplices_after)) {
                return false;
            }
        } else {
            if (!invariant->directly_modified_after(simplices_before, simplices_after)) {
                return false;
            }
        }
    }
    return true;
}

const std::shared_ptr<Invariant>& InvariantCollection::get(int64_t index) const
{
    return m_invariants.at(index);
}
int64_t InvariantCollection::size() const
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

std::map<Mesh const*, std::vector<std::shared_ptr<Invariant>>>
InvariantCollection::get_map_mesh_to_invariants()
{
    decltype(get_map_mesh_to_invariants()) mesh_invariants_map;

    throw std::runtime_error("Untested code. Potentially wrong.");

    for (std::shared_ptr<Invariant> inv : m_invariants) {
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

} // namespace wmtk::invariants
