#include "MappableContainer.hpp"
#include <type_traits>
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {

MappableContainer::MappableContainer(const Mesh& m)
    : Mappable(m)
{}
MappableContainer::~MappableContainer() = default;
MappableContainer::MappableContainer(const MappableContainer&) = default;
MappableContainer::MappableContainer(MappableContainer&&) = default;
MappableContainer& MappableContainer::operator=(const MappableContainer& o)
{
    assert(o.mesh() == mesh());
    m_mappables = o.m_mappables;
    return *this;
}
MappableContainer& MappableContainer::operator=(MappableContainer&& o)
{
    assert(o.mesh() == mesh());
    m_mappables = std::move(o.m_mappables);
    return *this;
}

void MappableContainer::add(std::shared_ptr<Mappable> invariant)
{
    m_mappables.emplace_back(std::move(invariant));
}
bool MappableContainer::before(const simplex::Simplex& t) const
{
    for (const auto& invariant : m_mappables) {
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
bool MappableContainer::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    for (const auto& invariant : m_mappables) {
        if (&mesh() != &invariant->mesh()) {
            auto mapped_tuples_after = mesh().map_tuples(
                invariant->mesh(),
                mesh().top_simplex_type(),
                top_dimension_tuples_after);
            auto mapped_tuples_before = mesh().parent_scope([&]() {
                return mesh().map_tuples(
                    invariant->mesh(),
                    mesh().top_simplex_type(),
                    top_dimension_tuples_before);
            });
            if (!invariant->after(mapped_tuples_before, mapped_tuples_after)) {
                return false;
            }
        } else {
            if (!invariant->after(top_dimension_tuples_before, top_dimension_tuples_after)) {
                return false;
            }
        }
        assert(&mesh() != &invariant->mesh());
        // if (!invariant->after(type, tuples)) {
        //     return false;
        // }
    }
    return true;
}

bool MappableContainer::directly_modified_after(
    const std::vector<simplex::Simplex>& simplices_before,
    const std::vector<simplex::Simplex>& simplices_after) const
{
    for (const auto& invariant : m_mappables) {
        if (&mesh() != &invariant->mesh()) {
            auto mapped_simplices_after = mesh().map(invariant->mesh(), simplices_after);
            auto mapped_simplices_before = mesh().parent_scope(
                [&]() { return mesh().map(invariant->mesh(), simplices_before); });
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

const std::shared_ptr<Mappable>& MappableContainer::get(int64_t index) const
{
    return m_mappables.at(index);
}
int64_t MappableContainer::size() const
{
    return m_mappables.size();
}
bool MappableContainer::empty() const
{
    return m_mappables.empty();
}
const std::vector<std::shared_ptr<Mappable>>& MappableContainer::invariants() const
{
    return m_mappables;
}

std::map<Mesh const*, std::vector<std::shared_ptr<Mappable>>>
MappableContainer::get_map_mesh_to_invariants()
{
    decltype(get_map_mesh_to_invariants()) mesh_invariants_map;

    throw std::runtime_error("Untested code. Potentially wrong.");

    for (std::shared_ptr<Mappable> inv : m_mappables) {
        // TODO check if that if statement is correct
        if (std::is_base_of<MappableContainer, decltype(inv)::element_type>()) {
            // go through invariant collections
            MappableContainer& sub_ic = static_cast<MappableContainer&>(*inv);
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
