#include "Operation.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/simplex/IdSimplexCollection.hpp>
#include <wmtk/simplex/closed_star_iterable.hpp>


// it's ugly but for teh visitor we need these included
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations {


Operation::Operation(Mesh& mesh)
    : m_mesh(mesh)
    , m_invariants(mesh)
{}

Operation::~Operation() = default;


std::shared_ptr<const operations::AttributeTransferStrategyBase> Operation::get_transfer_strategy(
    const attribute::MeshAttributeHandle& attribute)
{
    assert(attribute.is_same_mesh(mesh()));

    for (auto& s : m_attr_transfer_strategies) {
        if (s->matches_attribute(attribute)) return s;
    }

    throw std::runtime_error("unable to find attribute");
}

void Operation::clear_attribute_transfer_strategies()
{
    m_attr_transfer_strategies.clear();
}

void Operation::set_transfer_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const std::shared_ptr<const operations::AttributeTransferStrategyBase>& other)
{
    assert(attribute.is_same_mesh(mesh()));

    for (auto& s : m_attr_transfer_strategies) {
        if (s->matches_attribute(attribute)) {
            s = other;
            return;
        }
    }

    throw std::runtime_error("unable to find attribute");
}

void Operation::add_transfer_strategy(
    const std::shared_ptr<const operations::AttributeTransferStrategyBase>& other)
{
    spdlog::debug("Adding a transfer");
    m_attr_transfer_strategies.emplace_back(other);
}

std::vector<simplex::Simplex> Operation::operator()(const simplex::Simplex& simplex)
{
    if (!before(simplex)) {
        return {};
    }

    const auto simplex_resurrect = simplex;

    auto scope = mesh().create_scope();
    assert(simplex.primitive_type() == primitive_type());

#if defined(MTAO_CONSTANTLY_VERIFY_MESH)
    assert(mesh().is_connectivity_valid());
#endif
    try {
        auto unmods = unmodified_primitives(simplex_resurrect);
#if defined(MTAO_CONSTANTLY_VERIFY_MESH)
    assert(mesh().is_connectivity_valid());
#endif
        auto mods = execute(simplex_resurrect);
#if defined(MTAO_CONSTANTLY_VERIFY_MESH)
    assert(mesh().is_connectivity_valid());
#endif
        if (!mods.empty()) { // success should be marked here
            apply_attribute_transfer(mods);
            if (after(unmods, mods)) {
                return mods; // scope destructor is called
            }
        }
    } catch (const std::exception& e) {
        scope.mark_failed();
        throw e;
    }
    scope.mark_failed();
    return {}; // scope destructor is called
}

bool Operation::before(const simplex::Simplex& simplex) const
{
    // const attribute::Accessor<int64_t> accessor = hash_accessor();

    // if (!mesh().is_valid(
    //         simplex.tuple(),
    //         accessor)) { // TODO: chang to is_removed and resurrect later
    //     return false;
    // }

    // we assume the current MeshType's is_valid calls Mesh::is_valid first, which checks if the simplex is removed or not 
    if(!mesh().is_valid(simplex)) {
        return false;
    }

    const auto simplex_resurrect = simplex;

    // map simplex to the invariant mesh
    const Mesh& invariant_mesh = m_invariants.mesh();

    if (&invariant_mesh == &mesh()) {
        if (!m_invariants.before(simplex_resurrect)) {
            return false;
        }
    } else {
        // TODO check if this is correct
        const std::vector<simplex::Simplex> invariant_simplices =
            m_mesh.map(invariant_mesh, simplex_resurrect);

        for (const simplex::Simplex& s : invariant_simplices) {
            if (!m_invariants.before(s)) {
                return false;
            }
        }
    }

    return true;
}

bool Operation::after(
    const std::vector<simplex::Simplex>& unmods,
    const std::vector<simplex::Simplex>& mods) const
{
    return m_invariants.directly_modified_after(unmods, mods);
}

void Operation::apply_attribute_transfer(const std::vector<simplex::Simplex>& direct_mods)
{
    if (m_attr_transfer_strategies.size() == 0) {
        return;
    }

    simplex::IdSimplexCollection all(m_mesh);
    all.reserve(100);

    assert(m_mesh.is_connectivity_valid());

    for (const auto& s : direct_mods) {
        if (!s.tuple().is_null()) {
            assert(m_mesh.is_valid(s));
            for (const simplex::IdSimplex& ss : simplex::closed_star_iterable(m_mesh, s)) {
                // trying to get a simplex and this crashes
                m_mesh.get_simplex(ss);
                all.add(ss);
            }
        }
    }
    if (direct_mods.size() > 1) {
        all.sort_and_clean();
    }

    for (const auto& at_ptr : m_attr_transfer_strategies) {
        if (&m_mesh == &(at_ptr->mesh())) {
            for (const simplex::IdSimplex& s : all.simplex_vector()) {
                assert(m_mesh.get_const_flag_accessor(s.primitive_type()).is_active(s));
                if (s.primitive_type() == at_ptr->primitive_type()) {
                    at_ptr->run(m_mesh.get_simplex(s));
                }
            }
        } else {
            auto& at_mesh = at_ptr->mesh();
            auto at_mesh_simplices = m_mesh.map(at_mesh, direct_mods);

            simplex::IdSimplexCollection at_mesh_all(at_mesh);
            for (const simplex::Simplex& s : at_mesh_simplices) {
                for (const simplex::IdSimplex& ss : simplex::closed_star_iterable(at_mesh, s)) {
                    at_mesh_all.add(ss);
                }
            }

            at_mesh_all.sort_and_clean();

            for (const simplex::IdSimplex& s : at_mesh_all.simplex_vector()) {
                if (s.primitive_type() == at_ptr->primitive_type()) {
                    at_ptr->run(at_mesh.get_simplex(s));
                }
            }
        }
    }
}


void Operation::reserve_enough_simplices()
{
    // by default assume we'll at most create N * capacity
    constexpr static int64_t default_preallocation_size = 3;

    auto run = [&](auto&& m) {
        if constexpr (!std::is_const_v<std::remove_reference_t<decltype(m)>>) {
            auto cap = m.m_attribute_manager.m_capacities;
            for (auto& v : cap) {
                v *= default_preallocation_size;
            }
            m.reserve_more_attributes(cap);
        }
    };
    multimesh::MultiMeshVisitor visitor(run);
    visitor.execute_from_root(mesh());
}

} // namespace wmtk::operations
