#include "Operation.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>


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


std::shared_ptr<operations::AttributeTransferStrategyBase> Operation::get_transfer_strategy(
    const attribute::MeshAttributeHandle& attribute)
{
    assert(attribute.is_same_mesh(mesh()));

    for (auto& s : m_attr_transfer_strategies) {
        if (s->matches_attribute(attribute)) return s;
    }

    throw std::runtime_error("unable to find attribute");
}

void Operation::set_transfer_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const std::shared_ptr<operations::AttributeTransferStrategyBase>& other)
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
    const std::shared_ptr<operations::AttributeTransferStrategyBase>& other)
{
    m_attr_transfer_strategies.emplace_back(other);
}

std::vector<simplex::Simplex> Operation::operator()(const simplex::Simplex& simplex)
{
    auto scope = mesh().create_scope();
    assert(simplex.primitive_type() == primitive_type());

    if (before(simplex)) {
        auto unmods = unmodified_primitives(simplex);
        auto mods = execute(simplex);
        if (!mods.empty()) { // success should be marked here
            apply_attribute_transfer(mods);
            if (after(unmods, mods)) {
                return mods; // scope destructor is called
            }
        }
    }
    scope.mark_failed();
    return {}; // scope destructor is called
}

bool Operation::before(const simplex::Simplex& simplex) const
{
    ConstAccessor<int64_t> accessor = hash_accessor();

    if (!mesh().is_valid(simplex.tuple(), accessor)) {
        return false;
    }

    // map simplex to the invariant mesh
    const Mesh& invariant_mesh = m_invariants.mesh();

    // TODO check if this is correct
    const std::vector<simplex::Simplex> invariant_simplices = m_mesh.map(invariant_mesh, simplex);

    for (const simplex::Simplex& s : invariant_simplices) {
        if (!m_invariants.before(s)) {
            return false;
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
    // TODO: this has no chance of working in multimesh
    simplex::SimplexCollection all(m_mesh);
    for (const auto& s : direct_mods) {
        all.add(simplex::closed_star(m_mesh, s, false));
    }
    all.sort_and_clean();
    for (const auto& at_ptr : m_attr_transfer_strategies) {
        if (&m_mesh == &(at_ptr->mesh())) {
            for (const auto& s : all.simplex_vector()) {
                if (s.primitive_type() == at_ptr->primitive_type()) {
                    at_ptr->run(s);
                }
            }
        } else {
            auto& at_mesh = at_ptr->mesh();
            auto at_mesh_simplices = m_mesh.map(at_mesh, direct_mods);

            simplex::SimplexCollection at_mesh_all(at_mesh);
            for (const auto& s : at_mesh_simplices) {
                at_mesh_all.add(simplex::closed_star(at_mesh, s));
            }

            at_mesh_all.sort_and_clean();

            for (const auto& s : at_mesh_all.simplex_vector()) {
                if (s.primitive_type() == at_ptr->primitive_type()) {
                    at_ptr->run(s);
                }
            }
        }
    }
}

Tuple Operation::resurrect_tuple(const Tuple& tuple) const
{
    return mesh().resurrect_tuple(tuple, hash_accessor());
}

Accessor<int64_t> Operation::hash_accessor()
{
    return m_mesh.get_cell_hash_accessor();
}

ConstAccessor<int64_t> Operation::hash_accessor() const
{
    return m_mesh.get_const_cell_hash_accessor();
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
            m.guarantee_at_least_attributes(cap);
        }
    };
    multimesh::MultiMeshVisitor visitor(run);
    visitor.execute_from_root(mesh());
}

} // namespace wmtk::operations
