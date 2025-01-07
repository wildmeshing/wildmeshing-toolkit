#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/EdgeSwap.hpp>
#include <wmtk/operations/composite/TetEdgeSwap.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/utils/Logger.hpp>
#include "IsotropicRemeshing.hpp"
#include "internal/configure_collapse.hpp"
#include "internal/configure_split.hpp"
#include "internal/configure_swap.hpp"
namespace wmtk::components::isotropic_remeshing {

void IsotropicRemeshing::configure_swap()
{
    // adds common invariants like inversion check and asserts taht the swap is ready for prime time
    wmtk::logger().debug("Configure isotropic remeshing swap");
    if (m_options.edge_swap_mode == EdgeSwapMode::Skip) {
        return;
    }
    wmtk::Mesh& mesh = m_options.position_attribute.mesh();
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle:
        m_swap = std::make_shared<operations::composite::TriEdgeSwap>(static_cast<TriMesh&>(mesh));
        break;
    case PrimitiveType::Tetrahedron:
        m_swap = std::make_shared<operations::composite::TetEdgeSwap>(static_cast<TetMesh&>(mesh));
        break;
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    default: assert(false); break;
    }


    const std::optional<attribute::MeshAttributeHandle>& position_for_inversion =
        m_options.inversion_position_attribute;

    switch (m_options.edge_swap_mode) {
    case EdgeSwapMode::Valence: {
        auto tri = dynamic_cast<TriMesh*>(&mesh);
        if (tri == nullptr) {
            throw std::runtime_error(
                "EdgeSwap only works with trimesh, got a different type of mesh");
        }
        auto invariant_valence_improve =
            std::make_shared<invariants::ValenceImprovementInvariant>(*tri);
        m_swap->add_invariant(invariant_valence_improve);
        break;
    }
    case EdgeSwapMode::AMIPS: {
    }
    default:
    case EdgeSwapMode::Skip: {
        assert(false);
    }
    }
    // if (position_for_inversion) {
    //     m_swap->collapse().add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
    //         position_for_inversion.value().mesh(),
    //         position_for_inversion.value().as<double>()));
    // }
    //  m_swap->add_invariant(m_interior_edge_invariants);

    // internal::configure_split(m_swap->split(), mesh, m_options);
    // internal::configure_collapse(m_swap->collapse(), mesh, m_options);
    m_swap->collapse().add_invariant(internal::collapse_core_invariants(mesh, m_options));


    // if (m_envelope_invariants) {
    //     m_swap->add_invariant(m_envelope_invariants);
    // }


    for (const auto& p : m_options.all_positions()) {
        internal::configure_swap_transfer(*m_swap, p);
    }
    // clear out all the pass through attributes
    for (const auto& attr : m_options.pass_through_attributes) {
        m_swap->split().set_new_attribute_strategy(attr);
        m_swap->collapse().set_new_attribute_strategy(attr);
    }
    internal::finalize_swap(*m_swap, m_options);
    // m_swap = op;
}
} // namespace wmtk::components::isotropic_remeshing
