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

    internal::configure_split(m_swap->split(), mesh, m_options);
    internal::configure_collapse(m_swap->collapse(), mesh, m_options);

    if (m_envelope_invariants) {
        m_swap->collapse().add_invariant(m_envelope_invariants);
    }

    internal::configure_swap_transfer(*m_swap, m_options.position_attribute);
    internal::finalize_swap(*m_swap, m_options);
    // m_swap = op;
}
} // namespace wmtk::components::isotropic_remeshing
