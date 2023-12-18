#include "ExtremeOptSwap.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/LocalFunction.hpp>
#include <wmtk/function/SYMDIR.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {

void OperationSettings<tri_mesh::ExtremeOptSwap>::create_invariants()
{
    OperationSettings<tri_mesh::EdgeSwapBase>::create_invariants();

    invariants->add(std::make_shared<InteriorEdgeInvariant>(*uv_mesh_ptr));
    invariants->add(std::make_shared<TriangleInversionInvariant>(*uv_mesh_ptr, uv_handle));

    // TODO: add energy decrease invariant here
}

namespace tri_mesh {
ExtremeOptSwap::ExtremeOptSwap(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<ExtremeOptSwap>& settings)
    : EdgeSwapBase(m, t, settings)
    , m_settings{settings}
{}


std::string ExtremeOptSwap::name() const
{
    return "tri_mesh_edge_swap_valence";
}

bool ExtremeOptSwap::execute()
{
    const auto input_tuples_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple()));

    assert(input_tuples_uv.size() == 1);
    const auto input_tuple_uv = input_tuples_uv.front();

    wmtk::function::LocalFunction symdir_local(std::make_shared<wmtk::function::SYMDIR>(
        mesh(),
        *m_settings.uv_mesh_ptr,
        m_settings.position,
        m_settings.uv_handle,
        false));

    double energy_before_swap;
    if (m_settings.optimize_E_max) {
        energy_before_swap = symdir_local.get_value_max(Simplex::edge(input_tuple_uv));
    } else {
        energy_before_swap = symdir_local.get_value(Simplex::edge(input_tuple_uv));
    }

    // std::cout << "energy before swap: " << energy_before_swap << std::endl;

    if (!tri_mesh::EdgeSwapBase::execute()) {
        return false;
    }

    const Tuple swap_output_tuple = tri_mesh::EdgeSwapBase::return_tuple();
    const Tuple swap_output_tuple_uv =
        mesh()
            .map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(swap_output_tuple))
            .front();
    double energy_after_swap;
    if (m_settings.optimize_E_max) {
        energy_after_swap = symdir_local.get_value_max(Simplex::edge(swap_output_tuple_uv));
    } else {
        energy_after_swap = symdir_local.get_value(Simplex::edge(swap_output_tuple_uv));
    }
    // std::cout << "energy after swap: " << energy_after_swap << std::endl;

    if (energy_before_swap <= energy_after_swap) {
        // std::cout << "energy not decreasing, undoing swap" << std::endl;
        return false;
    }
    return true;
}


} // namespace tri_mesh
} // namespace wmtk::operations
