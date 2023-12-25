#include "ExtremeOptSwap.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/LocalFunction.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/TriangleDegenerateInvariant.hpp>
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
    invariants->add(std::make_shared<TriangleDegenerateInvariant>(m_mesh, position));
}

namespace tri_mesh {
ExtremeOptSwap::ExtremeOptSwap(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<ExtremeOptSwap>& settings)
    : EdgeSwapBase(m, t, settings)
    , m_settings{settings}
{
    if (m_settings.optimize_E_max) {
        symdir_ptr = std::make_shared<wmtk::function::SYMDIR>(
            mesh(),
            *m_settings.uv_mesh_ptr,
            m_settings.position,
            m_settings.uv_handle,
            false);
    } else {
        // we optimize E_sum, need to do integral here
        symdir_ptr = std::make_shared<wmtk::function::SYMDIR>(
            mesh(),
            *m_settings.uv_mesh_ptr,
            m_settings.position,
            m_settings.uv_handle,
            true);
    }
}


std::string ExtremeOptSwap::name() const
{
    return "tri_mesh_edge_swap_valence";
}

Tuple ExtremeOptSwap::get_input_tuple_uv() const
{
    const auto input_tuples_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple()));
    assert(input_tuples_uv.size() == 1);
    return input_tuples_uv.front();
}

Tuple ExtremeOptSwap::get_output_tuple_uv() const
{
    const auto output_tuples_uv = mesh().map_to_child_tuples(
        *m_settings.uv_mesh_ptr,
        Simplex::edge(tri_mesh::EdgeSwapBase::return_tuple()));
    assert(output_tuples_uv.size() == 1);
    return output_tuples_uv.front();
}

bool ExtremeOptSwap::execute()
{
    wmtk::function::LocalFunction symdir_local(symdir_ptr);

    // get input tuple from uv mesh
    const Tuple input_tuple_uv = get_input_tuple_uv();

    // evaluate energy before swap
    double energy_before_swap;
    if (m_settings.optimize_E_max) {
        energy_before_swap = symdir_local.get_value_max(Simplex::edge(input_tuple_uv));
    } else {
        energy_before_swap = symdir_local.get_value(Simplex::edge(input_tuple_uv));
    }

    // execute swap
    if (!tri_mesh::EdgeSwapBase::execute()) {
        return false;
    }

    // get output tuple from uv mesh
    const Tuple swap_output_tuple_uv = get_output_tuple_uv();

    // evaluate energy after swap
    double energy_after_swap;
    if (m_settings.optimize_E_max) {
        energy_after_swap = symdir_local.get_value_max(Simplex::edge(swap_output_tuple_uv));
    } else {
        energy_after_swap = symdir_local.get_value(Simplex::edge(swap_output_tuple_uv));
    }

    if (std::isinf(energy_after_swap) || std::isnan(energy_after_swap)) {
        return false;
    }
    // check if energy decreased
    if (energy_before_swap <= energy_after_swap) {
        return false;
    }
    return true;
}


} // namespace tri_mesh
} // namespace wmtk::operations
