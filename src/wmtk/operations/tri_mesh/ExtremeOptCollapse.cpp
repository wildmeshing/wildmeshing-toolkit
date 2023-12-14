#include "ExtremeOptCollapse.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>


namespace wmtk::operations {

void OperationSettings<tri_mesh::ExtremeOptCollapse>::create_invariants()
{
    OperationSettings<tri_mesh::EdgeCollapse>::create_invariants();
    invariants->add(std::make_shared<MaxEdgeLengthInvariant>(m_mesh, position, max_squared_length));
    invariants->add(std::make_shared<TriangleInversionInvariant>(*uv_mesh_ptr, uv_handle));

    // TODO: add energy decrease invariant here
}


namespace tri_mesh {
ExtremeOptCollapse::ExtremeOptCollapse(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<ExtremeOptCollapse>& settings)
    : EdgeCollapse(m, t, settings)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_uv_accessor{settings.uv_mesh_ptr->create_accessor(settings.uv_handle)}
    , m_settings{settings}
{}

std::string ExtremeOptCollapse::name() const
{
    return "tri_mesh_collapse_edge_to_mid_extreme_opt";
}


bool ExtremeOptCollapse::before() const
{
    return TupleOperation::before();
}

bool ExtremeOptCollapse::execute()
{
    // cache endpoint data for computing the midpoint
    bool v0_is_boundary = false;
    bool v1_is_boundary = false;
    auto p0 = m_pos_accessor.vector_attribute(input_tuple()).eval();
    auto p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple())).eval();

    const auto input_tuples_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple()));
    std::vector<Eigen::VectorXd> coord0s_uv;
    std::vector<Eigen::VectorXd> coord1s_uv;
    for (const auto& input_tuple_uv : input_tuples_uv) {
        coord0s_uv.push_back(m_uv_accessor.vector_attribute(input_tuple_uv));
        coord1s_uv.push_back(
            m_uv_accessor.vector_attribute(m_settings.uv_mesh_ptr->switch_vertex(input_tuple_uv)));
    }

    if (m_settings.collapse_towards_boundary) {
        v0_is_boundary = mesh().is_boundary_vertex(input_tuple());
        v1_is_boundary = mesh().is_boundary_vertex(mesh().switch_vertex(input_tuple()));
    }

    // collapse
    {
        if (!EdgeCollapse::execute()) {
            return false;
        }
    }

    // execute according to endpoint data
    if (v0_is_boundary && !v1_is_boundary) {
        m_pos_accessor.vector_attribute(m_output_tuple) = p0;
    } else if (v1_is_boundary && !v0_is_boundary) {
        m_pos_accessor.vector_attribute(m_output_tuple) = p1;
    } else {
        m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);
    }

    const auto output_tuples_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::vertex(m_output_tuple));

    assert(output_tuples_uv.size() == coord0s_uv.size());

    for (size_t i = 0; i < output_tuples_uv.size(); ++i) {
        if (v0_is_boundary && !v1_is_boundary) {
            m_uv_accessor.vector_attribute(output_tuples_uv[i]) = coord0s_uv[i];
        } else if (v1_is_boundary && !v0_is_boundary) {
            m_uv_accessor.vector_attribute(output_tuples_uv[i]) = coord1s_uv[i];
        } else {
            m_uv_accessor.vector_attribute(output_tuples_uv[i]) =
                0.5 * (coord0s_uv[i] + coord1s_uv[i]);
        }
    }


    return true;
}

} // namespace tri_mesh
} // namespace wmtk::operations
