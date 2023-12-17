#include "ExtremeOptSplit.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::ExtremeOptSplit>::create_invariants()
{
    OperationSettings<tri_mesh::EdgeSplit>::create_invariants();

    invariants->add(
        std::make_shared<MinEdgeLengthInvariant>(*uv_mesh_ptr, uv_handle, min_squared_length));
}


namespace tri_mesh {
ExtremeOptSplit::ExtremeOptSplit(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<ExtremeOptSplit>& settings)
    : EdgeSplit(m, t, settings)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_uv_accessor{settings.uv_mesh_ptr->create_accessor(settings.uv_handle)}
    , m_settings{settings}
{}

std::vector<double> ExtremeOptSplit::priority() const
{
    const auto input_tuple_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple())).front();
    auto uv0 = m_uv_accessor.vector_attribute(input_tuple_uv);
    auto uv1 =
        m_uv_accessor.vector_attribute(m_settings.uv_mesh_ptr->switch_vertex(input_tuple_uv));
    return {-(uv0 - uv1).norm()};
}
std::string ExtremeOptSplit::name() const
{
    return "tri_mesh_split_edge_at_midpoint_extreme_opt";
}

bool ExtremeOptSplit::before() const
{
    return TupleOperation::before();
}
bool ExtremeOptSplit::execute()
{
    Eigen::VectorXd coord0 = m_pos_accessor.vector_attribute(input_tuple());
    Eigen::VectorXd coord1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));

    const auto input_tuples_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple()));
    std::vector<Eigen::VectorXd> coord0s_uv;
    std::vector<Eigen::VectorXd> coord1s_uv;
    for (const auto& input_tuple_uv : input_tuples_uv) {
        coord0s_uv.push_back(m_uv_accessor.vector_attribute(input_tuple_uv));
        coord1s_uv.push_back(
            m_uv_accessor.vector_attribute(m_settings.uv_mesh_ptr->switch_vertex(input_tuple_uv)));
    }

    if (!EdgeSplit::execute()) {
        return false;
    }

    auto split_output_tuple = EdgeSplit::return_tuple();
    m_pos_accessor.vector_attribute(split_output_tuple) = 0.5 * (coord0 + coord1);

    const auto output_tuples_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::vertex(split_output_tuple));

    assert(output_tuples_uv.size() == coord0s_uv.size());

    for (size_t i = 0; i < output_tuples_uv.size(); ++i) {
        m_uv_accessor.vector_attribute(output_tuples_uv[i]) = 0.5 * (coord0s_uv[i] + coord1s_uv[i]);
    }

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
