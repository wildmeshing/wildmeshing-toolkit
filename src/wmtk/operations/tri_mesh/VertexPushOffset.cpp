#include "VertexPushOffset.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/utils/HelperFunctions.hpp>
#include <wmtk/simplex/link.hpp>

namespace wmtk::operations {

void OperationSettings<tri_mesh::VertexPushOffset>::initialize_invariants(const TriMesh& m)
{
    base_settings.initialize_invariants(m);
    base_settings.invariants.add(std::make_unique<InteriorVertexInvariant>(m));
    base_settings.invariants.add(std::make_unique<TodoInvariant>(m, todo_tag_handle));
} // namespace wmtk::operations

namespace tri_mesh {
VertexPushOffset::VertexPushOffset(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexPushOffset>& settings)
    : VertexAttributesUpdateBase(m, t, settings.base_settings)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
{}

std::string VertexPushOffset::name() const
{
    return "tri_mesh_vertex_push_offset";
}

bool VertexPushOffset::execute()
{
    Accessor<long> acc_vertex_tag = mesh().create_accessor(m_settings.vertex_tag_handle);
    Accessor<long> acc_edge_tag = mesh().create_accessor(m_settings.edge_tag_handle);
    Accessor<double> acc_pos = mesh().create_accessor(m_settings.position);
    Accessor<long> acc_todo = mesh().create_accessor(m_settings.todo_tag_handle);

    long tag = acc_vertex_tag.scalar_attribute(input_tuple());

    acc_todo.scalar_attribute(input_tuple()) = 0;
    Tuple itr;
    if (acc_vertex_tag.scalar_attribute(input_tuple()) == m_settings.offset_tag_value) {
        itr = input_tuple();
    } else if (acc_vertex_tag.scalar_attribute(input_tuple()) == m_settings.input_tag_value) {
        itr = mesh().switch_vertex(input_tuple());
    } else {
        throw std::runtime_error("error for the edges waiting for pushing!");
    }

    std::vector<Tuple> near_input_edges;
    wmtk::simplex::SimplexCollection sc =
        simplex::link(mesh(), Simplex(PrimitiveType::Vertex, input_tuple()));
    for (const Simplex& s : sc.simplex_vector(PrimitiveType::Edge)) {
        const Tuple& t = s.tuple();
        if (acc_edge_tag.scalar_attribute(t) == m_settings.input_tag_value) {
            near_input_edges.push_back(t);
        }
    }

    Eigen::Vector3d projection;
    Eigen::Vector3d offset_pos = acc_pos.vector_attribute(itr);
    double min_len = std::numeric_limits<double>::max();
    if (near_input_edges.size() == 0) {
        projection = acc_pos.vector_attribute(mesh().switch_vertex(itr));
    } else {
        for (int i = 0; i < near_input_edges.size(); ++i) {
            Eigen::Vector3d candidate_p =
                utils::nearest_point_to_edge(mesh(), m_settings.position, near_input_edges[i], itr);
            if ((candidate_p - offset_pos).norm() < min_len) {
                min_len = (candidate_p - offset_pos).norm();
                projection = candidate_p;
            }
        }
    }

    utils::push_offset(
        mesh(),
        m_settings.position,
        itr,
        projection,
        m_settings.offset_len,
        PrimitiveType::Face);

    return tri_mesh::VertexAttributesUpdateBase::execute();
}

} // namespace tri_mesh
} // namespace wmtk::operations
