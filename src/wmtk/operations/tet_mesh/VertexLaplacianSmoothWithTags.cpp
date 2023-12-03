#include "VertexLaplacianSmoothWithTags.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/utils/HelperFunctions.hpp>

namespace wmtk::operations {

void OperationSettings<tet_mesh::VertexLaplacianSmoothWithTags>::initialize_invariants(
    const TetMesh& m)
{
    base_settings.initialize_invariants(m);
    base_settings.invariants.add(std::make_unique<InteriorVertexInvariant>(m));
    base_settings.invariants.add(std::make_unique<TodoInvariant>(m, todo_tag_handle));
} // namespace wmtk::operations

namespace tet_mesh {
VertexLaplacianSmoothWithTags::VertexLaplacianSmoothWithTags(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexLaplacianSmoothWithTags>& settings)
    : VertexAttributesUpdateBase(m, t, settings.base_settings)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
{}

std::string VertexLaplacianSmoothWithTags::name() const
{
    return "tri_mesh_vertex_laplacian_smooth_with_tags";
}

bool VertexLaplacianSmoothWithTags::execute()
{
    Eigen::Vector3d p_mid = Eigen::Vector3d::Zero();

    Accessor<long> acc_vertex_tag = mesh().create_accessor(m_settings.vertex_tag_handle);
    Accessor<long> acc_todo_tag = mesh().create_accessor(m_settings.todo_tag_handle);
    acc_todo_tag.scalar_attribute(input_tuple()) = 0;

    if (acc_vertex_tag.scalar_attribute(input_tuple()) == m_settings.embedding_tag_value) {
        const std::vector<Simplex> one_ring =
            SimplicialComplex::vertex_one_ring(mesh(), input_tuple());
        for (const Simplex& s : one_ring) {
            p_mid += m_pos_accessor.vector_attribute(s.tuple());
        }
        p_mid /= one_ring.size();
    } else if (acc_vertex_tag.scalar_attribute(input_tuple()) == m_settings.offset_tag_value) {
        Accessor<long> acc_edge_tag = mesh().create_accessor(m_settings.edge_tag_handle);
        double times = 0;
        for (const Simplex& s :
             SimplicialComplex::open_star(mesh(), Simplex(PrimitiveType::Vertex, input_tuple()))
                 .get_edges()) {
            const Tuple& t = s.tuple();
            if (acc_edge_tag.scalar_attribute(t) == m_settings.offset_tag_value) {
                p_mid += m_pos_accessor.vector_attribute(mesh().switch_vertex(t));
                ++times;
            }
        }
        if (times <= 2) {
            throw std::runtime_error("offset is a non-manifold!" + std::to_string(times));
        }
        p_mid /= times;
        // we don't need a full smoothing step
        // p_mid = 0.2 * p_mid + 0.8 * m_pos_accessor.const_vector_attribute(input_tuple());
    } else {
        long debug_value = acc_vertex_tag.scalar_attribute(input_tuple());
        throw std::runtime_error(
            std::string("unexpected vertex tag!") + std::to_string(debug_value));
    }

    operations::utils::optimize_position(
        mesh(),
        m_settings.position,
        input_tuple(),
        p_mid,
        m_pos_accessor.const_vector_attribute(input_tuple()),
        PrimitiveType::Tetrahedron);

    return tet_mesh::VertexAttributesUpdateBase::execute();
}

} // namespace tet_mesh
} // namespace wmtk::operations
