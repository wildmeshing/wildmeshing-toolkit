#include "SeamlessCollapseInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>
namespace wmtk::invariants {
SeamlessCollapseInvariant::SeamlessCollapseInvariant(
    const TriMesh& m,
    std::shared_ptr<TriMesh> uv_mesh,
    const TypedAttributeHandle<double>& uv_handle)
    : Invariant(m)
    , m_uv_mesh(uv_mesh)
    , m_uv_handle(uv_handle)
{}
bool SeamlessCollapseInvariant::before(const simplex::Simplex& t) const
{
    const Tuple input_tuple_uv = mesh().map_to_child(*m_uv_mesh, t).front().tuple();

    bool v0_is_boundary = m_uv_mesh->is_boundary_vertex(input_tuple_uv);
    bool v1_is_boundary = m_uv_mesh->is_boundary_vertex(m_uv_mesh->switch_vertex(input_tuple_uv));

    if (!v0_is_boundary || !v1_is_boundary) {
        return true;
    }

    // helper function to find the next boundary edge
    auto find_next_bd_edge = [this](const Tuple input_edge_tuple) -> Tuple {
        Tuple cur_edge = input_edge_tuple;
        cur_edge = this->m_uv_mesh->switch_edge(cur_edge);

        while (!this->m_uv_mesh->is_boundary(simplex::Simplex(PrimitiveType::Edge, cur_edge))) {
            cur_edge = this->m_uv_mesh->switch_face(cur_edge);
            cur_edge = this->m_uv_mesh->switch_edge(cur_edge);
        }
        return cur_edge;
    };

    bool keep_v0 = true;
    // special invariant for ExtremeOptCollapse: keep the branch vertices
    const auto input_tuples_uv_v0 =
        mesh().map_to_child_tuples(*m_uv_mesh, simplex::Simplex::vertex(t.tuple()));
    const auto input_tuples_uv_v1 = mesh().map_to_child_tuples(
        *m_uv_mesh,
        simplex::Simplex::vertex(mesh().switch_vertex(t.tuple())));
    if (input_tuples_uv_v0.size() != 2 && input_tuples_uv_v1.size() != 2) {
        return false; // both are branch vertices, do not collapse
    }
    if (input_tuples_uv_v1.size() != 2) {
        keep_v0 = false; // v1 is branch vertex, keep v1
    }

    // the test tuple is the tuple that need to be collapsed
    Tuple test_tuple = input_tuples_uv_v0.front();
    if (keep_v0) {
        test_tuple = input_tuples_uv_v1.front();
    }
    Tuple bd_tuple_0 = find_next_bd_edge(test_tuple);
    simplex::Simplex bd_simplex_0(PrimitiveType::Edge, bd_tuple_0);
    Eigen::Matrix<double, 2, 2> rot_mat_0 = wmtk::utils::get_rotation_matrix(
        *m_uv_mesh,
        m_uv_handle,
        bd_simplex_0,
        wmtk::utils::get_pair_edge(static_cast<const TriMesh&>(mesh()), *m_uv_mesh, bd_simplex_0));
    Tuple bd_tuple_1 = find_next_bd_edge(bd_tuple_0);
    simplex::Simplex bd_simplex_1(PrimitiveType::Edge, bd_tuple_1);
    Eigen::Matrix<double, 2, 2> rot_mat_1 = wmtk::utils::get_rotation_matrix(
        *m_uv_mesh,
        m_uv_handle,
        bd_simplex_1,
        wmtk::utils::get_pair_edge(static_cast<const TriMesh&>(mesh()), *m_uv_mesh, bd_simplex_1));

    if (rot_mat_0 != rot_mat_1) {
        // std::cout << "singularity detected, do not collapse" << std::endl;
        return false;
    }
    return true;
}
} // namespace wmtk::invariants
