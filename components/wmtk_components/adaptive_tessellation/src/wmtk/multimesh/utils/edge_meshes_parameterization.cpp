#include "edge_meshes_parameterization.hpp"
#include <wmtk/Accessor.hpp>

using namespace wmtk;
namespace wmtk::components::adaptive_tessellation::multimesh::utils {
std::pair<Tuple, Tuple> get_ends_of_edge_mesh(const EdgeMesh& edge_mesh)
{
    // get the ends of the edge mesh
    std::vector<Tuple> ends;
    std::vector<Tuple> vertices = edge_mesh.get_all(PrimitiveType::Vertex);
    for (const Tuple& v : vertices) {
        if (edge_mesh.is_boundary(v, PrimitiveType::Vertex)) ends.push_back(v);
    }
    if (ends.size() != 2) throw std::runtime_error("The edge mesh is a closed curve!");
    return std::make_pair(ends[0], ends[1]);
}

void parameterize_edge_mesh(
    EdgeMesh& edge_mesh,
    const TriMesh& uv_mesh,
    MeshAttributeHandle<double>& t_handle,
    const MeshAttributeHandle<double>& uv_handle)
{
    // create an t accessor
    Accessor<double> t_accessor = edge_mesh.create_accessor(t_handle);
    // create an uv accessor
    ConstAccessor<double> uv_accessor = uv_mesh.create_const_accessor(uv_handle);
    // get ends of the edge mesh
    std::pair<Tuple, Tuple> ends = get_ends_of_edge_mesh(edge_mesh);
    Tuple v_start = ends.first;
    Tuple v_end = ends.second;
    Tuple v = v_start;
    Tuple v_next = edge_mesh.switch_tuples(v, {PrimitiveType::Vertex});
    t_accessor.scalar_attribute(v) = 0.;

    while (v_next != v_end) {
        // map v and v_next to uv_mesh
        Tuple uv_v = edge_mesh.map(uv_mesh, Simplex::vertex(v));
        Tuple uv_v_next = edge_mesh.map(uv_mesh, Simplex::vertex(v_next));
        // compute the arclength between v and v_next
        double arclength = (uv_accessor.const_vector_attribute(uv_v) -
                            uv_accessor.const_vector_attribute(uv_v_next))
                               .stableNorm();
        // update t
        t_accessor.scalar_attribute(v_next) = t_accessor.scalar_attribute(v) + arclength;
        // update v and v_next
        v = v_next;
        v_next = edge_mesh.switch_tuples(v_next, {PrimitiveType::Edge, PrimitiveType::Vertex});
    }
}

void parameterize_seam_edge_meshes(
    EdgeMesh& edge_mesh1,
    EdgeMesh& edge_mesh2,
    const TriMesh& uv_mesh,
    MeshAttributeHandle<double>& t1_handle,
    MeshAttributeHandle<double>& t2_handle,
    const MeshAttributeHandle<double>& uv_handle)
{
    // create an t accessor
    Accessor<double> t1_accessor = edge_mesh1.create_accessor(t1_handle);
    Accessor<double> t2_accessor = edge_mesh2.create_accessor(t2_handle);
    // create an uv accessor
    ConstAccessor<double> uv_accessor = uv_mesh.create_const_accessor(uv_handle);
    // get ends of the edge mesh
    std::pair<Tuple, Tuple> ends1 = get_ends_of_edge_mesh(edge_mesh1);
    Tuple v_start1 = ends1.first;
    Tuple v_end1 = ends1.second;
    // map v_start1, v_end1 to v_start2, v_end2
    Tuple v_start2 = edge_mesh1.map(edge_mesh2, Simplex::vertex(v_start1));
    Tuple v_end2 = edge_mesh1.map(edge_mesh2, Simplex::vertex(v_end1));
    // if any of v_start of the two edge meshes is already initialized, then the other edge mesh
    // should also already been initialized
    if (t1_accessor.scalar_attribute(v_start1) >= 0) {
        if (t2_accessor.scalar_attribute(v_start2) < 0.) {
            throw std::runtime_error("The two edge meshes are not initialized consistently!");
        }
        return;
    }

    Tuple v1 = v_start1;
    Tuple v_next1 = edge_mesh1.switch_tuples(v1, {PrimitiveType::Vertex});
    Tuple v2 = v_start2;
    Tuple v_next2 = edge_mesh1.switch_tuples(v2, {PrimitiveType::Vertex});

    t1_accessor.scalar_attribute(v1) = 0.;
    t2_accessor.scalar_attribute(v2) = 0.;

    while (v_next1 != v_end1) {
        // map v and v_next to uv_mesh
        Tuple uv_v = edge_mesh1.map(uv_mesh, Simplex::vertex(v1));
        Tuple uv_v_next = edge_mesh1.map(uv_mesh, Simplex::vertex(v_next1));

        // compute the arclength between v and v_next
        double arclength = (uv_accessor.const_vector_attribute(uv_v) -
                            uv_accessor.const_vector_attribute(uv_v_next))
                               .stableNorm();
        // update t
        t1_accessor.scalar_attribute(v_next1) = t1_accessor.scalar_attribute(v1) + arclength;
        t2_accessor.scalar_attribute(v_next2) = t2_accessor.scalar_attribute(v2) + arclength;
        // update v and v_next
        v1 = v_next1;
        v_next1 = edge_mesh1.switch_tuples(v_next1, {PrimitiveType::Edge, PrimitiveType::Vertex});
        v2 = v_next2;
        v_next2 = edge_mesh2.switch_tuples(v_next2, {PrimitiveType::Edge, PrimitiveType::Vertex});
    }
    // if edge_mesh1 has reached the end, the edge_mesh2 should also have reached the end in the
    // while loop above
    assert(v_next2 == v_end2);
}
} // namespace wmtk::components::adaptive_tessellation::multimesh::utils