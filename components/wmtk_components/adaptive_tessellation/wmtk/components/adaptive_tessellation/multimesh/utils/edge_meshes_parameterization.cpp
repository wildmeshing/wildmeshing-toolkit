#include "edge_meshes_parameterization.hpp"
#include <wmtk/attribute/Accessor.hpp>

using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::attribute;
namespace wmtk::components::multimesh::utils {
Tuple map_to_parent_single_tuple(
    const Mesh& my_mesh,
    const Mesh& parent_mesh,
    const Tuple& tuple,
    const PrimitiveType& primitive_type)
{
    Simplex simplex = my_mesh.map_to_parent(Simplex(primitive_type, tuple));
    return simplex.tuple();
}

Tuple map_single_tuple(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const Tuple& tuple,
    const PrimitiveType& primitive_type)
{
    std::vector<Simplex> simplices = my_mesh.map(other_mesh, Simplex(primitive_type, tuple));
    assert(simplices.size() == 1);
    return simplices[0].tuple();
}

double arclength(
    const Mesh& my_mesh,
    const Mesh& parent_mesh,
    const Accessor<double>& uv_accessor,
    const Tuple& v_tuple,
    const Tuple& v_next_tuple)
{
    Tuple uv_v = map_to_parent_single_tuple(my_mesh, parent_mesh, v_tuple, PrimitiveType::Vertex);
    Tuple uv_v_next =
        map_to_parent_single_tuple(my_mesh, parent_mesh, v_next_tuple, PrimitiveType::Vertex);

    // compute the arclength between v and v_next
    return (uv_accessor.const_vector_attribute(uv_v) -
            uv_accessor.const_vector_attribute(uv_v_next))
        .stableNorm();
}

std::pair<Tuple, Tuple> get_ends_of_edge_mesh(const EdgeMesh& edge_mesh)
{
    // get the ends of the edge mesh
    std::vector<Tuple> ends;
    std::vector<Tuple> vertices = edge_mesh.get_all(PrimitiveType::Vertex);
    for (const Tuple& v : vertices) {
        if (edge_mesh.is_boundary(PrimitiveType::Vertex, v)) ends.push_back(v);
    }
    if (ends.size() != 2) throw std::runtime_error("The edge mesh is a closed curve!");
    return std::make_pair(ends[0], ends[1]);
}

void parameterize_edge_mesh(
    EdgeMesh& edge_mesh,
    const TriMesh& uv_mesh,
    wmtk::attribute::MeshAttributeHandle& t_handle,
    const wmtk::attribute::MeshAttributeHandle& uv_handle)
{
    // create an t accessor
    Accessor<double> t_accessor = edge_mesh.create_accessor(t_handle.as<double>());
    // create an uv accessor
    Accessor<double> uv_accessor = uv_mesh.create_const_accessor(uv_handle.as<double>());
    // get ends of the edge mesh
    std::pair<Tuple, Tuple> ends = get_ends_of_edge_mesh(edge_mesh);
    Tuple v_start = ends.first;
    Tuple v_end = ends.second;
    Tuple v = v_start;
    Tuple v_next = edge_mesh.switch_tuples(v, {PrimitiveType::Vertex});
    t_accessor.scalar_attribute(v) = 0.;

    while (v_next != v_end) {
        // compute the arclength between v and v_next
        double l = arclength(edge_mesh, uv_mesh, uv_accessor, v, v_next);
        // update t
        t_accessor.scalar_attribute(v_next) = t_accessor.scalar_attribute(v) + l;
        // update v and v_next
        v = v_next;
        v_next = edge_mesh.switch_tuples(v_next, {PrimitiveType::Edge, PrimitiveType::Vertex});
    }
    // if edge_mesh has reached the end initialize the end
    double l = arclength(edge_mesh, uv_mesh, uv_accessor, v, v_next);
    t_accessor.scalar_attribute(v_end) = t_accessor.scalar_attribute(v) + l;
}

void parameterize_seam_edge_meshes(
    EdgeMesh& edge_mesh1,
    EdgeMesh& edge_mesh2,
    const TriMesh& uv_mesh,
    wmtk::attribute::MeshAttributeHandle& t1_handle,
    wmtk::attribute::MeshAttributeHandle& t2_handle,
    wmtk::attribute::MeshAttributeHandle& uv_handle)
{
    // create an t accessor
    Accessor<double> t1_accessor = edge_mesh1.create_accessor(t1_handle.as<double>());
    Accessor<double> t2_accessor = edge_mesh2.create_accessor(t2_handle.as<double>());
    // create an uv accessor
    Accessor<double> uv_accessor = uv_mesh.create_const_accessor(uv_handle.as<double>());
    // get ends of the edge mesh
    std::pair<Tuple, Tuple> ends1 = get_ends_of_edge_mesh(edge_mesh1);
    Tuple v_start1 = ends1.first;
    Tuple v_end1 = ends1.second;
    // map v_start1, v_end1 to v_start2, v_end2
    Tuple v_start2 = map_single_tuple(edge_mesh1, edge_mesh2, v_start1, PrimitiveType::Vertex);
    Tuple v_end2 = map_single_tuple(edge_mesh1, edge_mesh2, v_end1, PrimitiveType::Vertex);
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
        // compute the arclength between v and v_next
        double l = arclength(edge_mesh1, uv_mesh, uv_accessor, v1, v_next1);
        // update t
        t1_accessor.scalar_attribute(v_next1) = t1_accessor.scalar_attribute(v1) + l;
        t2_accessor.scalar_attribute(v_next2) = t2_accessor.scalar_attribute(v2) + l;
        // update v and v_next
        v1 = v_next1;
        v_next1 = edge_mesh1.switch_tuples(v_next1, {PrimitiveType::Edge, PrimitiveType::Vertex});
        v2 = v_next2;
        v_next2 = edge_mesh2.switch_tuples(v_next2, {PrimitiveType::Edge, PrimitiveType::Vertex});
    }
    // if edge_mesh1 has reached the end, the edge_mesh2 should also have reached the end in the
    // while loop above
    assert(v_next2 == v_end2);
    double l = arclength(edge_mesh1, uv_mesh, uv_accessor, v1, v_next1);
    t1_accessor.scalar_attribute(v_end1) = t1_accessor.scalar_attribute(v1) + l;
    t2_accessor.scalar_attribute(v_end2) = t2_accessor.scalar_attribute(v2) + l;
}

void parameterize_all_edge_meshes(
    const TriMesh& uv_mesh,
    std::vector<std::shared_ptr<Mesh>>& edge_meshes,
    std::map<Mesh*, Mesh*>& sibling_edge_meshes)
{
    MeshAttributeHandle uv_coord_handle =
        uv_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    for (std::shared_ptr<Mesh> edge_mesh : edge_meshes) {
        if (edge_mesh->has_attribute<double>("t", PrimitiveType::Vertex)) {
            MeshAttributeHandle t_handle =
                edge_mesh->get_attribute_handle<double>("t", PrimitiveType::Vertex);
            Accessor<double> t_accessor =
                edge_mesh->create_const_accessor<double>(t_handle.as<double>());
            auto vertices = edge_mesh->get_all(PrimitiveType::Vertex);
            Tuple v_start = vertices[0]; // any vertex
            if (t_accessor.const_scalar_attribute(v_start) != -1) {
                continue;
            }
        }

        MeshAttributeHandle handle1 =
            edge_mesh->register_attribute<double>("t", PrimitiveType::Vertex, 1, true, -1);
        auto sibling_edge_mesh = sibling_edge_meshes[edge_mesh.get()];
        if (sibling_edge_mesh != nullptr) {
            MeshAttributeHandle handle2 =
                sibling_edge_mesh
                    ->register_attribute<double>("t", PrimitiveType::Vertex, 1, false, -1);

            parameterize_seam_edge_meshes(
                reinterpret_cast<EdgeMesh&>(*edge_mesh),
                reinterpret_cast<EdgeMesh&>(*sibling_edge_mesh),
                uv_mesh,
                handle1,
                handle2,
                uv_coord_handle);
        } else {
            parameterize_edge_mesh(
                reinterpret_cast<EdgeMesh&>(*edge_mesh),
                uv_mesh,
                handle1,
                uv_coord_handle);
        }
    }
}

} // namespace wmtk::components::multimesh::utils