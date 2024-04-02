// #include "get_local_trimesh.hpp"
#include <unordered_map>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>

// TODO: we also need fid_maps
namespace wmtk::operations::utils {
std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t>, std::vector<int64_t>>
get_local_trimesh(const wmtk::TriMesh& mesh, const wmtk::simplex::Simplex& simplex)
{
    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos = mesh.create_const_accessor<double>(pos_handle);

    std::unordered_map<int64_t, int> global_to_local_map;
    const auto cofaces =
        wmtk::simplex::top_dimension_cofaces(mesh, simplex).simplex_vector(PrimitiveType::Triangle);


    Eigen::MatrixXi F(cofaces.size(), 3);
    std::vector<int64_t> f_local_to_global(cofaces.size());

    int vertex_count = 1;
    global_to_local_map[mesh.id(simplex::Simplex::vertex(simplex.tuple()))] = 0;
    int face_count = 0;
    for (const auto& f_tuple : cofaces) {
        // get 3 vertices
        Tuple cur_v = f_tuple.tuple();
        if (mesh.is_ccw(cur_v)) {
            cur_v = mesh.switch_edge(cur_v);
        }

        for (int i = 0; i < 3; i++) {
            int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(cur_v));
            if (global_to_local_map.count(global_vid) == 0) {
                global_to_local_map[global_vid] = vertex_count;
                vertex_count++;
            }
            F(face_count, i) = global_to_local_map[global_vid];
            cur_v = mesh.switch_tuples(
                cur_v,
                {PrimitiveType::Edge, PrimitiveType::Vertex}); // next vertex
        }
        f_local_to_global[face_count] = mesh.id(f_tuple);
        face_count++;
    }

    Eigen::MatrixXd V(vertex_count, pos.dimension());
    std::vector<int64_t> v_local_to_global(vertex_count);
    // build V, local_to_global
    for (const auto& pair : global_to_local_map) {
        v_local_to_global[pair.second] = pair.first;
        V.row(pair.second) =
            pos.const_vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, pair.first));
    }

    return std::make_tuple(F, V, f_local_to_global, v_local_to_global);
}

std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t>, std::vector<int64_t>>
get_local_trimesh_before_collapse(const wmtk::TriMesh& mesh, const wmtk::simplex::Simplex& simplex)
{
    assert(simplex.type() == PrimitiveType::Edge);
    auto v0 = simplex::Simplex::vertex(simplex.tuple());
    auto v1 = simplex::Simplex::vertex(mesh.switch_vertex(simplex.tuple()));


    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos = mesh.create_const_accessor<double>(pos_handle);

    std::unordered_map<int64_t, int> global_to_local_map;
    const auto cofaces0 = wmtk::simplex::top_dimension_cofaces(mesh, v0);
    const auto cofaces1 = wmtk::simplex::top_dimension_cofaces(mesh, v1);
    auto cofaces = simplex::SimplexCollection::get_union(cofaces0, cofaces1)
                       .simplex_vector(PrimitiveType::Triangle);

    Eigen::MatrixXi F(cofaces.size(), 3);
    std::vector<int64_t> f_local_to_global(cofaces.size());
    int vertex_count = 1;
    global_to_local_map[mesh.id(simplex::Simplex::vertex(mesh.switch_vertex(simplex.tuple())))] = 0;
    int face_count = 0;
    for (const auto& f_tuple : cofaces) {
        // get 3 vertices
        Tuple cur_v = f_tuple.tuple();
        if (mesh.is_ccw(cur_v)) {
            cur_v = mesh.switch_edge(cur_v);
        }
        for (int i = 0; i < 3; i++) {
            int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(cur_v));
            if (global_to_local_map.count(global_vid) == 0) {
                global_to_local_map[global_vid] = vertex_count;
                vertex_count++;
            }
            F(face_count, i) = global_to_local_map[global_vid];
            cur_v = mesh.switch_tuples(
                cur_v,
                {PrimitiveType::Edge, PrimitiveType::Vertex}); // next vertex
        }
        f_local_to_global[face_count] = mesh.id(f_tuple);
        face_count++;
    }

    Eigen::MatrixXd V(vertex_count, pos.dimension());
    std::vector<int64_t> v_local_to_global(vertex_count);
    // build V
    for (const auto& pair : global_to_local_map) {
        v_local_to_global[pair.second] = pair.first;
        V.row(pair.second) =
            pos.const_vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, pair.first));
    }

    return std::make_tuple(F, V, f_local_to_global, v_local_to_global);
}
} // namespace wmtk::operations::utils