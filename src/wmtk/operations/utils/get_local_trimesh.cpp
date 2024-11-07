// #include "get_local_trimesh.hpp"
#include <unordered_map>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/mesh_utils.hpp>
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
    global_to_local_map[mesh.id(simplex::Simplex::vertex(mesh, simplex.tuple()))] = 0;
    int face_count = 0;
    for (const auto& f_tuple : cofaces) {
        // get 3 vertices
        Tuple cur_v = f_tuple.tuple();
        if (mesh.is_ccw(cur_v)) {
            cur_v = mesh.switch_edge(cur_v);
        }

        for (int i = 0; i < 3; i++) {
            int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(mesh, cur_v));
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
    assert(simplex.primitive_type() == PrimitiveType::Edge);
    auto v0 = simplex::Simplex::vertex(mesh, simplex.tuple());
    auto v1 = simplex::Simplex::vertex(mesh, mesh.switch_vertex(simplex.tuple()));


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

    // the one which kept after collapse -- v1
    global_to_local_map[mesh.id(v1)] = 0;

    int face_count = 0;
    for (const auto& f_tuple : cofaces) {
        // get 3 vertices
        Tuple cur_v = f_tuple.tuple();
        if (mesh.is_ccw(cur_v)) {
            cur_v = mesh.switch_edge(cur_v);
        }
        for (int i = 0; i < 3; i++) {
            int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(mesh, cur_v));
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

std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t>, std::vector<int64_t>>
get_local_tetmesh(
    const wmtk::TetMesh& mesh,
    const wmtk::simplex::Simplex& simplex,
    bool get_boundary)
{
    // Get the vertex position attribute handle
    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos = mesh.create_const_accessor<double>(pos_handle);

    // Create a map to store global-to-local vertex index mapping
    std::unordered_map<int64_t, int> global_to_local_map;
    // Get all tetrahedrons related to the given simplex
    const auto cofaces = wmtk::simplex::top_dimension_cofaces(mesh, simplex)
                             .simplex_vector(PrimitiveType::Tetrahedron);

    // Initialize local tetrahedron face list and local-to-global mapping for tetrahedrons
    Eigen::MatrixXi T(cofaces.size(), 4);
    std::vector<int64_t> t_local_to_global(cofaces.size());

    int vertex_count = 1;
    global_to_local_map[mesh.id(simplex::Simplex::vertex(mesh, simplex.tuple()))] = 0;
    int tet_count = 0;
    for (const auto& t_tuple : cofaces) {
        // Get the 4 vertices of the tetrahedron
        Tuple cur_v = t_tuple.tuple();
        if (mesh.is_ccw(cur_v)) {
            cur_v = mesh.switch_edge(cur_v);
        }

        for (int i = 0; i < 4; i++) {
            int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(mesh, cur_v));
            // If vertex not in the map, add it
            if (global_to_local_map.count(global_vid) == 0) {
                global_to_local_map[global_vid] = vertex_count;
                vertex_count++;
            }

            T(tet_count, i) = global_to_local_map[global_vid];
            if (i == 2) {
                cur_v = mesh.switch_tuples(
                    cur_v,
                    {PrimitiveType::Triangle,
                     PrimitiveType::Edge,
                     PrimitiveType::Vertex}); // Next vertex
            } else {
                cur_v = mesh.switch_tuples(
                    cur_v,
                    {PrimitiveType::Edge, PrimitiveType::Vertex}); // Next vertex
            }
        }

        t_local_to_global[tet_count] = mesh.id(t_tuple);
        tet_count++;
    }
    Eigen::MatrixXd V(vertex_count, pos.dimension());
    std::vector<int64_t> v_local_to_global(vertex_count);
    for (const auto& pair : global_to_local_map) {
        v_local_to_global[pair.second] = pair.first;
        V.row(pair.second) =
            pos.const_vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, pair.first));
    }

    if (get_boundary) {
        const auto triangle_coface =
            wmtk::simplex::cofaces_single_dimension(mesh, simplex, PrimitiveType::Triangle)
                .simplex_vector(PrimitiveType::Triangle);

        int face_count = 0;
        Eigen::MatrixXi F_bd(0, 3);
        for (auto& t_tuples : triangle_coface) {
            if (!mesh.is_boundary(t_tuples)) {
                continue;
            }
            Tuple cur_v = t_tuples.tuple();
            if (mesh.is_ccw(cur_v)) {
                cur_v = mesh.switch_edge(cur_v);
            }
            Eigen::MatrixXi F_temp(1, 3);
            for (int i = 0; i < 3; i++) {
                int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(mesh, cur_v));
                F_temp(0, i) = global_to_local_map[global_vid];
                cur_v = mesh.switch_tuples(
                    cur_v,
                    {PrimitiveType::Edge, PrimitiveType::Vertex}); // Next vertex
            }
            F_bd.conservativeResize(F_bd.rows() + 1, 3);
            F_bd.row(face_count) = F_temp;
            face_count++;
        }
    }
    return std::make_tuple(T, V, t_local_to_global, v_local_to_global);
}

std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t>, std::vector<int64_t>>
get_local_tetmesh_before_collapse(const wmtk::TetMesh& mesh, const wmtk::simplex::Simplex& simplex)
{
    assert(simplex.primitive_type() == PrimitiveType::Edge);

    auto v0 = simplex::Simplex::vertex(mesh, simplex.tuple());
    auto v1 = simplex::Simplex::vertex(mesh, mesh.switch_vertex(simplex.tuple()));

    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos = mesh.create_const_accessor<double>(pos_handle);

    std::unordered_map<int64_t, int> global_to_local_map;
    const auto cofaces0 = wmtk::simplex::top_dimension_cofaces(mesh, v0);
    const auto cofaces1 = wmtk::simplex::top_dimension_cofaces(mesh, v1);
    auto cofaces = simplex::SimplexCollection::get_union(cofaces0, cofaces1)
                       .simplex_vector(PrimitiveType::Tetrahedron);

    Eigen::MatrixXi T(cofaces.size(), 4);
    std::vector<int64_t> t_local_to_global(cofaces.size());
    int vertex_count = 1;

    // the one which kept after collapse -- v1
    global_to_local_map[mesh.id(v1)] = 0;

    int tet_count = 0;
    for (const auto& t_tuple : cofaces) {
        // get 4 vertices
        Tuple cur_v = t_tuple.tuple();

        if (mesh.is_ccw(cur_v)) {
            cur_v = mesh.switch_edge(cur_v);
        }
        for (int i = 0; i < 4; i++) {
            int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(mesh, cur_v));
            if (global_to_local_map.count(global_vid) == 0) {
                global_to_local_map[global_vid] = vertex_count;
                vertex_count++;
            }
            T(tet_count, i) = global_to_local_map[global_vid];
            if (i == 2) {
                cur_v = mesh.switch_tuples(
                    cur_v,
                    {PrimitiveType::Triangle,
                     PrimitiveType::Edge,
                     PrimitiveType::Vertex}); // next vertex
            } else {
                cur_v = mesh.switch_tuples(
                    cur_v,
                    {PrimitiveType::Edge, PrimitiveType::Vertex}); // next vertex
            }
        }

        t_local_to_global[tet_count] = mesh.id(t_tuple);
        tet_count++;
    }

    Eigen::MatrixXd V(vertex_count, pos.dimension());
    std::vector<int64_t> v_local_to_global(vertex_count);
    // build V
    for (const auto& pair : global_to_local_map) {
        v_local_to_global[pair.second] = pair.first;
        V.row(pair.second) =
            pos.const_vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, pair.first));
    }

    // TODO: get local triangle mesh if it is on the boudnary

    if (mesh.is_boundary(simplex)) {
        const auto triangle_coface0 =
            wmtk::simplex::cofaces_single_dimension(mesh, v0, PrimitiveType::Triangle);
        const auto triangle_coface1 =
            wmtk::simplex::cofaces_single_dimension(mesh, v1, PrimitiveType::Triangle);
        auto triangle_cofaces =
            simplex::SimplexCollection::get_union(triangle_coface0, triangle_coface1)
                .simplex_vector(PrimitiveType::Triangle);

        int face_count = 0;
        Eigen::MatrixXi F_bd(0, 3);

        for (const auto& f_tuple : triangle_cofaces) {
            if (!mesh.is_boundary(f_tuple)) {
                continue;
            }
            Tuple cur_v = f_tuple.tuple();
            if (mesh.is_ccw(cur_v)) {
                cur_v = mesh.switch_edge(cur_v);
            }
            Eigen::MatrixXi F_temp(1, 3);
            for (int i = 0; i < 3; i++) {
                int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(mesh, cur_v));
                F_temp(0, i) = global_to_local_map[global_vid];
                cur_v = mesh.switch_tuples(
                    cur_v,
                    {PrimitiveType::Edge, PrimitiveType::Vertex}); // next vertex
            }
            F_bd.conservativeResize(F_bd.rows() + 1, 3);
            F_bd.row(face_count) = F_temp;
            face_count++;
        }
    }

    return std::make_tuple(T, V, t_local_to_global, v_local_to_global);
}

} // namespace wmtk::operations::utils