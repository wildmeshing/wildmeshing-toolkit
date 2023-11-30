#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <random>
#include <stack>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk_components/extract_subset/extract_subset.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

bool is_valid_mesh(const wmtk::TriMesh& tm)
{
    return true;
}

bool is_connected(
    const wmtk::TriMesh& tm,
    const std::set<long>& index_set,
    std::map<long, std::vector<long>>& connections)
{
    std::set<long> visited_vertices;
    std::stack<long> stack;
    stack.push(connections.begin()->first);
    while (!stack.empty()) {
        long current_vertex = stack.top();
        stack.pop();
        if (visited_vertices.count(current_vertex) == 0) {
            visited_vertices.insert(current_vertex);
            for (long neighbor : connections[current_vertex]) {
                stack.push(neighbor);
            }
        }
    }
    return visited_vertices.size() == index_set.size();
}

std::map<long, std::vector<long>> get_connection(const wmtk::TriMesh& tm, std::set<long>& index_set)
{
    auto edges = tm.get_all(wmtk::PrimitiveType::Edge);
    std::map<long, std::vector<long>> connections;
    for (auto edgeindex : index_set) {
        auto edgeTuple = edges[edgeindex];
        auto edgeVertexList = wmtk::simplex::faces_single_dimension(
            tm,
            wmtk::Simplex::edge(edgeTuple),
            wmtk::PrimitiveType::Vertex);
        long v1 = wmtk::components::internal::find_vertex_index(tm, edgeVertexList[0]);
        long v2 = wmtk::components::internal::find_vertex_index(tm, edgeVertexList[1]);
        if (!connections.count(v1)) {
            std::vector<long> nodes;
            nodes.push_back(v2);
            connections[v1] = nodes;
        } else
            connections[v1].push_back(v2);
        if (!connections.count(v2)) {
            std::vector<long> nodes;
            nodes.push_back(v1);
            connections[v2] = nodes;
        } else
            connections[v2].push_back(v1);
    }
    return connections;
}

// Reference: https://www.geeksforgeeks.org/determining-topology-formed-in-a-graph/
bool is_circle(const wmtk::TriMesh& tm, std::set<long> index_set)
{
    std::map<long, std::vector<long>> connections = get_connection(tm, index_set);
    if (index_set.size() != connections.size()) return false;
    bool isRing = all_of(connections.begin(), connections.end(), [](auto& nodes) {
        return nodes.second.size() == 2;
    });
    bool connected = is_connected(tm, index_set, connections);
    return isRing && connected;
}

// Reference: https://www.geeksforgeeks.org/determining-topology-formed-in-a-graph/
bool is_line(const wmtk::TriMesh& tm, std::set<long> index_set)
{
    if (index_set.size() == 1) return true;
    std::map<long, std::vector<long>> connections = get_connection(tm, index_set);
    if (index_set.size() != connections.size() - 1) return false;
    long deg1 = 0, deg2 = 0;
    for (auto& nodes : connections) {
        if (nodes.second.size() == 1)
            deg1++;
        else if (nodes.second.size() == 2)
            deg2++;
        else
            return false;
    }
    return deg1 == 2 && deg2 == connections.size() - 2;
}

bool is_manifold(const wmtk::TriMesh& tm)
{
    std::map<long, std::set<long>> vertexLinkEdges;
    auto faces = tm.get_all(wmtk::PrimitiveType::Face);
    auto vertices = tm.get_all(wmtk::PrimitiveType::Vertex);
    for (long vid = 0; vid < tm.capacity(wmtk::PrimitiveType::Vertex); ++vid) {
        std::vector<long> adj_faces = wmtk::components::internal::adj_faces_of_vertex(tm, vid);
        for (long fid : adj_faces) {
            auto faceTuple = faces[fid];
            auto edgeList = wmtk::simplex::faces_single_dimension(
                tm,
                wmtk::Simplex::face(faceTuple),
                wmtk::PrimitiveType::Edge);
            for (auto edgeTuple : edgeList) {
                auto edgeVertexList = wmtk::simplex::faces_single_dimension(
                    tm,
                    wmtk::Simplex::edge(edgeTuple),
                    wmtk::PrimitiveType::Vertex);
                if (!tm.simplices_are_equal(
                        wmtk::Simplex::vertex(edgeVertexList[0]),
                        wmtk::Simplex::vertex(vertices[vid])) &&
                    !tm.simplices_are_equal(
                        wmtk::Simplex::vertex(edgeVertexList[1]),
                        wmtk::Simplex::vertex(vertices[vid]))) {
                    vertexLinkEdges[vid].insert(
                        wmtk::components::internal::find_edge_index(tm, edgeTuple));
                }
            }
        }
    }

    std::vector<bool> edge_count(tm.capacity(wmtk::PrimitiveType::Edge), false);
    wmtk::components::internal::get_edge_count(tm, edge_count);

    for (auto& [vid, edgeSet] : vertexLinkEdges) {
        // for vertices on the boundary, the link needs to be a 1-ball, which is a line
        if (wmtk::components::internal::vertex_on_boundary(tm, edge_count, vid)) {
            // std::cout << "Vertex " << vid << " is on the boundary." << std::endl;
            // std::all_of(edgeSet.begin(), edgeSet.end(), [](long e) {
            //     std::cout << e << " ";
            //     return true;
            // });
            // std::cout << std::endl;
            if (!is_line(tm, edgeSet)) {
                // std::cout << "Vertex " << vid << " doesn't have a line link." << std::endl;
                return false;
            }
        }
        // for vertices inside the mesh, the link needs to be a 1-sphere, which is a circle
        else {
            // std::cout << "Vertex " << vid << " is not on the boundary." << std::endl;
            // std::all_of(edgeSet.begin(), edgeSet.end(), [](long e) {
            //     std::cout << e << " ";
            //     return true;
            // });
            // std::cout << std::endl;
            if (!is_circle(tm, edgeSet)) {
                // std::cout << "Vertex " << vid << " doesn't have a circle link." << std::endl;
                return false;
            }
        }
    }
    return true;
}

void check_new_mesh(
    wmtk::tests::DEBUG_TriMesh& m,
    std::vector<int> data,
    bool b,
    int vertex_count,
    int edge_count,
    int face_count)
{
    wmtk::tests::DEBUG_TriMesh new_tm = wmtk::components::extract_subset(m, 2, data, b);
    // new_tm.print_vf();
    // CHECK(is_valid_mesh(new_tm));
    // CHECK(is_manifold(new_tm));
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Vertex) == vertex_count);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Edge) == edge_count);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Face) == face_count);
    // wmtk::ParaviewWriter writer("mesh_smooth", "vertices", new_tm, true, true, true, false);
    // new_tm.serialize(writer);
}
/*
TEST_CASE("2d_tetrahedron_test_case", "[components][extract_subset][2D]")
{
    wmtk::tests::DEBUG_TriMesh tm = wmtk::tests::tetrahedron_with_position();
    for (int i1 = 0; i1 < 2; ++i1) {
        for (int i2 = 0; i2 < 2; ++i2) {
            for (int i3 = 0; i3 < 2; ++i3) {
                for (int i4 = 0; i4 < 2; ++i4) {
                    std::vector<int> tag_vector = {i1, i2, i3, i4};
                    // std::cout << i1 + i2 + i3 + i4 << std::endl;
                    switch (i1 + i2 + i3 + i4) {
                    // TODO: what to return if none of the faces are tagged? NULL?
                    // Maybe construct a trimesh with 0 vertices
                    case 1: check_new_mesh(tm, tag_vector, true, 3, 3, 1); break;
                    case 2: check_new_mesh(tm, tag_vector, true, 4, 5, 2); break;
                    case 3: check_new_mesh(tm, tag_vector, true, 4, 6, 3); break;
                    case 4: check_new_mesh(tm, tag_vector, true, 4, 6, 4); break;
                    }
                }
            }
        }
    }
}
*/
TEST_CASE("2d_9tri_with_a_hole_test_case", "[components][extract_subset][2D]")
{
    wmtk::tests::DEBUG_TriMesh tm = wmtk::tests::nine_triangles_with_a_hole();
    const unsigned long test_size = 1400; // total cases: 2^9 - 1 = 511, n logn = 1384
    std::vector<int> tag_vector(tm.capacity(wmtk::PrimitiveType::Face), 0);
    for (size_t i = 0; i < test_size; ++i) {
        std::mt19937 mt{i};
        std::uniform_int_distribution tag{0, 1};
        for (int j = 0; j < tag_vector.size(); ++j) {
            tag_vector[j] = tag(mt);
        }
        if (std::reduce(tag_vector.begin(), tag_vector.end()) == 0) {
            std::fill(tag_vector.begin(), tag_vector.end(), 0);
            continue;
        }
        // std::all_of(tag_vector.begin(), tag_vector.end(), [](int i) {
        //     std::cout << i << " ";
        //     return true;
        // });
        // check_new_mesh(tm, tag_vector, false, 0, 0, 0);
        wmtk::tests::DEBUG_TriMesh new_tm =
            wmtk::components::extract_subset(tm, 2, tag_vector, false);
        // std::cout << "\tBefore: manifold = " << is_manifold(new_tm);
        auto topo_tm = wmtk::components::internal::topology_separate_2d(new_tm);
        bool after = is_manifold(topo_tm);
        // std::cout << "; After: manifold = " << after << std::endl;
        CHECK(after);
        std::fill(tag_vector.begin(), tag_vector.end(), 0);
    }
}

TEST_CASE("component_3+4_test_case", "[components][extract_subset][2D][manual]")
{
    wmtk::tests::DEBUG_TriMesh tm;
    wmtk::RowVectors3l tris;
    tris.resize(46, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 0, 2, 3;
    tris.row(2) << 1, 2, 4;
    tris.row(3) << 2, 3, 4;
    tris.row(4) << 0, 3, 5;
    tris.row(5) << 3, 4, 5;
    tris.row(6) << 0, 5, 6;
    tris.row(7) << 5, 6, 7;
    tris.row(8) << 4, 5, 8;
    tris.row(9) << 0, 6, 7;
    tris.row(10) << 5, 7, 8;
    tris.row(11) << 0, 7, 9;
    tris.row(12) << 7, 8, 9;
    tris.row(13) << 0, 9, 10;
    tris.row(14) << 9, 10, 12;
    tris.row(15) << 9, 12, 11;
    tris.row(16) << 9, 8, 11;
    tris.row(17) << 0, 10, 13;
    tris.row(18) << 10, 12, 13;
    tris.row(19) << 13, 12, 15;
    tris.row(20) << 12, 15, 16;
    tris.row(21) << 11, 12, 16;
    tris.row(22) << 11, 16, 17;
    tris.row(23) << 15, 16, 17;
    tris.row(24) << 13, 15, 17;
    tris.row(25) << 0, 13, 14;
    tris.row(26) << 13, 14, 17;
    tris.row(27) << 0, 14, 20;
    tris.row(28) << 14, 18, 20;
    tris.row(29) << 14, 18, 17;
    tris.row(30) << 17, 18, 19;
    tris.row(31) << 0, 20, 24;
    tris.row(32) << 20, 18, 24;
    tris.row(33) << 18, 19, 24;
    tris.row(34) << 19, 21, 24;
    tris.row(35) << 0, 24, 23;
    tris.row(36) << 24, 21, 23;
    tris.row(37) << 21, 22, 23;
    tris.row(38) << 0, 23, 27;
    tris.row(39) << 27, 23, 26;
    tris.row(40) << 23, 22, 26;
    tris.row(41) << 0, 27, 29;
    tris.row(42) << 29, 27, 28;
    tris.row(43) << 28, 27, 26;
    tris.row(44) << 0, 25, 29;
    tris.row(45) << 25, 28, 29;
    tm.initialize(tris);

    std::vector<int> tag_vector(tm.capacity(wmtk::PrimitiveType::Face), 0);
    std::vector<int> id = {0,  1,  2,  3,  5,  6,  7,  8,  10, 11, 12, 25, 26, 29,
                           30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 42, 43, 44, 45};
    for (auto i : id) tag_vector[i] = 1;
    wmtk::tests::DEBUG_TriMesh new_tm = wmtk::components::extract_subset(tm, 2, tag_vector, false);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Vertex) == 25);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Face) == 28);
    // new_tm.print_vf();
    auto topo_tm = wmtk::components::internal::topology_separate_2d(new_tm);
    CHECK(is_valid_mesh(topo_tm));
    CHECK(is_manifold(topo_tm));
    CHECK(topo_tm.capacity(wmtk::PrimitiveType::Vertex) == 31);
    CHECK(topo_tm.capacity(wmtk::PrimitiveType::Face) == 28);
}


TEST_CASE("random_test_from_manext_branch", "[components][extract_subset][2D][random]")
{
    unsigned int nb_points = 50; // 20
    unsigned int nb_triangles;
    unsigned int nb_vertices;
    double range = 10.0;
    const size_t tagass_loop = 100; // 100
    const size_t pntgen_loop = 6; // 10
    const double prob = 0.2;

    // test for 10 iterations, each with 10 more vertices, so 10~100
    for (size_t i = 0; i < pntgen_loop; ++i) {
        wmtk::TriMesh tm;
        wmtk::RowVectors3l tris;

        wmtk::RowVectors2d points(nb_points, 2);
        std::random_device rd{};
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0, range);
        for (size_t j = 0; j < nb_points; ++j) {
            // generate 2 random doubles between 0 and the given range
            points.row(j) << dis(gen), dis(gen);
        }

        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        std::tie(vertices, faces) = wmtk::components::internal::delaunay_2d(points);
        nb_vertices = vertices.rows();
        nb_triangles = faces.rows();
        std::cout << "\nMan-ext 2D test: total tri num=" << nb_triangles << "\n";
        // std::cout<< faces << std::endl;

        // start using Trimesh data structure
        tris.resize(nb_triangles, 3);
        for (unsigned int j = 0; j < nb_triangles; ++j) {
            tris.row(j) << faces(j, 0), faces(j, 1), faces(j, 2);
        }
        tm.initialize(tris);
        wmtk::mesh_utils::set_matrix_attribute(vertices, "position", wmtk::PrimitiveType::Vertex, tm);

        // assign 100 sets of different tags for all triangles
        for (size_t j = 0; j < tagass_loop; ++j) {
            std::mt19937 mt{j};
            std::uniform_int_distribution tagger{0, 1};
            std::vector<int> tag_vector(nb_triangles, 0);
            for (int k = 0; k < tag_vector.size(); ++k) {
                tag_vector[k] = tagger(mt);
            }
            if (std::reduce(tag_vector.begin(), tag_vector.end()) == 0) {
                std::fill(tag_vector.begin(), tag_vector.end(), 0);
                continue;
            }
            // std::cout << "Tag: ";
            // std::all_of(tag_vector.begin(), tag_vector.end(), [](int i) {
            //     std::cout << i;
            //     return true;
            // });
            wmtk::tests::DEBUG_TriMesh new_tm =
                wmtk::components::extract_subset(tm, 2, tag_vector, false);
            std::cout << "\tBefore: manifold = " << is_manifold(new_tm);
            auto topo_tm = wmtk::components::internal::topology_separate_2d(new_tm);
            bool after = is_manifold(topo_tm);
            std::cout << "; After: manifold = " << after << std::endl;
            CHECK(after);
            std::fill(tag_vector.begin(), tag_vector.end(), 0);
        }
        nb_points += 10;
        range += 10.0;
    }
}
