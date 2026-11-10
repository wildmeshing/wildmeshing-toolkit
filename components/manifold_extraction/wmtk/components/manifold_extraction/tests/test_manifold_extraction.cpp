#include <wmtk/TetMesh.h>
#include <wmtk/components/manifold_extraction/ManExtractTetMesh.h>
#include <wmtk/components/topological_offset/Parameters.h>
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <wmtk/components/manifold_extraction/manifold_extraction.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#include <wmtk/utils/Delaunay.hpp>


using namespace wmtk;
using namespace wmtk::delaunay;
using namespace components::manifold_extraction;
using namespace components;


// used for checking attribute propagatoin. values are arbitrary
const int V0_LABEL = 10;
const int V1_LABEL = 11;
const int V2_LABEL = 12;
const int V3_LABEL = 13;
const int E0_LABEL = 20;
const int E1_LABEL = 21;
const int E2_LABEL = 22;
const int E3_LABEL = 23;
const int E4_LABEL = 24;
const int E5_LABEL = 25;
const int F0_LABEL = 30;
const int F1_LABEL = 31;
const int F2_LABEL = 32;
const int F3_LABEL = 33;
const int T0_LABEL = 40;
const std::set<std::string> T0_TAGS = {{"c"}};


/**
 * NOTE: these split tests are redundant, they just call the TopoOffsetTetMesh split functions which
 * have their own unit tests. This should contain manifoldness checks for 2d and 3d instead
 */


TEST_CASE("edge_split", "[split_op]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tag(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    Tag.coeffRef(0, 0) = 0;
    Tag.coeffRef(0, 1) = 0;
    Tag.coeffRef(0, 2) = 1;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    topological_offset::Parameters o_param = param.generate_offset_params();
    ManExtractTetMesh mesh(param, o_param, 0);
    mesh.init_from_image(V, T, Tag, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_vertex_extra[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;
    mesh.m_face_extra[1].label = F1_LABEL;
    mesh.m_face_extra[2].label = F2_LABEL;
    mesh.m_face_extra[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split edge
    TetMesh::Tuple e = mesh.tuple_from_edge({{1, 2}});
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_edge(e, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_extra[4].label == E1_LABEL);

    // edges
    std::array<std::array<size_t, 3>, 9> edges = {
        {// {v0id, v1id, correct label}
         {{0, 1, E0_LABEL}},
         {{0, 2, E2_LABEL}},
         {{0, 3, E3_LABEL}},
         {{0, 4, F0_LABEL}},
         {{1, 3, E4_LABEL}},
         {{1, 4, E1_LABEL}},
         {{2, 3, E5_LABEL}},
         {{2, 4, E1_LABEL}},
         {{3, 4, F3_LABEL}}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({{v0, v1}}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 7> faces = {
        {{{0, 1, 3, F2_LABEL}},
         {{0, 1, 4, F0_LABEL}},
         {{0, 2, 3, F1_LABEL}},
         {{0, 2, 4, F0_LABEL}},
         {{0, 3, 4, T0_LABEL}},
         {{1, 3, 4, F3_LABEL}},
         {{2, 3, 4, F3_LABEL}}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_extra[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 2; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_tet_attribute[i].tag) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == T0_TAGS);
    }
}


TEST_CASE("face_split", "[split_op]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tag(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    Tag.coeffRef(0, 0) = 0;
    Tag.coeffRef(0, 1) = 0;
    Tag.coeffRef(0, 2) = 1;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    topological_offset::Parameters o_param = param.generate_offset_params();
    ManExtractTetMesh mesh(param, o_param, 0);
    mesh.init_from_image(V, T, Tag, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_vertex_extra[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;
    mesh.m_face_extra[1].label = F1_LABEL;
    mesh.m_face_extra[2].label = F2_LABEL;
    mesh.m_face_extra[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split face
    auto [ftup, _] = mesh.tuple_from_face({{1, 2, 3}});
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_face(ftup, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_extra[4].label == F3_LABEL);

    // edges
    std::array<std::array<size_t, 3>, 10> edges = {
        {// {v0id, v1id, correct label}
         {{0, 1, E0_LABEL}},
         {{0, 2, E2_LABEL}},
         {{0, 3, E3_LABEL}},
         {{0, 4, T0_LABEL}},
         {{1, 2, E1_LABEL}},
         {{1, 3, E4_LABEL}},
         {{1, 4, F3_LABEL}},
         {{2, 3, E5_LABEL}},
         {{2, 4, F3_LABEL}},
         {{3, 4, F3_LABEL}}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({{v0, v1}}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 9> faces = {
        {// {v0id, v1id, v2id, correct label}
         {{0, 1, 2, F0_LABEL}},
         {{0, 1, 3, F2_LABEL}},
         {{0, 1, 4, T0_LABEL}},
         {{0, 2, 3, F1_LABEL}},
         {{0, 2, 4, T0_LABEL}},
         {{0, 3, 4, T0_LABEL}},
         {{1, 2, 4, F3_LABEL}},
         {{1, 3, 4, F3_LABEL}},
         {{2, 3, 4, F3_LABEL}}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_extra[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 3; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_tet_attribute[i].tag) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == T0_TAGS);
    }
}


TEST_CASE("tet_split", "[split_op]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tag(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    Tag.coeffRef(0, 0) = 0;
    Tag.coeffRef(0, 1) = 0;
    Tag.coeffRef(0, 2) = 1;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    topological_offset::Parameters o_param = param.generate_offset_params();
    ManExtractTetMesh mesh(param, o_param, 0);
    mesh.init_from_image(V, T, Tag, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_vertex_extra[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;
    mesh.m_face_extra[1].label = F1_LABEL;
    mesh.m_face_extra[2].label = F2_LABEL;
    mesh.m_face_extra[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split face
    TetMesh::Tuple ttup = mesh.tuple_from_tet(0);
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_tet(ttup, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_extra[4].label == T0_LABEL);

    // edges
    std::array<std::array<size_t, 3>, 10> edges = {
        {// {v0id, v1id, correct label}
         {{0, 1, E0_LABEL}},
         {{0, 2, E2_LABEL}},
         {{0, 3, E3_LABEL}},
         {{0, 4, T0_LABEL}},
         {{1, 2, E1_LABEL}},
         {{1, 3, E4_LABEL}},
         {{1, 4, T0_LABEL}},
         {{2, 3, E5_LABEL}},
         {{2, 4, T0_LABEL}},
         {{3, 4, T0_LABEL}}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({{v0, v1}}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 10> faces = {
        {// {v0id, v1id, v2id, correct label}
         {{0, 1, 2, F0_LABEL}},
         {{0, 1, 3, F2_LABEL}},
         {{0, 1, 4, T0_LABEL}},
         {{0, 2, 3, F1_LABEL}},
         {{0, 2, 4, T0_LABEL}},
         {{0, 3, 4, T0_LABEL}},
         {{1, 2, 3, F3_LABEL}},
         {{1, 2, 4, T0_LABEL}},
         {{1, 3, 4, T0_LABEL}},
         {{2, 3, 4, T0_LABEL}}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_extra[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 4; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_tet_attribute[i].tag) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == T0_TAGS);
    }
}


namespace {
/// `rows` as the `type` block of a tagged mesh, every row under the one label `name`, as
/// tetwild, triwild and simwild write their tagged faces, curves and points.
void add_tagged_block(
    MeshData& m,
    const CellType type,
    const std::vector<std::vector<int64_t>>& rows,
    const std::string& name)
{
    auto& block = m.cells[type];
    block.connectivity.resize(Eigen::Index(rows.size()), cell_size(type));
    for (size_t r = 0; r < rows.size(); ++r) {
        for (size_t k = 0; k < rows[r].size(); ++k)
            block.connectivity(Eigen::Index(r), Eigen::Index(k)) = rows[r][k];
    }
    const DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Ones(Eigen::Index(rows.size()), 1);
    block.attributes["tags"] = {member, {name}};
}

/// [-1, 1]^3 in 4^3 cubes of edge 0.5, each split into 6 tets around its diagonal, with the
/// regions `a`, cube (1, 1, 1), and `b`, cube (2, 2, 1), which touch along the grid edge
/// x = y = 0, -0.5 <= z <= 0: the boundary of a | b is not manifold there. Beside them the blocks
/// tetwild and simwild write: the tagged faces `sheet`, the square z = 0.5, -1 <= x, y <= -0.5;
/// the feature curve `curve`, the two grid edges y = z = 0, -0.5 <= x <= 0.5, which meet at the
/// top end of that edge, so that the carving splits them; the input point `p`, (0.5, -0.5, 0.5).
MeshData touching_cubes()
{
    const auto vid = [](int i, int j, int k) { return int64_t(i + 5 * (j + 5 * k)); };
    MatrixXd V(125, 3);
    for (int k = 0; k <= 4; ++k) {
        for (int j = 0; j <= 4; ++j) {
            for (int i = 0; i <= 4; ++i)
                V.row(vid(i, j, k)) << 0.5 * i - 1, 0.5 * j - 1, 0.5 * k - 1;
        }
    }
    const std::array<std::array<int, 3>, 6> perms = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    std::vector<std::array<int64_t, 4>> T;
    std::vector<int> region; // 0 ambient, 1 a, 2 b
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) {
                for (const auto& pm : perms) {
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int64_t, 4> t;
                    t[0] = vid(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[size_t(pm[size_t(s)])];
                        t[size_t(s + 1)] = vid(c[0], c[1], c[2]);
                    }
                    const auto p = [&](int q) { return Vector3d(V.row(t[size_t(q)])); };
                    if ((p(1) - p(0)).dot((p(2) - p(0)).cross(p(3) - p(0))) < 0.)
                        std::swap(t[2], t[3]);
                    T.push_back(t);
                    region.push_back(
                        i == 1 && j == 1 && k == 1   ? 1
                        : i == 2 && j == 2 && k == 1 ? 2
                                                     : 0);
                }
            }
        }
    }
    MatrixXi Tm(Eigen::Index(T.size()), 4);
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(Eigen::Index(T.size()), 3);
    for (size_t r = 0; r < T.size(); ++r) {
        for (int c = 0; c < 4; ++c) Tm(Eigen::Index(r), c) = int(T[r][size_t(c)]);
        tags(Eigen::Index(r), region[r]) = 1;
    }
    MeshData m = io::geometry_data(V, Tm, CellType::Tetrahedron);
    m.cells.at(CellType::Tetrahedron).attributes["tags"] = {tags, {"ambient", "a", "b"}};
    add_tagged_block(
        m,
        CellType::Triangle,
        {{vid(0, 0, 3), vid(1, 0, 3), vid(1, 1, 3)}, {vid(0, 0, 3), vid(1, 1, 3), vid(0, 1, 3)}},
        "sheet");
    add_tagged_block(
        m,
        CellType::Edge,
        {{vid(1, 2, 2), vid(2, 2, 2)}, {vid(2, 2, 2), vid(3, 2, 2)}},
        "curve");
    add_tagged_block(m, CellType::Point, {{vid(3, 1, 3)}}, "p");
    return m;
}

/// The 2D twin of touching_cubes(): [-1, 1]^2 in 4^2 squares of edge 0.5, each split into 2
/// triangles along its diagonal, with the regions `a`, square (1, 1), and `b`, square (2, 2),
/// which touch at the vertex (0, 0): the boundary of a | b is not manifold there. Beside them the
/// blocks triwild and simwild write: the tagged edges `curve`, the two grid edges y = 0,
/// -0.5 <= x <= 0.5, which meet at that vertex, so that the carving splits them; the input point
/// `p`, (0.5, -0.5).
MeshData touching_squares()
{
    const auto vid = [](int i, int j) { return int64_t(i + 5 * j); };
    MatrixXd V(25, 2);
    for (int j = 0; j <= 4; ++j) {
        for (int i = 0; i <= 4; ++i) V.row(vid(i, j)) << 0.5 * i - 1, 0.5 * j - 1;
    }
    MatrixXi F(32, 3);
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(32, 3);
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            const int r = 2 * (i + 4 * j);
            F.row(r) << int(vid(i, j)), int(vid(i + 1, j)), int(vid(i + 1, j + 1));
            F.row(r + 1) << int(vid(i, j)), int(vid(i + 1, j + 1)), int(vid(i, j + 1));
            const int region = i == 1 && j == 1 ? 1 : i == 2 && j == 2 ? 2 : 0;
            tags(r, region) = tags(r + 1, region) = 1;
        }
    }
    MeshData m = io::geometry_data(V, F, CellType::Triangle);
    m.cells.at(CellType::Triangle).attributes["tags"] = {tags, {"ambient", "a", "b"}};
    add_tagged_block(m, CellType::Edge, {{vid(1, 2), vid(2, 2)}, {vid(2, 2), vid(3, 2)}}, "curve");
    add_tagged_block(m, CellType::Point, {{vid(3, 1)}}, "p");
    return m;
}

/// The rows of `type` in `out` under the label `name`, which must be its only label.
const CellData& block_of(const MeshData& out, const CellType type, const std::string& name)
{
    REQUIRE(out.cells.count(type) == 1);
    const CellData& block = out.cells.at(type);
    REQUIRE(block.attributes.at("tags").labels == std::vector<std::string>{name});
    CHECK((block.attributes.at("tags").integers().array() == 1).all());
    return block;
}

/// `curve` of `out`: on the line y = z = 0 between x = -0.5 and x = 0.5, its whole length 1 --
/// both grid edges, split by the carving into more than the two of the input.
void check_curve(const MeshData& out)
{
    const CellData& edges = block_of(out, CellType::Edge, "curve");
    CHECK(edges.connectivity.rows() > 2);
    double length = 0.;
    for (Eigen::Index r = 0; r < edges.connectivity.rows(); ++r) {
        const Vector3d a = out.V.row(edges.connectivity(r, 0)).transpose();
        const Vector3d b = out.V.row(edges.connectivity(r, 1)).transpose();
        for (const Vector3d& x : {a, b}) {
            CHECK(x.tail<2>().norm() <= 1e-12);
            CHECK(std::abs(x[0]) <= 0.5);
        }
        length += (b - a).norm();
    }
    CHECK(std::abs(length - 1.) <= 1e-12);
}

/// `p` of `out`: one row, at `at`.
void check_point(const MeshData& out, const Vector3d& at)
{
    const CellData& points = block_of(out, CellType::Point, "p");
    REQUIRE(points.connectivity.rows() == 1);
    CHECK((out.V.row(points.connectivity(0, 0)).transpose() - at).norm() == 0.);
}
} // namespace


TEST_CASE("manifold_extraction_carries_tagged_faces_curves_points_3d", "[manifold_extraction]")
{
    // The carving of a | b, with the tagged faces, the feature curve and the input point beside
    // it: each comes back as the same block of the result, under its label -- the curve split
    // where the carving crosses it, the faces and the point where they were. Refused until
    // 2026-11.
    const nlohmann::json j = {
        {"input", "m"},
        {"tag_selection", "a | b"},
        {"fill_tags", nlohmann::json::array()}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::manifold_extraction::manifold_extraction(j, {{"m", touching_cubes()}}));
    CHECK(res.second.at("after #t") > res.second.at("before #t")); // carved
    const MeshData& out = res.first.at("mesh");
    const auto& labels = out.cells.at(CellType::Tetrahedron).attributes.at("tags").labels;
    CHECK(std::count(labels.begin(), labels.end(), "a") == 1);
    CHECK(std::count(labels.begin(), labels.end(), "b") == 1);
    const CellData& sheet = block_of(out, CellType::Triangle, "sheet");
    CHECK(sheet.connectivity.rows() == 2);
    for (Eigen::Index r = 0; r < sheet.connectivity.rows(); ++r) {
        for (int c = 0; c < 3; ++c) {
            const Vector3d x = out.V.row(sheet.connectivity(r, c)).transpose();
            CHECK(x[2] == 0.5);
            CHECK((x[0] >= -1. && x[0] <= -0.5 && x[1] >= -1. && x[1] <= -0.5));
        }
    }
    check_curve(out);
    check_point(out, Vector3d(0.5, -0.5, 0.5));
}

TEST_CASE("manifold_extraction_carries_tagged_edges_points_2d", "[manifold_extraction][2d]")
{
    // The 2D twin of the case above: the tagged edges and the input point come back as the
    // result's edge and point blocks, the edges split where the carving crosses them. Refused
    // until 2026-11.
    const nlohmann::json j = {
        {"input", "m"},
        {"tag_selection", "a | b"},
        {"fill_tags", nlohmann::json::array()}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::manifold_extraction::manifold_extraction(j, {{"m", touching_squares()}}));
    CHECK(res.second.at("after #f") > res.second.at("before #f")); // carved
    const MeshData& out = res.first.at("mesh");
    check_curve(out);
    check_point(out, Vector3d(0.5, -0.5, 0.));
}
