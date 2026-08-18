

#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <wmtk/components/tetwild/Parameters.h>
#include <wmtk/components/tetwild/TetWildMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <wmtk/envelope/Envelope.hpp>
#include "spdlog/common.h"
#include "wmtk/utils/InsertTriangleUtils.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/Partitioning.h>

using namespace wmtk;
using namespace components::tetwild;

TEST_CASE("triangle-insertion", "[tetwild_operation][.]")
{
    // Eigen::MatrixXd V;
    // Eigen::MatrixXd F;
    //std::string input_path = WMTK_DATA_DIR "/37322.stl";
    // igl::read_triangle_mesh(input_path, V, F);
    // wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());
    //
    // std::vector<Vector3d> vertices(V.rows());
    // std::vector<std::array<size_t, 3>> faces(F.rows());
    // for (int i = 0; i < V.rows(); i++) {
    //     vertices[i] = V.row(i);
    // }
    //std::vector<Eigen::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    // for (int i = 0; i < F.rows(); i++) {
    //     for (int j = 0; j < 3; j++) {
    //         faces[i][j] = F(i, j);
    //         env_faces[i][j] = F(i, j);
    //     }
    // }
    //
    // int NUM_THREADS = 1;
    // Parameters params;
    //params.lr = 1 / 15.0;
    // params.init(vertices, faces);
    // wmtk::ExactEnvelope envelope;
    // wmtk::logger().info("input_surface.params.eps {}", params.eps);
    // envelope.init(vertices, env_faces, params.eps);
    //
    // wmtk::remove_duplicates(vertices, faces, params.diag_l);
    // std::vector<size_t> partition_id(vertices.size(), 0);
    ////
    ////
    // tetwild::TetWild mesh(params, envelope);
    //
    // mesh.init_from_input_surface(vertices, faces, partition_id);
    // REQUIRE(mesh.check_attributes());
}


TEST_CASE("triangle-insertion-parallel", "[tetwild_operation][.]")
{
    // Eigen::MatrixXd V;
    // Eigen::MatrixXd F;
    //std::string input_path = WMTK_DATA_DIR "/Octocat.obj";
    // igl::read_triangle_mesh(input_path, V, F);
    // wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());
    //
    // std::vector<Vector3d> vertices(V.rows());
    // std::vector<std::array<size_t, 3>> faces(F.rows());
    // for (int i = 0; i < V.rows(); i++) {
    //     vertices[i] = V.row(i);
    // }
    //std::vector<Eigen::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    // for (int i = 0; i < F.rows(); i++) {
    //     for (int j = 0; j < 3; j++) {
    //         faces[i][j] = F(i, j);
    //         env_faces[i][j] = F(i, j);
    //     }
    // }
    //
    // int NUM_THREADS = 16;
    //
    // Parameters params;
    //params.lr = 1 / 30.0;
    // params.init(vertices, faces);
    //
    // wmtk::ExactEnvelope envelope;
    // envelope.init(vertices, env_faces, params.eps);
    //
    // wmtk::remove_duplicates(vertices, faces, params.diag_l);
    // Eigen::MatrixXd new_F(faces.size(), 3);
    // for (int i = 0; i < faces.size(); i++) {
    //     new_F(i, 0) = faces[i][0];
    //     new_F(i, 1) = faces[i][1];
    //     new_F(i, 2) = faces[i][2];
    // }
    // auto partitioned_v = partition_mesh_vertices(new_F, NUM_THREADS);
    // std::vector<size_t> partition_id(partitioned_v.rows());
    //
    // std::vector<int> cnt_id(NUM_THREADS);
    // for (int i = 0; i < partitioned_v.rows(); i++) {
    //     partition_id[i] = partitioned_v(i, 0);
    //     cnt_id[partition_id[i]]++;
    // }
    //
    ////
    ////
    // tetwild::TetWild mesh(params, envelope, NUM_THREADS);
    //
    // wmtk::logger().info("start insertion");
    // mesh.init_from_input_surface(vertices, faces, partition_id);
    // wmtk::logger().info("end insertion");
}

TEST_CASE("vertex_order", "[tetwild]")
{
    using Tuple = TetMesh::Tuple;

    MatrixXd V;
    MatrixXi F;

    size_t nvo3 = 0; // number of order 3 vertices
    SECTION("shark-fin")
    {
        // one-sided shark fin
        V.resize(9, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(2, 0, 0);
        V.row(2) = Vector3d(3, 0, 0);
        V.row(3) = Vector3d(1, 1, 0);
        V.row(4) = Vector3d(3, 1, 0);
        V.row(5) = Vector3d(0, 2, 0);
        V.row(6) = Vector3d(2, 2, 0);
        V.row(7) = Vector3d(3, 2, 0);
        V.row(8) = Vector3d(2, 1, 1);
        F.resize(8, 3);
        F.row(0) = Vector3i(0, 1, 3);
        F.row(1) = Vector3i(1, 4, 3);
        F.row(2) = Vector3i(1, 2, 4);
        F.row(3) = Vector3i(0, 3, 5);
        F.row(4) = Vector3i(3, 6, 5);
        F.row(5) = Vector3i(3, 4, 6);
        F.row(6) = Vector3i(4, 7, 6);
        F.row(7) = Vector3i(3, 4, 8);

        nvo3 = 2;
    }
    SECTION("two-triangles")
    {
        // two triangles touching in one vertex
        V.resize(5, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(2, 0, 0);
        V.row(2) = Vector3d(1, 1, 0);
        V.row(3) = Vector3d(0, 2, 0);
        V.row(4) = Vector3d(2, 2, 0);
        F.resize(2, 3);
        F.row(0) = Vector3i(0, 1, 2);
        F.row(1) = Vector3i(2, 4, 3);

        nvo3 = 1;
    }
    SECTION("two-triangles-disconnected")
    {
        // two triangles touching in one vertex
        V.resize(6, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(2, 0, 0);
        V.row(2) = Vector3d(1, 1, 0);
        V.row(3) = Vector3d(0, 2, 0);
        V.row(4) = Vector3d(2, 2, 0);
        V.row(5) = Vector3d(1, 1.1, 0);
        F.resize(2, 3);
        F.row(0) = Vector3i(0, 1, 2);
        F.row(1) = Vector3i(3, 4, 5);

        nvo3 = 0;
    }
    SECTION("hour-glass")
    {
        V.resize(7, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(0, -1, 0);
        V.row(2) = Vector3d(-1, 0, 0);
        V.row(3) = Vector3d(0, 0, -1);
        V.row(4) = Vector3d(1, 0, 0);
        V.row(5) = Vector3d(0, 0, 1);
        V.row(6) = Vector3d(0, 1, 0);
        F.resize(8, 3);
        F.row(0) = Vector3i(0, 2, 1);
        F.row(1) = Vector3i(0, 1, 3);
        F.row(2) = Vector3i(2, 0, 3);
        F.row(3) = Vector3i(1, 2, 3);
        F.row(4) = Vector3i(4, 5, 0);
        F.row(5) = Vector3i(0, 5, 6);
        F.row(6) = Vector3i(4, 0, 6);
        F.row(7) = Vector3i(6, 5, 4);

        nvo3 = 1;
    }

    std::vector<Vector3d> vertices;
    std::vector<std::array<size_t, 3>> faces;

    VF_to_vectors(V, F, vertices, faces);

    Parameters params;
    params.preserve_topology = true;
    params.init(vertices, faces);

    components::shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(vertices, 0);
    {
        std::vector<size_t> frozen_verts;
        surf_mesh.create_mesh(vertices.size(), faces, frozen_verts, 0.1);
    }
    REQUIRE(surf_mesh.check_mesh_connectivity_validity());

    // TetWildMesh now holds the envelope by shared_ptr; surf_mesh owns this one and
    // outlives both meshes below, so alias it with a no-op deleter rather than copying the
    // BVH. Same aliasing the old `SampleEnvelope&` parameter gave.
    const std::shared_ptr<SampleEnvelope> env(&surf_mesh.m_envelope, [](SampleEnvelope*) {});

    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;
    {
        TetWildMesh mesh_insertion(params, env, 0);
        mesh_insertion.insertion_by_volumeremesher(
            vertices,
            faces,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface);
    }

    // generate new mesh
    TetWildMesh mesh(params, env, 0);

    mesh.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);

    // mesh.save_paraview("debug_shark_fin", false);

    // check order of vertices
    size_t vo3_count = 0;
    for (const Tuple& t : mesh.get_vertices()) {
        const size_t vid = t.vid(mesh);
        const size_t vo = mesh.get_order_of_vertex(vid);
        if (vo == 3) {
            ++vo3_count;
        }

        if (vo == 2) {
            // must have order 2 edges incident
            const auto vs = mesh.get_one_ring_vids_for_vertex(vid);
            size_t eo2_count = 0;
            for (const size_t v : vs) {
                if (mesh.get_order_of_edge({{vid, v}}) == 2) {
                    ++eo2_count;
                }
            }
            CHECK(eo2_count == 2);
        }
    }

    CHECK(nvo3 == vo3_count);
}
// ---------------------------------------------------------------------------
// Feature edges and points through the insertion, checked in exact arithmetic.
//
// The contract (VolumeRemesher commit e09a229): each input edge is represented
// by output tet edges that tile it -- collinear with it, contiguous, and
// running endpoint to endpoint -- and each input point by an output vertex
// exactly equal to it. These checks are the wmtk-side counterpart of the
// remesher's own verify_tracking, run on the COMPACTED ids the caller gets.
// ---------------------------------------------------------------------------
namespace {

// Exact check that `tiling` tiles segment AB: every tiling vertex on the line
// through AB with parameter t in [0,1], and the pieces cover [0,1] with no gap.
void require_exact_tiling(
    const std::vector<std::array<size_t, 2>>& tiling,
    const std::vector<Vector3r>& v_rational,
    const Vector3d& seg_a,
    const Vector3d& seg_b)
{
    REQUIRE(!tiling.empty());
    const Vector3r a{Rational(seg_a[0]), Rational(seg_a[1]), Rational(seg_a[2])};
    const Vector3r d{
        Rational(seg_b[0] - seg_a[0]),
        Rational(seg_b[1] - seg_a[1]),
        Rational(seg_b[2] - seg_a[2])};
    const Rational dd = d.dot(d);

    // Parameter of a vertex along AB, after requiring it exactly on the line.
    const auto param = [&](size_t vid) -> Rational {
        const Vector3r p = v_rational.at(vid) - a;
        const Vector3r c = p.cross(d);
        REQUIRE(c[0] == Rational(0));
        REQUIRE(c[1] == Rational(0));
        REQUIRE(c[2] == Rational(0));
        return p.dot(d) / dd;
    };

    std::vector<std::pair<Rational, Rational>> intervals;
    intervals.reserve(tiling.size());
    for (const auto& e : tiling) {
        Rational t0 = param(e[0]);
        Rational t1 = param(e[1]);
        if (t1 < t0) {
            std::swap(t0, t1);
        }
        REQUIRE(t0 >= Rational(0));
        REQUIRE(t1 <= Rational(1));
        REQUIRE(t0 < t1); // no degenerate pieces
        intervals.emplace_back(t0, t1);
    }
    std::sort(intervals.begin(), intervals.end(), [](const auto& x, const auto& y) {
        return x.first < y.first;
    });
    REQUIRE(intervals.front().first == Rational(0));
    for (size_t i = 1; i < intervals.size(); ++i) {
        REQUIRE(intervals[i].first == intervals[i - 1].second); // contiguous, no overlap
    }
    REQUIRE(intervals.back().second == Rational(1));
}

} // namespace

TEST_CASE("insertion-feature-edges-and-points", "[tetwild_operation][features]")
{
    // A cube [0,2]^3, with features interior to it, on its surface, and touching a corner.
    MatrixXd V(8, 3);
    V << 0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
        0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2;
    MatrixXi F(12, 3);
    F << 0, 2, 1, 0, 3, 2, // z = 0
        4, 5, 6, 4, 6, 7, // z = 2
        0, 1, 5, 0, 5, 4, // y = 0
        2, 3, 7, 2, 7, 6, // y = 2
        1, 2, 6, 1, 6, 5, // x = 2
        0, 4, 7, 0, 7, 3; // x = 0

    std::vector<Vector3d> vertices;
    std::vector<std::array<size_t, 3>> faces;
    VF_to_vectors(V, F, vertices, faces);

    Parameters params;
    params.init(vertices, faces);

    components::shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(vertices, 0);
    {
        std::vector<size_t> frozen_verts;
        surf_mesh.create_mesh(vertices.size(), faces, frozen_verts, 0.1);
    }
    const std::shared_ptr<SampleEnvelope> env(&surf_mesh.m_envelope, [](SampleEnvelope*) {});

    // Features: an interior diagonal edge chain (two edges sharing a vertex), an edge lying
    // ON the y = 0 face, an interior point, a point on the surface, and a point exactly at a
    // cube corner (already a background vertex -- the dedup case).
    const std::vector<Vector3d> fe_verts = {
        {0.5, 0.5, 0.5},
        {1.0, 1.0, 1.0},
        {1.5, 1.0, 1.5}, // interior chain 0-1, 1-2
        {0.5, 0.0, 0.5},
        {1.5, 0.0, 1.5}, // on-surface edge 3-4
    };
    const std::vector<std::array<size_t, 2>> fe = {{{0, 1}}, {{1, 2}}, {{3, 4}}};
    const std::vector<Vector3d> fp = {
        {1.0, 0.5, 1.5}, // interior
        {2.0, 1.0, 1.0}, // on the x = 2 face
        {0.0, 0.0, 0.0}, // exactly a cube corner
    };

    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;
    utils::EmbedFeaturesResult features_out;
    {
        TetWildMesh mesh_insertion(params, env, 0);
        mesh_insertion.insertion_by_volumeremesher(
            vertices,
            faces,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            fe_verts,
            fe,
            fp,
            &features_out);
    }

    // Every feature edge is tiled exactly, endpoint to endpoint.
    REQUIRE(features_out.edge_tiling.size() == fe.size());
    for (size_t e = 0; e < fe.size(); ++e) {
        require_exact_tiling(
            features_out.edge_tiling[e],
            v_rational,
            fe_verts[fe[e][0]],
            fe_verts[fe[e][1]]);
    }

    // Every feature point is an output vertex, exactly.
    REQUIRE(features_out.point_vertex.size() == fp.size());
    for (size_t p = 0; p < fp.size(); ++p) {
        const int64_t vid = features_out.point_vertex[p];
        REQUIRE(vid >= 0);
        for (int k = 0; k < 3; ++k) {
            CHECK(v_rational[size_t(vid)][k] == Rational(fp[p][k]));
        }
    }

    // And the tiling edges are real tet edges of the output connectivity.
    std::set<std::pair<size_t, size_t>> tet_edges;
    for (const auto& t : tets) {
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                tet_edges.emplace(std::min(t[i], t[j]), std::max(t[i], t[j]));
            }
        }
    }
    for (const auto& tiling : features_out.edge_tiling) {
        for (const auto& e : tiling) {
            CHECK(tet_edges.count({std::min(e[0], e[1]), std::max(e[0], e[1])}) == 1);
        }
    }

    // Init the mesh with the features and check the tags landed: every tiling edge's slot
    // is tagged and its endpoints are flagged.
    TetWildMesh mesh(params, env, 0);
    mesh.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface,
        &features_out);

    for (size_t ie = 0; ie < features_out.edge_tiling.size(); ++ie) {
        for (const auto& e : features_out.edge_tiling[ie]) {
            const auto t = mesh.tuple_from_edge({{e[0], e[1]}});
            REQUIRE(t.is_valid(mesh));
            CHECK(mesh.m_feature_edge_attribute[t.eid(mesh)].m_is_feature_edge);
            CHECK(mesh.m_vertex_extra[e[0]].m_is_on_feature_curve);
            CHECK(mesh.m_vertex_extra[e[1]].m_is_on_feature_curve);
        }
    }
    // A vertex NOT on any feature curve is not flagged: count flagged vertices and compare
    // with the union of tiling vertices.
    std::set<size_t> tiling_verts;
    for (const auto& tiling : features_out.edge_tiling) {
        for (const auto& e : tiling) {
            tiling_verts.insert(e[0]);
            tiling_verts.insert(e[1]);
        }
    }
    size_t flagged = 0;
    for (size_t v = 0; v < mesh.vert_capacity(); ++v) {
        flagged += mesh.m_vertex_extra[v].m_is_on_feature_curve ? 1 : 0;
    }
    CHECK(flagged == tiling_verts.size());
}
