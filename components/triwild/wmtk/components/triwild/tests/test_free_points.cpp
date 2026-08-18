#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/EmbedSegments.hpp>

#include <catch2/catch_test_macros.hpp>

#include <map>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

// A free point is an input vertex with no incident segment. The arrangement triangulates
// every point it is handed, so a free point becomes an ordinary mesh vertex; what makes it
// a FEATURE is the anchor registered at init, which reuses the endpoint/junction machinery
// (test_feature_points.cpp) unchanged. These tests cover the two new pieces: the point map
// out of embed_segments, and the registration in init_mesh.

TEST_CASE("embed-segments-point-map", "[triwild_operation][free_points]")
{
    // One segment plus two free points: one strictly inside the future triangulation, one
    // exactly ON the segment's interior -- the point the arrangement must MERGE with a
    // constrained-edge vertex rather than keep separate.
    MatrixXd V(4, 2);
    V << 0, 0, //
        2, 0, // segment (0,0)-(2,0)
        1, 1, // free point off the segment
        1, 0; // free point ON the segment interior
    MatrixXi E(1, 2);
    E << 0, 1;

    MatrixXd V_out;
    std::vector<Vector2r> V_rational;
    MatrixXi F_out, E_out;
    std::vector<int> point_map;
    utils::embed_segments(V, E, V_out, V_rational, F_out, E_out, nullptr, &point_map);

    REQUIRE(point_map.size() == 4);
    for (int i = 0; i < 4; ++i) {
        REQUIRE(point_map[i] >= 0);
        REQUIRE(point_map[i] < V_out.rows());
        // Every input point survives at its exact coordinates.
        CHECK(V_out(point_map[i], 0) == V(i, 0));
        CHECK(V_out(point_map[i], 1) == V(i, 1));
    }

    // The on-segment point splits the constrained edge: its output vertex must be an
    // endpoint of some constrained edge, and the segment must now be tiled by two of them.
    const int on_seg = point_map[3];
    int incident = 0;
    for (int e = 0; e < E_out.rows(); ++e) {
        if (E_out(e, 0) == on_seg || E_out(e, 1) == on_seg) {
            ++incident;
        }
    }
    CHECK(incident == 2);
    CHECK(E_out.rows() == 2);

    // The off-segment point is on no constrained edge.
    const int off_seg = point_map[2];
    for (int e = 0; e < E_out.rows(); ++e) {
        CHECK(E_out(e, 0) != off_seg);
        CHECK(E_out(e, 1) != off_seg);
    }
}

TEST_CASE("embed-segments-point-map-duplicates", "[triwild_operation][free_points]")
{
    // Exact duplicates merge to the same output vertex; the map reports it for both.
    MatrixXd V(4, 2);
    V << 0, 0, //
        1, 0, //
        0.5, 0.5, //
        0.5, 0.5;
    MatrixXi E(1, 2);
    E << 0, 1;

    MatrixXd V_out;
    std::vector<Vector2r> V_rational;
    MatrixXi F_out, E_out;
    std::vector<int> point_map;
    utils::embed_segments(V, E, V_out, V_rational, F_out, E_out, nullptr, &point_map);

    REQUIRE(point_map.size() == 4);
    CHECK(point_map[2] >= 0);
    CHECK(point_map[2] == point_map[3]);
}

TEST_CASE("free-point-coincident-with-endpoint", "[triwild_operation][free_points]")
{
    // A free point EXACTLY at an open polyline's endpoint. The arrangement merges the two
    // input points into one vertex; at init that vertex is first anchored as an endpoint,
    // and the free-point registration must then KEEP that id rather than stack a second
    // anchor at the same position -- the retention audit is geometric, one anchor covers
    // both readings of the point.
    MatrixXd V(3, 2);
    V << 0, 0, //
        2, 0, // open polyline (0,0)-(2,0)
        0, 0; // free point, exactly the first endpoint
    MatrixXi E(1, 2);
    E << 0, 1;

    MatrixXd V_arr;
    std::vector<Vector2r> V_rational;
    MatrixXi F_arr, E_arr;
    std::vector<int> point_map;
    utils::embed_segments(V, E, V_arr, V_rational, F_arr, E_arr, nullptr, &point_map);

    // The duplicate merged: the free point maps to the same vertex as the endpoint.
    REQUIRE(point_map[2] >= 0);
    CHECK(point_map[2] == point_map[0]);

    Parameters params;
    params.init(Vector2d(0, 0), Vector2d(2, 0));
    TriWildMesh mesh(params, /*envelope_eps=*/0.1);
    mesh.init_mesh(
        V_arr,
        V_rational,
        F_arr,
        E_arr,
        /*tag_names=*/{},
        V,
        E,
        /*free_point_vids=*/{size_t(point_map[2])});

    // Exactly the polyline's two endpoint anchors -- no third anchor for the free point.
    REQUIRE(mesh.m_feature_points.size() == 2);
    // The shared vertex carries the endpoint's id (the first registered), and both anchors
    // are covered geometrically.
    CHECK(mesh.m_vertex_extra[size_t(point_map[0])].m_feature_id != NO_FEATURE);
    const auto [kept, total] = mesh.feature_retention();
    CHECK(kept == 2);
    CHECK(total == 2);
}

TEST_CASE("init-mesh-registers-free-points", "[triwild_operation][free_points]")
{
    // Run the real pipeline front end -- arrangement, then init_mesh -- on a square curve
    // with one free point inside, and check the anchor comes out the other side.
    MatrixXd V(5, 2);
    V << 0, 0, //
        4, 0, //
        4, 4, //
        0, 4, //
        2, 2; // free point
    MatrixXi E(4, 2);
    E << 0, 1, 1, 2, 2, 3, 3, 0;

    MatrixXd V_arr;
    std::vector<Vector2r> V_rational;
    MatrixXi F_arr, E_arr;
    std::vector<int> point_map;
    utils::embed_segments(V, E, V_arr, V_rational, F_arr, E_arr, nullptr, &point_map);
    REQUIRE(point_map[4] >= 0);

    Parameters params;
    params.init(Vector2d(0, 0), Vector2d(4, 4));
    TriWildMesh mesh(params, /*envelope_eps=*/0.1);
    mesh.init_mesh(
        V_arr,
        V_rational,
        F_arr,
        E_arr,
        /*tag_names=*/{},
        V,
        E,
        /*free_point_vids=*/{size_t(point_map[4])});

    // The closed square has no endpoints and no junctions, so the free point is the only
    // anchor.
    REQUIRE(mesh.m_feature_points.size() == 1);
    CHECK(mesh.m_feature_points[0] == Vector2d(2, 2));
    CHECK(mesh.m_vertex_extra[size_t(point_map[4])].m_feature_id == 0);

    // And the retention audit sees it.
    const auto [kept, total] = mesh.feature_retention();
    CHECK(kept == 1);
    CHECK(total == 1);
}
