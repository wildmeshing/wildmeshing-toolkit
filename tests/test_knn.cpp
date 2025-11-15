#include <wmtk/envelope/KNN.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace wmtk;

TEST_CASE("test_knn", "[envelope]")
{
    // This test is just for making sure the KNN API works, not for testing the underlying kd_tree,
    // i.e., nanoflann.

    std::vector<Vector3d> pts;
    pts.emplace_back(0, 0, 0);
    pts.emplace_back(1, 0, 0);
    pts.emplace_back(0, 1, 0);
    pts.emplace_back(0, 0, 1);

    KNN knn(pts);

    SECTION("single_neighbor")
    {
        const Vector3d q(0.9, 0, 0);
        uint32_t idx;
        double sq_dist;
        knn.nearest_neighbor(q, idx, sq_dist);
        CHECK(idx == 1);
        CHECK(pts[idx] == knn.point(idx));
    }
    SECTION("multiple_neighbors")
    {
        const Vector3d q(0.9, 0, 0);
        std::vector<uint32_t> idx;
        std::vector<double> sq_dist;
        idx.resize(2);
        sq_dist.resize(2);
        knn.nearest_neighbors(q, idx, sq_dist);
        REQUIRE(sq_dist.size() == 2);
        CHECK(sq_dist[0] < sq_dist[1]);
    }
}