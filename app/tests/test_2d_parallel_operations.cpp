#include <igl/Timer.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <stdlib.h>
#include <tbb/concurrent_vector.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <catch2/catch.hpp>
#include <iostream>
#include "EdgeOperations2d.h"

using namespace wmtk;

TEST_CASE("metis_test_bigmesh", "[test_2d_parallel_operations][.slow]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/circle.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    REQUIRE(ok);

    // change this for max concurrency
    int max_num_threads = 8;

    std::vector<double> timecost;

    for (int thread = 1; thread <= max_num_threads; thread *= 2) {
        std::vector<Eigen::Vector3d> v(V.rows());
        std::vector<std::array<size_t, 3>> tri(F.rows());
        for (int i = 0; i < V.rows(); i++) {
            v[i] = V.row(i);
        }
        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
        }

        Edge2d::EdgeOperations2d m(v, thread);
        // m.print_num_attributes();
        m.create_mesh(V.rows(), tri);
        REQUIRE(m.check_mesh_connectivity_validity());
        igl::Timer timer;
        double time;
        timer.start();

        // change this for num of operations
        m.collapse_shortest(2000);
        time = timer.getElapsedTimeInMilliSec();
        timecost.push_back(time);
    }
}
