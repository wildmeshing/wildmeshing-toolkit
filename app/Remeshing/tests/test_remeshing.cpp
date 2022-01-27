
#include <remeshing/UniformRemeshing.h>
#include <catch2/catch.hpp>

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Core>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include "wmtk/utils/Logger.hpp"
using namespace wmtk;
using namespace remeshing;
#define CATCH_CONFIG_MAIN
#include <chrono>
using namespace std::chrono;

int test_thread()
{
    // input
    // output
    // ep
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/bunny.off";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    wmtk::logger().info("Before_vertices#: {} \n Before_tris#: {}", V.rows(), F.rows());


    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    const double diag = (box_max - box_min).norm();
    // const double envelope_size = atof(argv[3]) * diag;
    int max_num_threads = 64;

    if (!igl::is_edge_manifold(F)) {
        wmtk::logger().info("Input is not edge manifold");
        return 1;
    } else {
        for (int thread = 1; thread <= max_num_threads; thread *= 2) {
            wmtk::logger().info("======================  {} =====================", thread);
            std::vector<Eigen::Vector3d> v(V.rows());
            std::vector<std::array<size_t, 3>> tri(F.rows());
            for (int i = 0; i < V.rows(); i++) {
                v[i] = V.row(i);
            }
            for (int i = 0; i < F.rows(); i++) {
                for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
            }

            UniformRemeshing m(v, thread);
            // m.print_num_attributes();
            m.create_mesh(v.size(), tri, 0);
            assert(m.check_mesh_connectivity_validity());
            std::vector<double> properties = m.average_len_valen();
            wmtk::logger().info(
                "Before_vertices#: {} \n Before_tris#: {}",
                m.vert_capacity(),
                m.tri_capacity());
            // run_remeshing(path, properties[0] * 5, std::string(argv[2]), m);
            double len = 0.0023;
            std::string output = "../wildmesh_plotting/bunny";
            auto start = high_resolution_clock::now();
            wmtk::logger().info("target len: {}", len);
            m.uniform_remeshing(len, 2);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(stop - start);

            m.consolidate_mesh();
            m.write_triangle_mesh(fmt::format("{}_{}_{}.obj", output, len, thread));
            wmtk::logger().info("runtime {}", duration.count());
            wmtk::logger().info(
                "After_vertices#: {} \n After_tris#: {}",
                m.vert_capacity(),
                m.tri_capacity());
            std::vector<double> newproperties = m.average_len_valen();
            wmtk::logger().info("avg edge len : {}", newproperties[0]);
        }
    }
    return 0;
}
TEST_CASE("test_thread", "[test_util]")
{
    test_thread();
}