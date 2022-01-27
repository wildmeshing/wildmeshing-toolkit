#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/writeDMAT.h>
#include <remeshing/UniformRemeshing.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <cstdlib>
#include <iostream>
#include <wmtk/utils/ManifoldUtils.hpp>
using namespace wmtk;

using namespace remeshing;
#include <chrono>
using namespace std::chrono;

extern "C" {
#include <wmtk/utils/getRSS.c>
};
void run_remeshing(std::string input, double len, std::string output, UniformRemeshing& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target len: {}", len);
    m.uniform_remeshing(len, 2);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    wmtk::logger().info("runtime {}", duration.count());
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

int main(int argc, char** argv)
{
    // input
    // output
    // ep
    const std::string root(WMT_DATA_DIR);
    const std::string path = argv[1];
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    Eigen::VectorXi SVI, SVJ;
    Eigen::MatrixXd temp_V = V; // for STL file
    igl::remove_duplicate_vertices(temp_V, 0, V, SVI, SVJ);
    for (int i = 0; i < F.rows(); i++)
        for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];
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
    const double envelope_size = atof(argv[3]) * diag;

    if (!igl::is_edge_manifold(F)) {
        wmtk::logger().info("Input is not edge manifold");
        return 1;
    } else {
        UniformRemeshing m(v, atoi(argv[4]));
        m.create_mesh(v.size(), tri, envelope_size);
        assert(m.check_mesh_connectivity_validity());
        std::vector<double> properties = m.average_len_valen();
        wmtk::logger().info(
            "edgelen: avg max min valence:avg max min before remesh is: {}",
            properties);
        igl::Timer timer;
        timer.start();
        run_remeshing(path, properties[0] * 5, std::string(argv[2]), m);
        //run_remeshing(path, properties[0] / 2, std::string(argv[2]), m);
        timer.stop();
        logger().info("Took {}", timer.getElapsedTimeInSec());
    }


    return 0;
}
