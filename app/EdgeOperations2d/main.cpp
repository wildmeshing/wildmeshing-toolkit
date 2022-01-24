#include <EdgeOperations2d.h>
#include <igl/is_edge_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <cstdlib>
#include <iostream>
#include <wmtk/utils/ManifoldUtils.hpp>
using namespace wmtk;

using namespace Edge2d;
#include <chrono>
using namespace std::chrono;

extern "C" {
#include <wmtk/utils/getRSS.c>
};
void run(std::string input, double len, std::string output, EdgeOperations2d& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target len: {}", len);
    m.adaptive_remeshing(len, 2, 0);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime {}", duration.count());
    m.write_triangle_mesh(fmt::format("{}_{}.obj", output, len));
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

void run_shortest_collapse(std::string input, int target, std::string output, EdgeOperations2d& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target number of verts: {}", target);
    m.collapse_shortest(target);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime {}", duration.count());
    m.consolidate_mesh();
    m.write_triangle_mesh(fmt::format("{}_{}.obj", output, target));
    wmtk::logger().info(
        "After_vertices#: {} \n After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

int main(int argc, char** argv)
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + argv[1];
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

    if (!igl::is_edge_manifold(F)) {
        std::vector<Eigen::Vector3d> out_v;
        std::vector<std::array<size_t, 3>> out_f;
        wmtk::separate_to_manifold(v, tri, out_v, out_f);

        wmtk::logger().info(
            "After_separation_vertices#: {} \n After_separation_tris#: {}",
            out_v.size(),
            out_f.size());
        Eigen::MatrixXd outF = Eigen::MatrixXd::Zero(out_f.size(), 3);
        for (int i = 0; i < out_f.size(); i++) {
            outF.row(i) << out_f[i][0], out_f[i][1], out_f[i][2];
        }

        assert(igl::is_edge_manifold(out_f));
        EdgeOperations2d m(out_v);
        m.create_mesh(out_v.size(), out_f, atof(argv[3]));
        run_shortest_collapse(path, out_v.size() / 5, std::string(argv[2]), m);
    }

    else {
        EdgeOperations2d m(v);
        m.create_mesh(v.size(), tri, atof(argv[3]));
        assert(m.check_mesh_connectivity_validity());
        wmtk::logger().info("collapse to {}", v.size() - std::stoi(argv[2]));
        m.collapse_qec(v.size() - std::stoi(argv[2]));
        m.write_triangle_mesh("qec_result.obj");
    }

    // std::vector<double> properties = m.average_len_valen();
    // wmtk::logger().info(
    //     "edgelen: avg max min valence:avg max min before remesh is: {}",
    //     properties);
    // double small = properties[0] * 0.1;

    // run(path, properties[0] * 5, std::string(argv[2]), m);
    //run_shortest_collapse(path, out_v.size() / 5, std::string(argv[2]), m);

    return 0;
}
