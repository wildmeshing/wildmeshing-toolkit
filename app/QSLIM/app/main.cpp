#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/writeDMAT.h>
#include <qslim/QSLIM.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <wmtk/utils/ManifoldUtils.hpp>

extern "C" {
#include <wmtk/utils/getRSS.c>
};
using namespace wmtk;
using namespace qslim;
using namespace std::chrono;

void run_qslim_collapse(std::string input, int target, std::string output, QSLIM& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target number of verts: {}", target);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("mesh is valid");
    m.collapse_qslim(target);
    wmtk::logger().info("collapsed");
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime {}", duration.count());
    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    wmtk::logger().info(
        "After_vertices#: {} \n After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}
int main(int argc, char** argv)
{
    // input
    // target_verts
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

    const double envelope_size = atof(argv[4]) * diag;
    Eigen::VectorXi dummy;
    std::vector<size_t> modified_v;
    if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
        auto v1 = v;
        auto tri1 = tri;
        wmtk::separate_to_manifold(v1, tri1, v, tri, modified_v);
    }

    QSLIM m(v, atoi(argv[5]));
    m.create_mesh(v.size(), tri, modified_v, envelope_size);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("collapsing mesh {}", argv[1]);
    // auto edges = m.get_edges();
    // Eigen::MatrixXd C(edges.size(), 7);
    // for (int i = 0; i < edges.size(); i++) {
    //     auto e = edges[i];
    //     Eigen::Vector3d pos1 = m.vertex_attrs[e.vid()].pos;
    //     Eigen::Vector3d pos2 = m.vertex_attrs[e.switch_vertex(m).vid()].pos;

    //     C.row(i) << pos1[0], pos1[1], pos1[2], pos2[0], pos2[1], pos2[2],
    //     m.compute_cost_for_e(e);
    //     // wmtk::logger().info("this row is {}", C.row(i));
    // }
    // igl::writeDMAT("qslim_color.dmat", C);
    // wmtk::logger().info("priority written finish");
    int target_verts = v.size() * 0.1;

    igl::Timer timer;
    timer.start();
    run_qslim_collapse(path, target_verts, argv[3], m);
    timer.stop();
    logger().info("Took {}", timer.getElapsedTimeInSec());
    m.consolidate_mesh();
    // edges = m.get_edges();
    // C = Eigen::MatrixXd::Zero(edges.size(), 7);
    // for (int i = 0; i < edges.size(); i++) {
    //     auto e = edges[i];
    //     Eigen::Vector3d pos1 = m.vertex_attrs[e.vid()].pos;
    //     Eigen::Vector3d pos2 = m.vertex_attrs[e.switch_vertex(m).vid()].pos;

    //     C.row(i) << pos1[0], pos1[1], pos1[2], pos2[0], pos2[1], pos2[2],
    //     m.compute_cost_for_e(e);
    //     // x::logger().info("this row is {}", C.row(i));
    // }
    // igl::writeDMAT("qslim_post_color.dmat", C);
    // wmtk::logger().info("post priority written finish");
    return 0;
}
