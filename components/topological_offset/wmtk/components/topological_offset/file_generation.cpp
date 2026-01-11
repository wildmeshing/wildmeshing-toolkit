#include <cstdlib>
#include <wmtk/components/topological_offset/TopoOffsetMesh.h>
#include <wmtk/utils/Delaunay.hpp>
#include <igl/write_triangle_mesh.h>
#include <Eigen/Core>
#include <wmtk/utils/io.hpp>
#include "read_image_msh.hpp"


using namespace wmtk::delaunay;


void write_manual_msh(std::string path, std::vector<Point3D> verts, std::vector<Tetrahedron> tets, std::vector<double> wns) {
    // logger().info("Write {}", file);

    wmtk::MshData msh;

    msh.add_tet_vertices(verts.size(), [&](size_t i) {
        return verts[i];
    });

    msh.add_tets(tets.size(), [&](size_t i) {
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = tets[i][j];
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_tet_attribute<1>("winding_number_tracked", [&](size_t i) {
        return wns[i];
    });

    msh.save(path, true);
}


void edgeNonManifoldMesh() {
    std::string path = "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/nonman_edge.msh";

    std::vector<Point3D> verts;
    verts.push_back({-1, 0, 0});
    verts.push_back({0, -1, 0});
    verts.push_back({0, 0, 0});
    verts.push_back({0, 0, 1});
    verts.push_back({1, 0, 0});
    verts.push_back({0, 1, 0});

    std::vector<Tetrahedron> tets;
    tets.push_back({0, 1, 2, 3});
    tets.push_back({2, 3, 4, 5});

    std::vector<double> wns;
    wns.push_back(0.8);
    wns.push_back(0.9);

    write_manual_msh(path, verts, tets, wns);
}


void edgeNonManifoldOBJ() {
    std::string path = "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/nonman_edge.obj";

    Eigen::MatrixXd V(6, 3);
    V << -1, 0, 0,
         0, -1, 0,
         0, 0, 0,
         0, 0, 1,
         1, 0, 0,
         0, 1, 0;

    Eigen::MatrixXi F(8, 3);
    F << 0, 1, 3,
         1, 2, 3,
         0, 3, 2,
         2, 1, 0,
         5, 3, 4,
         2, 3, 5,
         4, 3, 2,
         4, 2, 5;

    igl::write_triangle_mesh(path, V, F);
}


void randMesh() {
    const int N_POINTS = 1000;
    std::srand(0);
    // std::string opath = "/Users/seb9449/Desktop/wildmeshing/point_cloud.obj";
    std::string opath = "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/rand_mesh.msh";

    std::vector<Point3D> points;
    for (int i = 0; i < N_POINTS; i++) {
        double x = ((double) (std::rand() % 10000)) / 1000.0;
        double y = ((double) (std::rand() % 10000)) / 1000.0;
        double z = ((double) (std::rand() % 10000)) / 1000.0;
        Point3D p = {x, y, z};
        points.push_back(p);
    }

    Eigen::MatrixXd V(N_POINTS, 3);
    for (int i = 0; i < N_POINTS; i++) {
        V(i, 0) = points[i][0];
        V(i, 1) = points[i][1];
        V(i, 2) = points[i][2];
    }
    // Eigen::MatrixXi F(0, 3);  // dont need
    // igl::write_triangle_mesh(opath, V, F);

    auto [verts, tets] = delaunay3D(points);

    // random tagging
    std::vector<double> wns;
    std::srand(100);
    for (int i = 0; i < tets.size(); i++) {
        double val = ((double) std::rand()) / 100000.0;
        double ipart;
        double dpart;
        dpart = std::modf(val, &ipart);
        wns.push_back(dpart);
    }

    // save
    write_manual_msh(opath, verts, tets, wns);
}

