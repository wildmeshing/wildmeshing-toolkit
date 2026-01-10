#include <cstdlib>
#include <wmtk/components/topological_offset/TopoOffsetMesh.h>
#include <wmtk/utils/Delaunay.hpp>
#include <igl/write_triangle_mesh.h>
#include <Eigen/Core>

using namespace wmtk::delaunay;

void randMesh() {
    const int N_POINTS = 1000;
    double inc_prob = 0.1;
    std::srand(0);
    std::string opath = "/Users/seb9449/Desktop/wildmeshing/point_cloud.obj";

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
    Eigen::MatrixXi F(0, 3);  // dont need
    igl::write_triangle_mesh(opath, V, F);

    auto [verts, tets] = delaunay3D(points);
}
