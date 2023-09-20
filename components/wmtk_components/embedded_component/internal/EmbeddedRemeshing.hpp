#pragma once

#include <igl/adjacency_list.h>
#include <igl/edges.h>
#include <igl/triangle/triangulate.h>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class EmbeddedRemeshing2D
{
public:
    EmbeddedRemeshing2D(
        Eigen::MatrixXi& E_,
        Eigen::MatrixXd& V_,
        double blank_rate_ = 0.5,
        double resolute_area = 0.1);
    void compute_bounding_vaule(double& max_x, double& max_y, double& min_x, double& min_y);
    void process();

    // can also be int, double any type, just code as bool for now.
    Eigen::MatrixXi E;
    Eigen::MatrixXd V;
    double blank_rate;
    double resolute_area;
    Eigen::MatrixXi F;
    std::vector<int> markedV;
    std::vector<bool> Vtags;
    std::vector<std::pair<int, int>> markedE;
};

class EmbeddedRemeshing3D
{
public:
    EmbeddedRemeshing3D() {}
    // this class will be filled with some code from exsiting wildmeshing toolkit.
    // talk with the senior student for this.
};

} // namespace wmtk::components::internal