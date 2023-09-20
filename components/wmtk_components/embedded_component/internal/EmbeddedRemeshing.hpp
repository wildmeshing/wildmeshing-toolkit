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
        Eigen::MatrixXi& m_edges_,
        Eigen::MatrixXd& m_vertices_,
        double m_blank_rate_ = 0.5
        // double m_resolute_area = 0.1
    );
    void compute_bounding_value(double& max_x, double& max_y, double& min_x, double& min_y);
    void process();

    Eigen::MatrixXi m_edges;
    Eigen::MatrixXd m_vertices;
    // m_blank_rate is used to define how big the boundary should be
    // if the m_blank_rate = 0.5, then the bounding box's x and y length
    // should be respectively 0.5 larger than the range of x and y of input
    // Therefore, the area of the bounding box is as 4 times as the input's bounding box's.
    double m_blank_rate;
    // double m_resolute_area; // this variable is not used for now, relate to the resolution and need to be discussed
    Eigen::MatrixXi m_faces;
    // can also be int, double any type, just code as bool for now.
    std::vector<bool> m_vertex_tags; // output tags
    std::vector<int> m_marked_vertices;
    std::vector<std::pair<int, int>> m_marked_edges;
};

class EmbeddedRemeshing3D
{
public:
    EmbeddedRemeshing3D() {}
    // this class will be filled with some code from exsiting wildmeshing toolkit.
    // talk with the senior student for this.
};

} // namespace wmtk::components::internal