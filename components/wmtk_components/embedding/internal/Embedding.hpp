#pragma once

#include <igl/adjacency_list.h>
#include <igl/edges.h>
#include <igl/triangle/triangulate.h>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include "EmbeddingOptions.hpp"

namespace wmtk::components::internal {

class Embedding
{
public:
    Embedding(
        Eigen::Matrix<long, -1, -1>& m_edges_,
        Eigen::MatrixXd& m_vertices_,
        EmbeddingOptions& options_,
        double m_blank_rate_ = 0.5
        // double m_resolute_area = 0.1
    );
    void compute_bounding_value(double& max_x, double& max_y, double& min_x, double& min_y);
    void process();

    // m_blank_rate is used to define how big the boundary should be.
    // if the m_blank_rate = 0.5, then the bbox's x and y length should be
    // respectively 0.5 larger than the range of x and y of input
    // Therefore, each side of the bbox will be double the input size in this case.
    double m_blank_rate;

    // this variable is not used for now, relate to the resolution and need to be discussed
    // double m_resolute_area;

    Eigen::Matrix<long, -1, -1> m_edges;
    Eigen::MatrixXd m_vertices;
    Eigen::Matrix<long, -1, -1> m_faces;
    // EmbeddingOptions options;

    // 0 for scallfold, 1 for input
    std::vector<long> m_vertex_tags; // output tags
    std::vector<long> m_marked_vertices;
    std::vector<std::pair<long, long>> m_marked_edges;
    EmbeddingOptions options;
};

} // namespace wmtk::components::internal