#pragma once

#include <igl/adjacency_list.h>
#include <igl/edges.h>
#include <igl/triangle/triangulate.h>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include "EmbeddingOptions.hpp"

namespace wmtk::components::internal {
/*
 * @Embedding
 * This class is used to embed a given low dimension mesh with a high dimension scaffold.
 * The embedding component should do as least as possible, i.e. the worst resolution that is still
 *valid. for now, this class only supports the 1-->2 dimension embedding.
 *
 * @input
 * m_edges: vertices' indices pairs of the edges
 * m_vertices: vertex position
 * options: please see the document in EmbeddingOptions.hpp
 * m_black_rate: used to define the Bbox size
 * @output
 * m_faces: 2D mesh composed of faces
 * m_edges: 2D mesh's edges
 * m_vertex_tags: each vertex's tag
 * m_marked_vertices: vertices on the input edges
 * m_marked_edges: edges on the input edges
 **/
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
    /*
     * @compute_bounding_value
     * This is a function used to compute the Bbox of the embedding area.
     * The area size depends on the parameter "m_blank_rate".
     *
     * @output
     * tuple with the order (max_x,max_y,min_x,min_y)
     * max_x: Bbox right edge's x coordinate
     * max_y: Bbox top edge's y coordinate
     * min_x: Bbox left edge's x coordinate
     * min_y: Bbox bottom edge's y coordinate
     **/
    std::tuple<double, double, double, double> compute_bounding_value();
    void process();

    /*
     *
     * m_blank_rate is used to define how big the boundary should be.
     * if the m_blank_rate = 0.5, then the bbox's x and y length should be
     * respectively 0.5 larger than the range of x and y of input
     * Therefore, each side of the bbox will be double the input size in this case.
     **/
    double m_blank_rate;

    Eigen::Matrix<long, -1, -1> m_edges;
    Eigen::MatrixXd m_vertices;
    Eigen::Matrix<long, -1, -1> m_faces;

    std::vector<long> m_vertex_tags; // output tags
    std::vector<long> m_marked_vertices;
    std::vector<std::pair<long, long>> m_marked_edges;
    EmbeddingOptions options;
};

} // namespace wmtk::components::internal