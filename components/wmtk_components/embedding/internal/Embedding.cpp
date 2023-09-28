#include "Embedding.hpp"

namespace wmtk::components::internal {
Embedding::Embedding(
    Eigen::MatrixXi& m_edges_,
    Eigen::MatrixXd& m_vertices_,
    double m_blank_rate_
    // double m_resolute_area_)
    )
    : m_edges(m_edges_)
    , m_vertices(m_vertices_)
    , m_blank_rate(m_blank_rate_)
//, m_resolute_area(m_resolute_area_)
{
    m_marked_vertices.reserve(m_vertices.rows());
    m_marked_edges.reserve(m_edges.rows());

    for (size_t i = 0; i < m_vertices.rows(); ++i) {
        m_marked_vertices.emplace_back(i);
    }
    for (size_t i = 0; i < m_edges.rows(); ++i) {
        m_marked_edges.emplace_back(std::pair<long, long>(m_edges(i, 0), m_edges(i, 1)));
    }
}
void Embedding::compute_bounding_value(double& max_x, double& max_y, double& min_x, double& min_y)
{
    max_x = max_y = std::numeric_limits<double>::lowest();
    min_x = min_y = std::numeric_limits<double>::max();
    for (size_t i = 0; i < m_vertices.rows(); ++i) {
        max_x = std::max(max_x, m_vertices(i, 0));
        max_y = std::max(max_y, m_vertices(i, 1));
        min_x = std::min(max_x, m_vertices(i, 0));
        min_y = std::min(max_x, m_vertices(i, 1));
    }
    const double blank_region_length_x = (max_x - min_x) * m_blank_rate;
    const double blank_region_length_y = (max_y - min_y) * m_blank_rate;
    max_x += blank_region_length_x;
    min_x -= blank_region_length_x;
    max_y += blank_region_length_y;
    min_y -= blank_region_length_y;
}
void Embedding::process()
{
    // find boundingbox
    double max_x, min_x, max_y, min_y;
    compute_bounding_value(max_x, max_y, min_x, min_y);

    Eigen::MatrixXd bounding_box_vertices(4, 2);
    Eigen::MatrixXi bounding_box_edges(4, 2);
    bounding_box_vertices << max_x, max_y, min_x, max_y, min_x, min_y, max_x, min_y;
    bounding_box_edges << m_marked_vertices.size(), m_marked_vertices.size() + 1,
        m_marked_vertices.size() + 1, m_marked_vertices.size() + 2, m_marked_vertices.size() + 2,
        m_marked_vertices.size() + 3, m_marked_vertices.size() + 3, m_marked_vertices.size();

    // NOTE: make sure give the matrix right size before assign a value to them
    Eigen::MatrixXd temp_vertices(m_vertices.rows() + bounding_box_vertices.rows(), 2);
    Eigen::MatrixXi temp_edges(m_edges.rows() + bounding_box_edges.rows(), 2);
    temp_vertices << m_vertices, bounding_box_vertices;
    temp_edges << m_edges, bounding_box_edges;

    // this is for triangulate function, should be outside of the bounding box.
    Eigen::MatrixXd outside_vertex;
    outside_vertex.resize(1, 2);
    outside_vertex << max_x + 1.0, max_y + 1.0;

    // the fourth parameter is resolution, we should discuss it maybe.
    igl::triangle::triangulate(
        temp_vertices,
        temp_edges,
        outside_vertex,
        "a0.1q",
        m_vertices,
        m_faces);

    // spdlog::info("DONE FOR TRIANGULATE FUNCTION\n");

    // need to connect the topology
    // it would be easy, just check each edge and then move on
    std::vector<std::vector<long>> adj_list;
    igl::adjacency_list(m_faces, adj_list);

    std::vector<std::pair<long, long>> temp_marked_edges;
    std::vector<long> temp_marked_vertices;
    for (const auto& e : m_marked_edges) {
        // find marked E index before triangulate operation
        long vid0 = e.first, vid1 = e.second;
        Eigen::Vector2<double> p0, p1, dir;
        p0 << m_vertices(vid0, 0), m_vertices(vid0, 1);
        p1 << m_vertices(vid1, 0), m_vertices(vid1, 1);
        long cur_vid = vid0;
        temp_marked_vertices.push_back(cur_vid);
        // our start vertex is p0, and constantly head to p1, and stop when encounter p1
        while (cur_vid != vid1) {
            Eigen::Vector2<double> cur_p;
            cur_p << m_vertices(cur_vid, 0), m_vertices(cur_vid, 1);
            dir = (p1 - cur_p).normalized();
            const std::vector<long>& neighbour_v_list = adj_list[cur_vid];
            double value_dir = -1;
            long next_vid = -1;
            // for each step, just foward the direction with less value of dot operation(sin)
            for (const long neighbour_id : neighbour_v_list) {
                Eigen::Vector2<double> p, cur_dir;
                p << m_vertices(neighbour_id, 0), m_vertices(neighbour_id, 1);
                cur_dir = (p - cur_p).normalized();
                if (value_dir < cur_dir.dot(dir)) {
                    value_dir = cur_dir.dot(dir);
                    next_vid = neighbour_id;
                }
            }
            long vid0_ = cur_vid, vid1_ = next_vid;
            if (vid0_ > vid1_) std::swap(vid0_, vid1_);
            temp_marked_edges.push_back(std::pair<long, long>(vid0_, vid1_));
            temp_marked_vertices.push_back(next_vid);
            cur_vid = next_vid;
        }
    }
    std::sort(temp_marked_vertices.begin(), temp_marked_vertices.end());
    temp_marked_vertices.erase(
        std::unique(temp_marked_vertices.begin(), temp_marked_vertices.end()),
        temp_marked_vertices.end());
    m_marked_edges = temp_marked_edges;
    m_marked_vertices = temp_marked_vertices;

    m_vertex_tags = std::vector<long>(m_vertices.rows(), 0);
    for (const int marked_vertex_idx : m_marked_vertices) {
        m_vertex_tags[marked_vertex_idx] = 1;
    }
}

} // namespace wmtk::components::internal