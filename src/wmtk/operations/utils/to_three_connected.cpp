#include "to_three_connected.hpp"
#include <wmtk/utils/Logger.hpp>

#include <Eigen/Core>
#include <deque>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wmtk {
namespace operations {
namespace utils {

using AdjacencyList = std::unordered_map<int, std::unordered_set<int>>;

AdjacencyList build_adjacency(const Eigen::MatrixXi& faces)
{
    AdjacencyList adj;
    for (int i = 0; i < faces.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int u = faces(i, j);
            int v = faces(i, (j + 1) % 3);
            adj[u].insert(v);
            adj[v].insert(u);
        }
    }
    return adj;
}

bool is_separating_pair(const AdjacencyList& adj, int u, int v)
{
    std::unordered_set<int> banned = {u, v};
    std::unordered_set<int> verts;

    // Collect all vertices
    for (const auto& pair : adj) {
        verts.insert(pair.first);
    }

    std::unordered_set<int> remaining;
    for (int vertex : verts) {
        if (vertex != u && vertex != v) {
            remaining.insert(vertex);
        }
    }

    if (remaining.empty()) {
        return false;
    }

    int start = *remaining.begin();
    std::unordered_set<int> visited = {start};
    std::deque<int> queue = {start};

    while (!queue.empty()) {
        int cur = queue.front();
        queue.pop_front();

        for (int w : adj.at(cur)) {
            if (visited.find(w) == visited.end() && banned.find(w) == banned.end()) {
                visited.insert(w);
                queue.push_back(w);
            }
        }
    }

    return visited.size() != remaining.size();
}

std::pair<int, int> find_two_separator(const AdjacencyList& adj)
{
    std::vector<int> vertices;
    for (const auto& pair : adj) {
        vertices.push_back(pair.first);
    }

    std::sort(vertices.begin(), vertices.end());

    for (int u : vertices) {
        std::vector<int> neighbors(adj.at(u).begin(), adj.at(u).end());
        // std::sort(neighbors.begin(), neighbors.end());

        for (int v : neighbors) {
            if (u < v && is_separating_pair(adj, u, v)) {
                return {u, v};
            }
        }
    }

    return {-1, -1}; // No separator found
}

std::pair<Eigen::MatrixXi, Eigen::MatrixXd> split_edge(
    const Eigen::MatrixXi& faces,
    const std::pair<int, int>& edge,
    int new_vid,
    Eigen::MatrixXd& V)
{
    int u = edge.first;
    int v = edge.second;
    std::vector<Eigen::Vector3i> new_faces;

    // Create the new vertex at the midpoint of (u,v)
    Eigen::VectorXd new_vertex_pos = (V.row(u) + V.row(v)) / 2.0;

    // Append the new vertex to V
    Eigen::MatrixXd new_V(V.rows() + 1, V.cols());
    new_V.topRows(V.rows()) = V;
    new_V.row(V.rows()) = new_vertex_pos;
    V = new_V;

    for (int i = 0; i < faces.rows(); ++i) {
        Eigen::Vector3i tri = faces.row(i);

        // Check if the triangle contains both u and v
        bool has_u = (tri(0) == u || tri(1) == u || tri(2) == u);
        bool has_v = (tri(0) == v || tri(1) == v || tri(2) == v);

        if (has_u && has_v) {
            // Find the third vertex w
            int w = -1;
            for (int j = 0; j < 3; ++j) {
                if (tri(j) != u && tri(j) != v) {
                    w = tri(j);
                    break;
                }
            }

            // Find indices of u and v in the triangle
            int u_idx = -1, v_idx = -1;
            for (int j = 0; j < 3; ++j) {
                if (tri(j) == u) u_idx = j;
                if (tri(j) == v) v_idx = j;
            }

            // Check if u and v are adjacent in the triangle
            if ((u_idx + 1) % 3 == v_idx) { // u -> v is in order
                new_faces.push_back(Eigen::Vector3i(u, new_vid, w));
                new_faces.push_back(Eigen::Vector3i(new_vid, v, w));
            } else if ((v_idx + 1) % 3 == u_idx) { // v -> u is in order
                new_faces.push_back(Eigen::Vector3i(u, w, new_vid));
                new_faces.push_back(Eigen::Vector3i(new_vid, w, v));
            } else {
                // u and v are not adjacent, preserve the original orientation
                if ((u_idx + 1) % 3 == w) { // u -> w -> v
                    new_faces.push_back(Eigen::Vector3i(u, new_vid, w));
                    new_faces.push_back(Eigen::Vector3i(new_vid, v, w));
                } else { // v -> w -> u
                    new_faces.push_back(Eigen::Vector3i(u, w, new_vid));
                    new_faces.push_back(Eigen::Vector3i(new_vid, w, v));
                }
            }
        } else {
            new_faces.push_back(tri);
        }
    }

    // Convert vector of triangles back to Eigen matrix
    Eigen::MatrixXi result(new_faces.size(), 3);
    for (size_t i = 0; i < new_faces.size(); ++i) {
        result.row(i) = new_faces[i];
    }

    return {result, V};
}

void make_3_connected(const Eigen::MatrixXi& F, Eigen::MatrixXd& V)
{
    Eigen::MatrixXi faces = F;
    int next_vid = faces.maxCoeff() + 1;
    int iteration = 0;

    while (true) {
        AdjacencyList adj = build_adjacency(faces);
        std::pair<int, int> sep = find_two_separator(adj);

        if (sep.first == -1) {
            wmtk::logger().info(
                "✅ 3-connected transformation complete, split {} edges.",
                iteration);
            break;
        }

        wmtk::logger().info(
            "Iteration {}: Found separating pair ({}, {}), splitting edge, new vertex {}",
            iteration,
            sep.first,
            sep.second,
            next_vid);

        auto result = split_edge(faces, sep, next_vid, V);
        faces = result.first;
        next_vid++;
        iteration++;
    }
}

} // namespace utils
} // namespace operations
} // namespace wmtk
