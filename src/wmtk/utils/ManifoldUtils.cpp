#include "ManifoldUtils.hpp"

#include <igl/extract_manifold_patches.h>
#include <igl/is_edge_manifold.h>
#include <igl/remove_unreferenced.h>
#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/ostr.h>
#include <Eigen/Core>
#include "spdlog/spdlog.h"
#include "wmtk/utils/VectorUtils.h"

#include <cstddef>
#include <set>


/**
 * Adobe Code, Apache License, from Lagrange.
 */

void wmtk::resolve_nonmanifoldness(
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    std::vector<size_t>& modified_vertices)
{
    using Index = size_t;

    const Index num_vertices = V.rows();
    const Index num_facets = F.rows();
    const Index vertex_per_facet = 3;
    const auto& vertices = V;
    const auto& facets = F;

    // collect edges and relevant utils
    std::map<std::array<size_t, 2>, size_t> v2_to_eid;

    Eigen::MatrixXi edge_ids(F.rows(), 3); // global edge id for each tri.
    std::vector<std::vector<int>> VF_map(V.rows());
    std::vector<std::vector<int>> edge_adjacent_faces;
    edge_adjacent_faces.reserve(F.rows() * 2);
    std::vector<std::array<size_t, 2>> edges;
    auto cur_eid = 0;
    for (auto i = 0; i < F.rows(); i++) {
        for (auto j = 0; j < 3; j++) {
            size_t v0 = F(i, j), v1 = F(i, (j + 1) % 3);
            VF_map[v0].push_back(i);
            if (v0 > v1) std::swap(v0, v1);
            auto v2 = std::array<size_t, 2>{{v0, v1}};
            auto it = v2_to_eid.find(v2);
            if (it == v2_to_eid.end()) { // new edge
                v2_to_eid.emplace(v2, cur_eid);
                edge_ids(i, j) = cur_eid;
                edge_adjacent_faces.emplace_back(std::vector<int>{i});
                edges.emplace_back(v2);
                cur_eid++;
            } else { // existing
                auto eid = it->second;
                edge_ids(i, j) = eid;
                edge_adjacent_faces[eid].push_back(i);
            }
        }
    }
    auto get_edge_vertices = [&edges](size_t eid) -> std::array<size_t, 2> { return edges[eid]; };
    auto get_edge = [&](auto fid, auto j) -> int { return edge_ids(fid, j); };
    auto num_facets_around_edge = [&](size_t eid) { return edge_adjacent_faces[eid].size(); };
    auto foreach_facets_around_edge = [&](size_t eid, const auto& func) {
        for (auto f : edge_adjacent_faces[eid]) func(f);
    };
    auto get_num_edges = [&]() { return edge_adjacent_faces.size(); };
    auto get_facets_adjacent_to_vertex = [&](size_t i) { return VF_map[i]; };


    /**
     * Return true iff Edge e is consistently oriented with
     * the specified facet.
     *
     * This method assumes e is a valid edge in the facet.
     */
    auto get_orientation = [&facets](const Index v0, const Index v1, const Index fid) -> bool {
        const auto& f = facets.row(fid);
        if (f[0] == v0 && f[1] == v1)
            return true;
        else if (f[1] == v0 && f[2] == v1)
            return true;
        else if (f[2] == v0 && f[0] == v1)
            return true;
        else
            return false;
    };

    /**
     * Return true iff facets around an edge are **inconsistently** oriented.
     * E.g. f0: * [0, 1, 2] and f1: [1, 2, 3], with e=[1, 2]
     *
     * @note: This method does not depend on the orientaiton of the edge e.
     *        This method also assumes the edge `ei` has exactly 2 adjacent
     *        facets.
     */
    auto is_inconsistently_oriented = [&](const Index ei) -> bool {
        const auto e = get_edge_vertices(ei);
        std::array<bool, 2> orientations;
        size_t count = 0;
        foreach_facets_around_edge(ei, [&](Index fid) {
            orientations[count] = get_orientation(e[0], e[1], fid);
            count++;
        });
        return orientations[0] == orientations[1];
    };

    /**
     * Return true iff f0 and f1 are consistantly oriented with respect to edge
     * ei.  This is **almost** the same as `is_inconsistently_oriented`.
     * With `f0` and `f1` specified, this version can work with non-manifold
     * edges.
     */
    auto is_inconsistently_oriented_wrt_facets =
        [&](const Index ei, const Index f0, const Index f1) {
            const auto e = get_edge_vertices(ei);
            return get_orientation(e[0], e[1], f0) == get_orientation(e[0], e[1], f1);
        };

    /**
     * Return true iff edge e has more than 2 incident facets or it has
     * exactly 2 incident facet but they are inconsistently oriented.
     */
    auto is_nonmanifold_edge = [&](const Index ei) {
        auto edge_valence = num_facets_around_edge(ei);
        if (edge_valence > 2) return true;
        if (edge_valence <= 1) return false;
        return is_inconsistently_oriented(ei);
    };

    // Flood fill color across manifold edges.  The color field split the
    // facets into locally manifold components.  Edges and vertices adjacent
    // to multiple colors will be split.
    constexpr Index BLANK = 0;
    std::vector<Index> colors(num_facets, BLANK);
    Index curr_color = 1;
    for (Index i = 0; i < num_facets; i++) {
        if (colors[i] != BLANK) continue;
        std::queue<Index> Q;
        Q.push(i);
        colors[i] = curr_color;
        while (!Q.empty()) {
            Index curr_fid = Q.front();
            Q.pop();
            for (Index j = 0; j < vertex_per_facet; j++) {
                Index ei = get_edge(curr_fid, j);
                if (is_nonmanifold_edge(ei)) continue;

                foreach_facets_around_edge(ei, [&](Index adj_fid) {
                    if (colors[adj_fid] == BLANK) {
                        colors[adj_fid] = curr_color;
                        Q.push(adj_fid);
                    }
                });
            }
        }
        curr_color++;
    }

    Index vertex_count = num_vertices;
    // Note:
    // The goal of vertex_map is to split the 1-ring neighborhood of a
    // non-manifold vertex based on the colors of its adjacent facets.  All
    // adjacent facets sharing the sanme color will share the same copy of
    // this vertex.
    //
    // To achieve this, I store a color map for each non-manifold vertex,
    // where vertex_map maps a non-manifold vertex to its color map.
    // A color map maps specific color to the vertex index after the split.
    std::unordered_map<Index, std::unordered_map<Index, Index>> vertex_map;
    // Split non-manifold edges.
    for (Index i = 0; i < get_num_edges(); ++i) {
        const auto e = get_edge_vertices(i);

        if (!is_nonmanifold_edge(i)) continue;
        auto itr0 = vertex_map.find(e[0]);
        auto itr1 = vertex_map.find(e[1]);

        if (itr0 == vertex_map.end()) {
            itr0 = vertex_map.insert({e[0], std::unordered_map<Index, Index>()}).first;
        }
        if (itr1 == vertex_map.end()) {
            itr1 = vertex_map.insert({e[1], std::unordered_map<Index, Index>()}).first;
        }

        auto& color_map_0 = itr0->second;
        auto& color_map_1 = itr1->second;

        std::unordered_map<Index, std::list<Index>> color_count;
        foreach_facets_around_edge(i, [&](Index fid) {
            const auto c = colors[fid];

            auto c_itr = color_count.find(c);
            if (c_itr == color_count.end()) {
                color_count.insert({c, {fid}});
            } else {
                c_itr->second.push_back(fid);
            }

            auto c_itr0 = color_map_0.find(c);
            if (c_itr0 == color_map_0.end()) {
                if (color_map_0.size() == 0) {
                    color_map_0.insert({c, e[0]});
                } else {
                    color_map_0.insert({c, vertex_count});
                    vertex_count++;
                }
            }

            auto c_itr1 = color_map_1.find(c);
            if (c_itr1 == color_map_1.end()) {
                if (color_map_1.size() == 0) {
                    color_map_1.insert({c, e[1]});
                } else {
                    color_map_1.insert({c, vertex_count});
                    vertex_count++;
                }
            }
        });

        for (const auto& entry : color_count) {
            // Corner case 1:
            // Exact two facets share the same color around this edge, but
            // they are inconsistently oriented.  Thus, they needs to be
            // detacted.
            const bool inconsistent_edge =
                (entry.second.size() == 2) &&
                is_inconsistently_oriented_wrt_facets(i, entry.second.front(), entry.second.back());

            // Corner case 2:
            // Some facets around this non-manifold edge are connected via
            // a chain of manifold edges.  Thus, they have the same color.
            // To resolve this, I am detacching all facets of this color
            // adjacent to this edge.
            const bool single_comp_nonmanifoldness = entry.second.size() > 2;

            if (single_comp_nonmanifoldness || inconsistent_edge) {
                // Each facet will be reconnect to a newly created edge.
                // This solution is not ideal, but works.
                for (const auto fid : entry.second) {
                    colors[fid] = curr_color;
                    color_map_0.insert({curr_color, vertex_count});
                    vertex_count++;
                    color_map_1.insert({curr_color, vertex_count});
                    vertex_count++;
                    curr_color++;
                }
            }
        }
    }

    // Split non-manifold vertices
    for (Index i = 0; i < num_vertices; i++) {
        const auto& adj_facets = get_facets_adjacent_to_vertex(i);
        std::set<size_t> adj_colors;
        for (auto adj_f : adj_facets) {
            adj_colors.insert(colors[adj_f]);
        }
        if (adj_colors.size() <= 1) continue;

        auto itr = vertex_map.find(i);
        if (itr == vertex_map.end()) {
            vertex_map.insert({i, {}});
        }

        auto& vi_map = vertex_map[i];
        for (auto c : adj_colors) {
            auto c_itr = vi_map.find(c);
            if (c_itr == vi_map.end()) {
                if (vi_map.size() == 0) {
                    vi_map.insert({c, i});
                } else {
                    vi_map.insert({c, vertex_count});
                    vertex_count++;
                }
            }
        }
    }

    Eigen::MatrixXd manifold_vertices(vertex_count, 3);
    manifold_vertices.topRows(num_vertices) = vertices;

    std::vector<Index> backward_vertex_map(vertex_count, 0);
    std::iota(backward_vertex_map.begin(), backward_vertex_map.begin() + num_vertices, 0);

    for (const auto& itr : vertex_map) {
        Index vid = itr.first;
        for (const auto& itr2 : itr.second) {
            manifold_vertices.row(itr2.second) = vertices.row(vid);
            backward_vertex_map[itr2.second] = vid;
        }
    }

    auto manifold_facets = facets;
    for (Index i = 0; i < num_facets; i++) {
        const Index c = colors[i];
        for (Index j = 0; j < vertex_per_facet; j++) {
            const auto& itr = vertex_map.find(manifold_facets(i, j));
            if (itr != vertex_map.end()) {
                manifold_facets(i, j) = itr->second[c];
            }
        }
    }

    V = manifold_vertices;
    F = manifold_facets;
    modified_vertices.clear();
    for (auto& itr : vertex_map) {
        modified_vertices.push_back(itr.first);
        for (auto itr2 : itr.second) modified_vertices.push_back(itr2.second);
    }
    vector_unique(modified_vertices);
}


bool wmtk::separate_to_manifold(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    std::vector<Eigen::Vector3d>& out_v,
    std::vector<std::array<size_t, 3>>& out_f,
    std::vector<size_t>& mod_v)
{
    Eigen::MatrixXi F(faces.size(), 3);
    for (auto i = 0; i < faces.size(); i++) F.row(i) << faces[i][0], faces[i][1], faces[i][2];
    Eigen::MatrixXd V(vertices.size(), 3);
    for (auto i = 0; i < vertices.size(); i++) V.row(i) = vertices[i];

    resolve_nonmanifoldness(V, F, mod_v);
    Eigen::MatrixXd NV;
    Eigen::MatrixXi NF;
    Eigen::VectorXi I, J;
    igl::remove_unreferenced(V, F, NV, NF, I, J);
    for (auto& v : mod_v) v = I[v];
    out_v.resize(V.rows());
    out_f.resize(F.rows());
    for (auto i = 0; i < V.rows(); i++) out_v[i] = V.row(i);
    for (auto i = 0; i < F.rows(); i++)
        out_f[i] =
            std::array<size_t, 3>{{(size_t)faces[i][0], (size_t)faces[i][1], (size_t)faces[i][2]}};
    return true;
}
