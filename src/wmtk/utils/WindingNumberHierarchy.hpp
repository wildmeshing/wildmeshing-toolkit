// Adapted from libigl's WindingNumberTree and WindingNumberAABB.
//
// Copyright (C) 2014 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#pragma once

#include <Eigen/Core>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/barycenter.h>
#include <igl/exterior_edges.h>
#include <igl/median.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/winding_number.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <limits>
#include <vector>

namespace wmtk::utils {

/**
 * @brief The hierarchy libigl's winding_number(V, F, O, W) evaluates a triangle mesh with --
 * igl::WindingNumberAABB, exact method -- made deterministic and self-contained.
 *
 * The algorithm is igl's, step for step: exact duplicate vertices merged, nodes split at the
 * median face barycenter along the box's longest axis down to 100 faces, and a query outside a
 * node's box answered from the node's boundary, closed by a fan, when that is cheaper than its
 * faces. Two things differ:
 *
 * - The fan's apex. igl picks it with rand(), so every build of the same mesh closes its nodes
 *   differently. The values agree mathematically -- the cap lies in the node's box and is only
 *   used for points outside it -- but not to the last bit: two builds of one mesh disagreed by
 *   an ulp on a quarter of the points of a test sphere. The winding number then depended on the
 *   process's rand() state, and a point on the surface, at 1/2, could fall either side of the
 *   inside test from run to run. Here the apex is the first boundary edge's first vertex.
 * - The vertices. igl's tree keeps them in a static shared by every tree of its type, so
 *   building one tree while another is queried corrupts the other's results. Here each
 *   hierarchy owns them.
 *
 * winding_number(p) is const and touches no shared state, so a hierarchy can be queried from
 * any number of threads at once.
 */
class WindingNumberHierarchy
{
public:
    WindingNumberHierarchy(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
    {
        Eigen::MatrixXi SVI, SVJ, SF;
        igl::remove_duplicate_vertices(V, F, 0.0, m_V, SVI, SVJ, SF);
        add_node(std::move(SF));
        grow(0);
    }

    /// Winding number of p with respect to the mesh.
    double winding_number(const Eigen::RowVector3d& p) const { return node_winding_number(0, p); }

private:
    /// Fewest faces a node is split at, as igl's WindingNumberAABB_MIN_F.
    static constexpr Eigen::Index kMinFaces = 100;

    struct Node
    {
        Eigen::MatrixXi F; // faces, into m_V
        Eigen::MatrixXi cap; // the fan closing F's boundary
        Eigen::RowVector3d min_corner, max_corner; // the box of F's vertices
        int left = -1, right = -1;
    };

    /// igl::triangle_fan(igl::exterior_edges(F)), with a fixed apex instead of a random one.
    static Eigen::MatrixXi cap_of(const Eigen::MatrixXi& F)
    {
        const Eigen::MatrixXi E = igl::exterior_edges(F);
        if (E.size() == 0) return Eigen::MatrixXi(0, 3);
        const int s = E(0, 0);
        std::vector<Eigen::Index> rows;
        for (Eigen::Index i = 0; i < E.rows(); ++i) {
            if (E(i, 0) != s && E(i, 1) != s) rows.push_back(i);
        }
        Eigen::MatrixXi cap(rows.size(), 3);
        for (size_t k = 0; k < rows.size(); ++k) {
            cap.row(k) << s, E(rows[k], 0), E(rows[k], 1);
        }
        return cap;
    }

    int add_node(Eigen::MatrixXi F)
    {
        Node n;
        n.cap = cap_of(F);
        n.min_corner.setConstant(std::numeric_limits<double>::infinity());
        n.max_corner.setConstant(-std::numeric_limits<double>::infinity());
        for (Eigen::Index i = 0; i < F.rows(); ++i) {
            for (Eigen::Index j = 0; j < F.cols(); ++j) {
                n.min_corner = n.min_corner.cwiseMin(m_V.row(F(i, j)));
                n.max_corner = n.max_corner.cwiseMax(m_V.row(F(i, j)));
            }
        }
        n.F = std::move(F);
        m_nodes.push_back(std::move(n));
        return int(m_nodes.size()) - 1;
    }

    /// WindingNumberAABB::grow with the median split.
    void grow(const int node)
    {
        // Indices, not references: add_node below may reallocate m_nodes.
        const Eigen::Index n_faces = m_nodes[node].F.rows();
        if (n_faces <= kMinFaces || m_nodes[node].cap.rows() - 2 >= n_faces) return;

        int axis = -1;
        double longest = -std::numeric_limits<double>::infinity();
        for (int d = 0; d < 3; ++d) {
            const double len = m_nodes[node].max_corner[d] - m_nodes[node].min_corner[d];
            if (len > longest) {
                longest = len;
                axis = d;
            }
        }
        Eigen::MatrixXd BC;
        igl::barycenter(m_V, m_nodes[node].F, BC);
        double split_value;
        igl::median(BC.col(axis), split_value);

        std::vector<Eigen::Index> left, right;
        for (Eigen::Index i = 0; i < n_faces; ++i) {
            (BC(i, axis) <= split_value ? left : right).push_back(i);
        }
        if (left.empty() || right.empty()) return;
        const auto rows_of = [&](const std::vector<Eigen::Index>& rows) {
            Eigen::MatrixXi F(rows.size(), 3);
            for (size_t k = 0; k < rows.size(); ++k) F.row(k) = m_nodes[node].F.row(rows[k]);
            return F;
        };
        const int l = add_node(rows_of(left));
        m_nodes[node].left = l;
        grow(l);
        const int r = add_node(rows_of(right));
        m_nodes[node].right = r;
        grow(r);
    }

    bool inside(const Node& n, const Eigen::RowVector3d& p) const
    {
        // Conservative, as igl's: a point on the box is inside.
        for (int d = 0; d < 3; ++d) {
            if (p[d] < n.min_corner[d] || p[d] > n.max_corner[d]) return false;
        }
        return true;
    }

    /// WindingNumberTree::winding_number, exact method.
    double node_winding_number(const int node, const Eigen::RowVector3d& p) const
    {
        const Node& n = m_nodes[node];
        if (inside(n, p)) {
            if (n.left >= 0) {
                double sum = 0;
                sum += node_winding_number(n.left, p);
                sum += node_winding_number(n.right, p);
                return sum;
            }
            return igl::winding_number(m_V, n.F, p);
        }
        // Outside the box the boundary decides, when it is the cheaper of the two.
        if (n.cap.rows() - 2 < n.F.rows()) return igl::winding_number(m_V, n.cap, p);
        return igl::winding_number(m_V, n.F, p);
    }

    Eigen::MatrixXd m_V; // the mesh's vertices, exact duplicates merged
    std::vector<Node> m_nodes; // m_nodes[0] is the root
};

} // namespace wmtk::utils
