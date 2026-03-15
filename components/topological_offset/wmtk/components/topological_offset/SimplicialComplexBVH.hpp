#pragma once
#include <Eigen/Dense>
#include <SimpleBVH/BVH.hpp>


namespace wmtk::components::topological_offset {


class SimplicialComplexBVH
{
private:
    // triangle bvh
    SimpleBVH::BVH m_tri_bvh;
    bool m_has_tris;

    // edge bvh
    SimpleBVH::BVH m_edge_bvh;
    bool m_has_edges;

    bool m_is_3d;
    MatrixXd m_V_T = MatrixXd(0, 3); // tet vertices
    MatrixXi m_T_T = MatrixXi(0, 4); // tets w.r.t. V_T vertices

public:
    /**
     * @brief initialize BVH from "closed" simplicial complex
     * @param V: Nx2 or Nx3, all vertices contained anywhere in complex
     * @param T: Mx4, tets contained in complex
     * @param F: Lx3, isolated faces contained in complex (ie, faces not in any tet)
     * @param E: Kx2, isolated edges contained in complex (ie, edges not in any faces or tets)
     * @param P: Jx1, isolated vertices contained in complex (ie, verts not in any edges, faces, or
     * tets)
     */
    void init(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixXi& F,
        const MatrixXi& E,
        const MatrixXi& P)
    {
        m_is_3d = (V.cols() == 3);

        std::vector<Eigen::Vector3i> faces; // extract isolated faces
        for (int i = 0; i < F.rows(); i++) {
            faces.push_back(F.row(i));
        }

        // complex is 3d and has tets
        if (m_is_3d && T.rows() > 0) {
            m_V_T = V;
            m_T_T = T;
            for (int i = 0; i < T.rows(); i++) {
                int a = T(i, 0);
                int b = T(i, 1);
                int c = T(i, 2);
                int d = T(i, 3);
                faces.emplace_back(a, c, b);
                faces.emplace_back(a, b, d);
                faces.emplace_back(b, c, d);
                faces.emplace_back(a, d, c);
            }
        }

        // initialize triangle bvh
        if (!faces.empty()) {
            MatrixXi F_combo(faces.size(), 3);
            for (size_t i = 0; i < faces.size(); i++) {
                F_combo.row(i) = faces[i];
            }
            m_tri_bvh.init(V, F_combo, 1e-6);
            m_has_tris = true;
        }

        std::vector<Eigen::Vector2i> edges; // extract isolated edges
        for (int i = 0; i < E.rows(); i++) { // actual edges
            edges.push_back(E.row(i));
        }
        for (int i = 0; i < P.rows(); i++) { // pseudo edges
            edges.emplace_back(P(i), P(i));
        }

        if (!edges.empty()) {
            MatrixXi E_combo(edges.size(), 2);
            for (size_t i = 0; i < edges.size(); i++) {
                E_combo.row(i) = edges[i];
            }
            m_edge_bvh.init(V, E_combo, 1e-6);
            m_has_edges = true;
        }
    }

    /**
     * @brief check if a point is inside a given tet. This may not be robust
     */
    bool inside_tet(const Vector3d& p, const size_t tet_id) const
    {
        Vector3d v0 = m_V_T.row(m_T_T(tet_id, 0));
        Vector3d v1 = m_V_T.row(m_T_T(tet_id, 1));
        Vector3d v2 = m_V_T.row(m_T_T(tet_id, 2));
        Vector3d v3 = m_V_T.row(m_T_T(tet_id, 3));

        // fast reject
        double min_x = std::min({v0.x(), v1.x(), v2.x(), v3.x()});
        double max_x = std::max({v0.x(), v1.x(), v2.x(), v3.x()});
        if (p.x() < min_x || p.x() > max_x) {
            return false;
        }
        double min_y = std::min({v0.y(), v1.y(), v2.y(), v3.y()});
        double max_y = std::max({v0.y(), v1.y(), v2.y(), v3.y()});
        if (p.y() < min_y || p.y() > max_y) {
            return false;
        }
        double min_z = std::min({v0.z(), v1.z(), v2.z(), v3.z()});
        double max_z = std::max({v0.z(), v1.z(), v2.z(), v3.z()});
        if (p.z() < min_z || p.z() > max_z) {
            return false;
        }

        // fast reject failed, actually compute
        Eigen::Matrix3d Basis;
        Basis.col(0) = v0 - v3;
        Basis.col(1) = v1 - v3;
        Basis.col(2) = v2 - v3;

        Eigen::Vector3d rhs = p - v3;
        Eigen::Vector3d uvw = Basis.partialPivLu().solve(rhs);

        const double eps = 1e-6;
        return (uvw(0) >= -eps) && (uvw(1) >= -eps) && (uvw(2) >= -eps) && (uvw.sum() <= 1 + eps);
    }

    /**
     * @brief check if a point is inside any tet. NOTE: this can be sped up with a true tetmesh bvh
     */
    bool inside_any_tet(const Vector3d& p) const
    {
        // 2D or no tets
        if (!m_is_3d || m_T_T.rows() == 0) {
            return false;
        }

        for (int i = 0; i < m_T_T.rows(); i++) {
            if (inside_tet(p, i)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief compute distance to complex
     */
    double squared_dist(const VectorXd& p) const
    {
        double min_sq_dist = std::numeric_limits<double>::max();

        // pad to 3d if necessary
        Vector3d p3;
        if (p.size() == 2) {
            p3 << p(0), p(1), 0.0;
        } else {
            p3 = p;
        }

        // inside tet check
        if (m_is_3d && m_T_T.rows() > 0) { // has any tets
            if (inside_any_tet(p3)) {
                return 0.0;
            }
        }

        Vector3d closest_p;
        double tmp_sq_dist;

        if (m_has_tris) { // min dist to isolated triangles (and tet faces)
            m_tri_bvh.nearest_facet(p3, closest_p, min_sq_dist);
        }

        if (m_has_edges) { // min dist to isolated edges (and 'pseudo'edges, ie isolated vertices)
            m_edge_bvh.nearest_facet(p3, closest_p, tmp_sq_dist);
            if (tmp_sq_dist < min_sq_dist) {
                min_sq_dist = tmp_sq_dist;
            }
        }

        return min_sq_dist;
    }

    double dist(const VectorXd& p) const { return sqrt(squared_dist(p)); }

    void clear()
    {
        m_tri_bvh.clear();
        m_has_tris = false;
        m_edge_bvh.clear();
        m_has_edges = false;
        m_V_T.resize(0, 3);
        m_T_T.resize(0, 4);
    }
};


} // namespace wmtk::components::topological_offset