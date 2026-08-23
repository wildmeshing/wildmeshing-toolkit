#pragma once
#include <igl/AABB.h>
#include <igl/in_element.h>
#include <Eigen/Dense>
#include <SimpleBVH/BVH.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components::topological_offset {


class SimplicialComplexBVH
{
private:
    // tet bvh
    igl::AABB<MatrixXd, 3> m_tet_aabb_tree;
    bool m_has_tets = false;

    // triangle bvh
    SimpleBVH::BVH m_tri_bvh;
    bool m_has_tris = false;

    // edge bvh
    SimpleBVH::BVH m_edge_bvh;
    bool m_has_edges = false;

    bool m_is_3d;
    MatrixXd m_V_T = MatrixXd(0, 3); // tet vertices
    MatrixXi m_T_T = MatrixXi(0, 4); // tets w.r.t. V_T vertices
    // igl::AABB<MatrixXd, 3> m_tet_aabb_tree;

    /// Input copies backing nearest_point_feature (2D only): the BVH's nearest_facet returns
    /// its INTERNAL primitive index, so classifying the hit needs the input arrays at hand.
    /// m_e2 is in the edge BVH's own primitive order -- real edges first, then the pseudo-edges
    /// (i, i) that carry isolated vertices -- because intersect_box reports ids in that order.
    std::vector<Eigen::Vector2d> m_v2;
    std::vector<Eigen::Vector2i> m_e2;

public:
    /**
     * @brief initialize BVH from "closed" simplicial complex
     * @param V: Nx2 or Nx3, all vertices contained anywhere in complex
     * @param T: Mx4, tets contained in complex
     * @param F: Lx3, isolated faces contained in complex (ie, faces not in any tet)
     * @param E: Kx2, the complex's EDGE SET: isolated edges (edges not in any faces or tets),
     * and -- when the caller wants the 2D feature query to answer for a solid complex -- the
     * boundary segments of F as well. Boundary segments lie on the faces, so including them
     * leaves squared_dist() unchanged; nearest_point_feature() searches exactly this set.
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
        MatrixXd V_3d; // if V 2d, pad vertices to 3d (BVH needs this. should be fixed inside BVH)
        if (!m_is_3d) {
            V_3d.resize(V.rows(), 3);
            V_3d.setZero();
            V_3d.block(0, 0, V.rows(), V.cols()) = V;
        } else {
            V_3d = V;
        }
        assert(V_3d.rows() == V.rows());
        assert(V_3d.cols() == 3);

        // extract isolated faces
        std::vector<Vector3i> faces;
        for (int i = 0; i < F.rows(); i++) {
            faces.push_back(F.row(i));
        }

        // complex is 3d and has tets
        if (m_is_3d && T.rows() > 0) {
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

            m_V_T = V;
            m_T_T = T;
            // m_T_T.col(2).swap(m_T_T.col(3));
            m_has_tets = true;
            m_tet_aabb_tree.init(m_V_T, m_T_T);
        }

        // initialize triangle bvh
        if (!faces.empty()) {
            MatrixXi F_combo(faces.size(), 3);
            for (size_t i = 0; i < faces.size(); i++) {
                F_combo.row(i) = faces[i];
            }
            m_tri_bvh.init(V_3d, F_combo, 1e-6);
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
            m_edge_bvh.init(V_3d, E_combo, 1e-6);
            m_has_edges = true;
        }

        // The feature query's backing copies; see the members. 2D only -- the 3D feature query
        // lives on the 3D input-complex envelope, which 3D still uses.
        m_v2.clear();
        m_e2.clear();
        if (!m_is_3d) {
            m_v2.reserve(size_t(V.rows()));
            for (int i = 0; i < V.rows(); i++) {
                m_v2.emplace_back(V(i, 0), V(i, 1));
            }
            m_e2.assign(edges.begin(), edges.end());
        }
    }

    /**
     * @brief check if a point is inside any tet.
     */
    bool inside_any_tet(const Vector3d& p) const
    {
        // 2D or no tets
        if (!m_has_tets) {
            return false;
        }

        Eigen::MatrixXd Q(1, 3);
        Q.row(0) = p;
        Eigen::VectorXi I;
        igl::in_element(m_V_T, m_T_T, Q, m_tet_aabb_tree, I);
        return (I(0) != -1);
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
        if (m_has_tets) { // has any tets
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

    /**
     * @brief find the nearest point on the complex (triangles and edges) to p
     * @note unlike squared_dist(), this does not special-case points inside a tet of the
     * complex -- for those the returned point is still the nearest surface point, not p itself
     */
    Vector3d nearest_point(const VectorXd& p) const
    {
        // pad to 3d if necessary
        Vector3d p3;
        if (p.size() == 2) {
            p3 << p(0), p(1), 0.0;
        } else {
            p3 = p;
        }

        Vector3d best_p = p3;
        double best_sq_dist = std::numeric_limits<double>::max();

        Vector3d closest_p;
        double tmp_sq_dist;
        if (m_has_tris) {
            m_tri_bvh.nearest_facet(p3, closest_p, tmp_sq_dist);
            if (tmp_sq_dist < best_sq_dist) {
                best_sq_dist = tmp_sq_dist;
                best_p = closest_p;
            }
        }
        if (m_has_edges) {
            m_edge_bvh.nearest_facet(p3, closest_p, tmp_sq_dist);
            if (tmp_sq_dist < best_sq_dist) {
                best_sq_dist = tmp_sq_dist;
                best_p = closest_p;
            }
        }
        return best_p;
    }

    /**
     * @brief 2D: the nearest point of the complex together with WHICH feature it is.
     *
     * The exact same algorithm SampleEnvelope::nearest_point_feature runs (Envelope.cpp), here
     * so the euclidean potential can query the ONE retained input-complex structure instead of
     * a second envelope over the same segments. nearest_facet seeds the search; the foot is
     * then RE-DERIVED on every box candidate from the input arrays, so classification and the
     * returned point come from the same arithmetic (the BVH's own foot can differ in the last
     * ulp). A hit at t <= 0 or t >= 1 is the endpoint itself: on_corner, with the POLYLINE
     * vertex index as the feature id, canonical across the two segments sharing it. A pseudo-
     * edge (i, i) has len2 == 0 and lands on the corner branch, which is what the geometry is.
     *
     * @return squared distance to the returned foot point.
     */
    double nearest_point_feature(
        const Eigen::Vector2d& p,
        Eigen::Vector2d& result,
        bool& on_corner,
        Eigen::Vector2d& seg_normal,
        int& feature_id) const
    {
        // HARD CHECKS, not asserts: SimpleBVH's nearest_facet walks boxlist[2] unconditionally,
        // so querying an empty tree is a segfault, and a candidate id past m_e2 is an OOB read.
        // Release builds compile asserts out; a thrown message names the broken invariant.
        if (m_is_3d || !m_has_edges || m_v2.empty() || m_e2.empty()) {
            log_and_throw_error(
                "SimplicialComplexBVH::nearest_point_feature: 2D {} | edges {} | copies {}/{}",
                !m_is_3d,
                m_has_edges,
                m_v2.size(),
                m_e2.size());
        }

        const Eigen::Vector3d p3(p[0], p[1], 0.0);
        Eigen::Vector3d nearest;
        double sq_dist;
        m_edge_bvh.nearest_facet(p3, nearest, sq_dist);

        std::vector<unsigned int> candidates;
        const double pad = 1e-9 + 1e-9 * std::sqrt(sq_dist);
        // The BVH stores 2D data lifted to z = 0; query with a 3D box spanning it.
        const Eigen::Vector3d lo(nearest[0] - pad, nearest[1] - pad, -pad);
        const Eigen::Vector3d hi(nearest[0] + pad, nearest[1] + pad, pad);
        m_edge_bvh.intersect_box(lo, hi, candidates);
        if (candidates.empty()) {
            log_and_throw_error(
                "SimplicialComplexBVH::nearest_point_feature: no box candidate at the foot "
                "point -- the BVH and the input copies disagree");
        }

        double best = std::numeric_limits<double>::max();
        for (const unsigned int fid : candidates) {
            if (fid >= m_e2.size()) {
                log_and_throw_error(
                    "SimplicialComplexBVH::nearest_point_feature: candidate id {} out of range "
                    "({} edges)",
                    fid,
                    m_e2.size());
            }
            const Eigen::Vector2d a = m_v2[size_t(m_e2[fid][0])];
            const Eigen::Vector2d b = m_v2[size_t(m_e2[fid][1])];
            const Eigen::Vector2d ab = b - a;
            const double len2 = ab.squaredNorm();
            const double t = len2 > 0 ? (p - a).dot(ab) / len2 : 0.0;
            Eigen::Vector2d foot;
            bool corner;
            int id;
            if (len2 <= 0 || t <= 0) {
                foot = a;
                corner = true;
                id = m_e2[fid][0];
            } else if (t >= 1) {
                foot = b;
                corner = true;
                id = m_e2[fid][1];
            } else {
                foot = a + t * ab;
                corner = false;
                id = int(fid);
            }
            const double d2 = (p - foot).squaredNorm();
            if (d2 < best) {
                best = d2;
                result = foot;
                on_corner = corner;
                feature_id = id;
                if (!corner) {
                    seg_normal = Eigen::Vector2d(-ab[1], ab[0]) / std::sqrt(len2);
                }
            }
        }
        return best;
    }

    void clear()
    {
        m_tri_bvh.clear();
        m_has_tris = false;
        m_edge_bvh.clear();
        m_has_edges = false;
        m_v2.clear();
        m_e2.clear();
        m_tet_aabb_tree = igl::AABB<MatrixXd, 3>(); // reset
        m_has_tets = false;
        m_V_T.resize(0, 3);
        m_T_T.resize(0, 4);
    }
};


} // namespace wmtk::components::topological_offset