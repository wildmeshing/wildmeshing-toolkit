#pragma once

#include <Eigen/Core>
#include <memory>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <fastenvelope/FastEnvelope2D.h>
#include <SimpleBVH/BVH.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

namespace wmtk {
class Envelope
{
public:
    virtual ~Envelope() = default;
    virtual void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double) {};
    virtual bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const { return false; };
    virtual bool is_outside(const Eigen::Vector3d& pts) const { return false; };
};

class ExactEnvelope : public Envelope, public fastEnvelope::FastEnvelope
{
public:
    ExactEnvelope()
        : fastEnvelope::FastEnvelope::FastEnvelope() {};
    ~ExactEnvelope() {};
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double eps)
    {
        fastEnvelope::FastEnvelope::init(m_ver, m_faces, eps);
    }
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const
    {
        return fastEnvelope::FastEnvelope::is_outside(tris);
    }
    bool is_outside(const Eigen::Vector3d& pts) const
    {
        return fastEnvelope::FastEnvelope::is_outside(pts);
    }
};


class SampleEnvelope : public wmtk::Envelope
{
public:
    /**
     * Which of the three init() overloads built this envelope.
     *
     * The sampled path does not need it -- every overload ends up in the same BVH, and a 2D
     * query is answered by lifting to z = 0. The exact path does: a triangle-built and an
     * edge-built FastEnvelope are different objects, a 2D envelope is a different class
     * again, and only init() knows which one was filled in. Without this, is_outside(Vector2d)
     * would lift to 3D and query a FastEnvelope that was never initialized, which answers
     * "outside" for everything rather than failing.
     */
    enum class Kind { Uninitialized, Triangles3d, Edges3d, Edges2d };

    SampleEnvelope(bool exact = false)
        : use_exact(exact) {};
    /**
     * Per-sample acceptance radius, squared, for POINT and TRIANGLE queries.
     *
     * eps shrunk by the covering radius of sampleTriangle's lattice, sampling_dist/sqrt(3).
     * A point query needs no shrink at all and is therefore merely conservative here; it is
     * left as it is because every 3D operation goes through it and widening it would change
     * tetwild's behaviour everywhere, which belongs in its own change.
     */
    double eps2 = 1e-6;
    /**
     * The same, for EDGE queries, where the covering radius is different.
     *
     * is_outside(edge) samples the segment at N+1 evenly spaced points with
     * N = floor(L/sampling_dist) + 1 > L/sampling_dist, so the spacing is L/N < sampling_dist
     * and every point of the segment lies within sampling_dist/2 of a sample. Distance to a
     * set is 1-Lipschitz, so "every sample within r" implies "every point within
     * r + sampling_dist/2"; for that to imply containment in eps,
     *
     *     r = eps - sampling_dist / 2      (= eps/2, since sampling_dist = eps)
     *
     * The lattice constant eps - sampling_dist/sqrt(3) belongs to sampleTriangle and is
     * simply the wrong figure here: it is 0.4226*eps against the correct 0.5*eps, so using
     * it made the edge test 18% stricter than the geometry requires. That is safe on its own
     * -- but it also made the simplification's own guarantee (r_s + sampling_dist_s/2 = eps_s)
     * exceed the acceptance threshold of the coarser envelope the optimizer checks against,
     * so a correctly simplified input could be rejected by the init sanity check. It was,
     * once in 15665 models.
     */
    double eps2_edge = 1e-6;
    double sampling_dist = 1e-3;
    bool use_exact = false;

    /**
     * Debug escape hatch: when set, every is_outside() answers "inside".
     *
     * The envelope is the only thing that can reject an operation for geometric rather
     * than combinatorial reasons, so turning it off tells you whether a stalled
     * optimization is blocked by the envelope or by the mesh itself. It removes the
     * containment guarantee entirely -- the output is not usable, only diagnostic.
     *
     * Note nearest_point() is unaffected, so EnvelopeEnergy still pulls surface vertices
     * toward the input while smoothing. Only the veto goes away, which is the point: it
     * separates "the optimizer cannot move because the envelope forbids it" from "the
     * optimizer cannot move at all".
     */
    bool disabled = false;
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double);
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector2i>& m_edges,
        const double);
    void init(
        const std::vector<Eigen::Vector2d>& m_ver,
        const std::vector<Eigen::Vector2i>& m_edges,
        const double);

    /**
     * @brief Same, from Eigen matrices, dispatching on the column counts.
     *
     *   V Nx3, F Mx3 -> a 3D triangle envelope
     *   V Nx3, F Mx2 -> a 3D edge envelope
     *   V Nx2, F Mx2 -> a 2D edge envelope
     *
     * Marshals into the vector form above. Most callers already hold matrices and were each
     * writing the same ten-line copy loop; this is that loop, once.
     */
    void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double eps);
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const;
    bool is_outside(const std::array<Eigen::Vector3d, 2>& edge) const;
    bool is_outside(const std::array<Eigen::Vector2d, 2>& edge) const;
    bool is_outside(const Eigen::Vector3d& pts) const;
    bool is_outside(const Eigen::Vector2d& pts) const;
    double nearest_point(const Eigen::Vector3d& pts, Eigen::Vector3d& result) const;
    double nearest_point(const Eigen::Vector2d& pts, Eigen::Vector2d& result) const;

    /**
     * 2D only: nearest point plus which feature of the input carries it. Returns the SQUARED
     * distance. on_corner reports whether the foot point is a polyline vertex; seg_normal is
     * the unit normal of the closest segment, valid only when !on_corner (sign arbitrary);
     * feature_id is the polyline vertex index when on_corner, else the segment index --
     * canonical, so two queries can be compared for "did the closest feature change".
     *
     * The distance to a piecewise-linear input is exactly quadratic within each closest-
     * feature region and kinks where the feature changes; anything that wants the true
     * second-order behavior (ExactDistanceEnergy2D) needs this classification.
     */
    double nearest_point_feature(
        const Eigen::Vector2d& p,
        Eigen::Vector2d& result,
        bool& on_corner,
        Eigen::Vector2d& seg_normal,
        int& feature_id) const;

    /**
     * 3D triangle-mesh counterpart. feature_dim reports where the foot point lies: 2 = face
     * interior, 1 = edge interior, 0 = mesh vertex. dir is the face normal (dim 2) or the
     * edge direction (dim 1), unit length, sign arbitrary; unused for dim 0. feature_id is
     * canonical across adjacent triangles: the face index (dim 2), the sorted vertex pair
     * packed as min * n_vertices + max (dim 1), or the vertex index (dim 0).
     */
    double nearest_point_feature(
        const Eigen::Vector3d& p,
        Eigen::Vector3d& result,
        int& feature_dim,
        Eigen::Vector3d& dir,
        long long& feature_id) const;
    bool initialized() { return m_bvh != nullptr; };

    Kind kind() const { return m_kind; }

    double squared_distance(const Eigen::Vector3d& p) const;
    double squared_distance(const Eigen::Vector2d& p) const;

private:
    void require_exact_kind(Kind expected, const char* query) const;
    void require_exact_3d(const char* query) const;
    void require_exact_built(const char* query) const;

    template <typename VertexList>
    void
    init_exact_edges(const VertexList& V, const std::vector<Eigen::Vector2i>& F, const double _eps);

    std::vector<int> geo_vertex_ind;
    std::vector<int> geo_face_ind;
    /// 2D input copy backing nearest_point_feature (filled by the 2D init only).
    std::vector<Eigen::Vector2d> m_v2;
    std::vector<Eigen::Vector2i> m_e2;
    /// 3D input copy backing nearest_point_feature (filled by the triangle init only).
    std::vector<Eigen::Vector3d> m_v3;
    std::vector<Eigen::Vector3i> m_f3;
    std::shared_ptr<SimpleBVH::BVH> m_bvh;

private:
    /// Serves both Triangles3d and Edges3d: FastEnvelope has an init() for each.
    fastEnvelope::FastEnvelope exact_envelope;
    /// Edges2d only. A separate class upstream, not an overload.
    fastEnvelope::FastEnvelope2D exact_envelope_2d;
    Kind m_kind = Kind::Uninitialized;
    /// Whether the edge/2D exact structure was actually built. Always false for Triangles3d,
    /// which builds its exact envelope unconditionally and is checked by kind alone.
    bool m_exact_built = false;
};
} // namespace wmtk