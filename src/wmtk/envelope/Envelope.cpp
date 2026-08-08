#include "Envelope.hpp"

#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {

// From TetWild
void sampleTriangle(
    const std::array<Vector3d, 3>& vs,
    std::vector<Vector3d>& ps,
    const double sampling_dist)
{
    double sqrt3_2 = std::sqrt(3) / 2;

    std::array<double, 3> ls;
    for (int i = 0; i < 3; i++) {
        ls[i] = (vs[i] - vs[(i + 1) % 3]).squaredNorm();
    }
    auto min_max = std::minmax_element(ls.begin(), ls.end());
    int min_i = min_max.first - ls.begin();
    int max_i = min_max.second - ls.begin();
    double N = sqrt(ls[max_i]) / sampling_dist;
    if (N <= 1) {
        for (int i = 0; i < 3; i++) ps.push_back(vs[i]);
        return;
    }
    if (N == int(N)) N -= 1;

    Vector3d v0 = vs[max_i];
    Vector3d v1 = vs[(max_i + 1) % 3];
    Vector3d v2 = vs[(max_i + 2) % 3];

    Vector3d n_v0v1 = (v1 - v0).normalized();
    for (int n = 0; n <= N; n++) {
        ps.push_back(v0 + n_v0v1 * sampling_dist * n);
    }
    ps.push_back(v1);

    double h = (((v2 - v0).dot(v1 - v0) * (v1 - v0) / ls[max_i] + v0) - v2).norm();
    int M = h / (sqrt3_2 * sampling_dist);
    if (M < 1) {
        ps.push_back(v2);
        return;
    }

    Vector3d n_v0v2 = (v2 - v0).normalized();
    Vector3d n_v1v2 = (v2 - v1).normalized();
    auto sin_v0 = (((v2 - v0).cross(v1 - v0))).norm() / ((v0 - v2).norm() * (v0 - v1).norm());
    auto tan_v0 = (((v2 - v0).cross(v1 - v0))).norm() / ((v2 - v0).dot(v1 - v0));
    auto tan_v1 = (((v2 - v1).cross(v0 - v1))).norm() / ((v2 - v1).dot(v0 - v1));
    auto sin_v1 = (((v2 - v1).cross(v0 - v1))).norm() / ((v1 - v2).norm() * (v0 - v1).norm());

    for (int m = 1; m <= M; m++) {
        int n = sqrt3_2 / tan_v0 * m + 0.5;
        int n1 = sqrt3_2 / tan_v0 * m;
        if (m % 2 == 0 && n == n1) {
            n += 1;
        }
        Vector3d v0_m = v0 + m * sqrt3_2 * sampling_dist / sin_v0 * n_v0v2;
        Vector3d v1_m = v1 + m * sqrt3_2 * sampling_dist / sin_v1 * n_v1v2;
        if ((v0_m - v1_m).norm() <= sampling_dist) break;

        double delta_d = ((n + (m % 2) / 2.0) - m * sqrt3_2 / tan_v0) * sampling_dist;
        Vector3d v = v0_m + delta_d * n_v0v1;
        int N1 = (v - v1_m).norm() / sampling_dist;
        for (int i = 0; i <= N1; i++) {
            ps.push_back(v + i * n_v0v1 * sampling_dist);
        }
    }
    ps.push_back(v2);

    // sample edges
    N = sqrt(ls[(max_i + 1) % 3]) / sampling_dist;
    if (N > 1) {
        if (N == int(N)) N -= 1;
        Vector3d n_v1v2 = (v2 - v1).normalized();
        for (int n = 1; n <= N; n++) {
            ps.push_back(v1 + n_v1v2 * sampling_dist * n);
        }
    }

    N = sqrt(ls[(max_i + 2) % 3]) / sampling_dist;
    if (N > 1) {
        if (N == int(N)) N -= 1;
        Vector3d n_v2v0 = (v0 - v2).normalized();
        for (int n = 1; n <= N; n++) {
            ps.push_back(v2 + n_v2v0 * sampling_dist * n);
        }
    }
}

/**
 * Build the exact edge envelope, unless there is no envelope to build.
 *
 * A zero epsilon is a legitimate way to ask for an envelope that will only ever be used for
 * nearest_point() -- triwild's Delaunay seeding does it, to reject grid points that fall too
 * close to the input curves. There is no such thing as a zero-width exact envelope: the 2D
 * construction asserts on it, and a containment query against one would answer "outside" for
 * everything not exactly on the input. So skip the build, and make the combination that would
 * actually consult it fail here rather than silently reject every query.
 */
template <typename VertexList>
void SampleEnvelope::init_exact_edges(
    const VertexList& V,
    const std::vector<Eigen::Vector2i>& F,
    const double _eps)
{
    if (!(_eps > 0)) {
        if (use_exact) {
            log_and_throw_error("An exact envelope needs a positive epsilon, got {}.", _eps);
        }
        return;
    }

    if constexpr (std::is_same_v<VertexList, std::vector<Eigen::Vector2d>>) {
        exact_envelope_2d.init(V, F, _eps);
    } else {
        exact_envelope.init(V, F, _eps);
    }
}

void SampleEnvelope::init(
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<Eigen::Vector3i>& F,
    const double _eps)
{
    if (!use_exact) {
        logger().info("Using sample envelope.");
    }
    m_kind = Kind::Triangles3d;
    exact_envelope.init(V, F, _eps);

    // sampleTriangle lays samples on a triangular lattice of spacing `sampling_dist`, whose
    // covering radius is sampling_dist / sqrt(3): every point of the triangle is within that
    // distance of some sample. So for "every sample is inside" to imply "the whole triangle
    // is inside the true eps-envelope", the per-sample test has to use the shrunk radius
    //
    //     real_envelope = _eps - sampling_dist / sqrt(3)
    //
    // Testing against the unshrunk _eps made this anti-conservative: a triangle straying up
    // to _eps * (1 + 1/sqrt(3)) ~= 1.577 * _eps outside the input could still pass every
    // sample and be accepted.
    //
    // This is what both reference implementations do. TetWild, src/tetwild/State.cpp:
    //     sampling_dist = eps_input / args.stage;
    //     eps = eps_input - sampling_dist / std::sqrt(3) * (args.stage + 1 - sub_stage);
    // and fTetWild, src/Parameters.h, on the sampled (non-NEW_ENVELOPE) path:
    //     dd = eps_input / 1.5;
    //     eps_usable = eps_input - dd / std::sqrt(3);
    // Neither shrinks on its exact-envelope path, because the shrink exists purely to pay
    // for the sampling. The two edge overloads below already did this; only the triangle
    // one did not.
    sampling_dist = _eps;
    const double real_envelope = _eps - _eps / std::sqrt(3);
    eps2 = real_envelope * real_envelope;
    // Edge queries sample a line, not a lattice; see eps2_edge in the header.
    const double real_envelope_edge = _eps - _eps / 2;
    eps2_edge = real_envelope_edge * real_envelope_edge;

    MatrixXd VV;
    VV.resize(V.size(), 3);
    for (size_t i = 0; i < V.size(); ++i) {
        VV.row(i) = V[i];
    }
    MatrixXi FF;
    FF.resize(F.size(), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        FF.row(i) = F[i];
    }

    m_bvh = std::make_shared<SimpleBVH::BVH>();
    m_bvh->init(VV, FF, 0);
}

void SampleEnvelope::init(
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<Eigen::Vector2i>& F,
    const double _eps)
{
    m_kind = Kind::Edges3d;
    // The raw _eps, not the shrunk one below: the shrink pays for the sampling lattice and
    // the exact envelope has no lattice. fast-envelope builds a hexahedron of half-width
    // eps/sqrt(3) around each segment, so its corners sit at exactly eps.
    //
    // Built whether or not use_exact is set, because callers flip the flag on an envelope
    // that is already initialized -- tetwild does exactly that around its simplification
    // (tetwild.cpp) -- so both structures have to be there. See init_exact_edges() for the
    // one case where it is skipped.
    init_exact_edges(V, F, _eps);

    sampling_dist = _eps;
    // eps2 keeps the lattice constant for point queries; eps2_edge carries the one derived
    // for a sampled segment. See the header.
    const double real_envelope = _eps - _eps / sqrt(3);
    eps2 = real_envelope * real_envelope;
    const double real_envelope_edge = _eps - _eps / 2;
    eps2_edge = real_envelope_edge * real_envelope_edge;

    MatrixXd VV;
    VV.resize(V.size(), 3);
    for (size_t i = 0; i < V.size(); ++i) {
        VV.row(i) = V[i];
    }
    MatrixXi FF;
    FF.resize(F.size(), 2);
    for (size_t i = 0; i < F.size(); ++i) {
        FF.row(i) = F[i];
    }

    m_bvh = std::make_shared<SimpleBVH::BVH>();
    m_bvh->init(VV, FF, 0);
}
void SampleEnvelope::init(
    const std::vector<Eigen::Vector2d>& V,
    const std::vector<Eigen::Vector2i>& F,
    const double _eps)
{
    m_kind = Kind::Edges2d;
    // Raw _eps again; see the 3D edge overload. In 2D the conservative shape is a rectangle
    // of half-width eps/sqrt(2) around each segment, extended by the same amount past both
    // ends, so its corners sit at exactly eps.
    init_exact_edges(V, F, _eps);

    sampling_dist = _eps;
    // eps2 keeps the lattice constant for point queries; eps2_edge carries the one derived
    // for a sampled segment. See the header.
    const double real_envelope = _eps - _eps / sqrt(3);
    eps2 = real_envelope * real_envelope;
    const double real_envelope_edge = _eps - _eps / 2;
    eps2_edge = real_envelope_edge * real_envelope_edge;

    MatrixXd VV;
    VV.resize(V.size(), 2);
    for (size_t i = 0; i < V.size(); ++i) {
        VV.row(i) = V[i];
    }
    MatrixXi FF;
    FF.resize(F.size(), 2);
    for (size_t i = 0; i < F.size(); ++i) {
        FF.row(i) = F[i];
    }

    m_bvh = std::make_shared<SimpleBVH::BVH>();
    m_bvh->init(VV, FF, 0);
}

/**
 * An exact query must be answered by the structure the matching init() built.
 *
 * Getting this wrong is silent and dangerous rather than noisy: a default-constructed
 * FastEnvelope has no prisms, and a query against no prisms answers "outside" for every
 * point -- so a mismatched envelope does not crash, it just vetoes every operation and
 * looks like an optimization that cannot move.
 */
void SampleEnvelope::require_exact_kind(Kind expected, const char* query) const
{
    if (m_kind != expected) {
        log_and_throw_error(
            "Exact {} query against an envelope built by a different init() (kind {}, need {}).",
            query,
            static_cast<int>(m_kind),
            static_cast<int>(expected));
    }
}

bool SampleEnvelope::is_outside(const Eigen::Vector3d& pts) const
{
    if (disabled) return false;
    if (use_exact) {
        // Points work against either 3D envelope, so both kinds are accepted here; only a 2D
        // one is wrong, and that is what the check catches.
        if (m_kind == Kind::Edges2d || m_kind == Kind::Uninitialized) {
            require_exact_kind(Kind::Triangles3d, "3D point");
        }
        return exact_envelope.is_outside(pts);
    }
    double dist2 = squared_distance(pts);

    return (dist2 > eps2);
}

bool SampleEnvelope::is_outside(const Eigen::Vector2d& pts) const
{
    // The 2D exact envelope is its own object, so this cannot go through the 3D overload the
    // way the sampled path does -- that would consult an exact_envelope no 2D init ever
    // filled in, and an empty FastEnvelope answers "outside" for every query.
    if (!disabled && use_exact && m_kind == Kind::Edges2d) {
        return exact_envelope_2d.is_outside(pts);
    }
    return is_outside(Vector3d(pts[0], pts[1], 0));
}

bool SampleEnvelope::is_outside(const std::array<Eigen::Vector3d, 3>& tri) const
{
    if (disabled) return false;
    if (use_exact) {
        require_exact_kind(Kind::Triangles3d, "triangle");
        return exact_envelope.is_outside(tri);
    }
    std::array<Vector3d, 3> vs = {
        {Vector3d(tri[0][0], tri[0][1], tri[0][2]),
         Vector3d(tri[1][0], tri[1][1], tri[1][2]),
         Vector3d(tri[2][0], tri[2][1], tri[2][2])}};
    static thread_local std::vector<Vector3d> ps;
    ps.clear();


    sampleTriangle(vs, ps, sampling_dist);

    size_t num_queries = 0;
    size_t num_samples = ps.size();

    double bvh_sq_dist = std::numeric_limits<double>::max();
    Vector3d bvh_nearest_point;
    int bvh_prev_facet = -1;

    size_t cnt = 0;
    const size_t ps_size = ps.size();
    for (size_t i = ps_size / 2; i < ps.size(); i = (i + 1) % ps_size) { // check from the middle

        const Vector3d& bvh_p = ps[i];
        if (bvh_prev_facet != -1) {
            m_bvh->point_facet_distance(bvh_p, bvh_prev_facet, bvh_nearest_point, bvh_sq_dist);
        }
        if (bvh_sq_dist > eps2) {
            m_bvh->facet_in_envelope_with_hint(
                bvh_p,
                eps2,
                bvh_prev_facet,
                bvh_nearest_point,
                bvh_sq_dist);
        }

        ++num_queries;

        if (bvh_sq_dist > eps2) {
            wmtk::logger().trace("num_queries {} / {}", num_queries, num_samples);
            return true;
        }
        cnt++;
        if (cnt >= ps_size) {
            break;
        }
    }


    wmtk::logger().trace("num_queries {} / {}", num_queries, num_samples);
    return false;
}

bool SampleEnvelope::is_outside(const std::array<Vector3d, 2>& edge) const
{
    if (disabled) return false;
    if (use_exact) {
        // Exact for both 3D envelope kinds: a segment is inside when it is covered by the
        // union of the prisms (Triangles3d) or the hexahedra (Edges3d), which fast-envelope
        // decides without sampling. The sampled path below can only test finitely many points
        // on the segment, so it pays for that with the eps/2 shrink in eps2_edge; this does not.
        if (m_kind == Kind::Edges2d || m_kind == Kind::Uninitialized) {
            require_exact_kind(Kind::Edges3d, "3D segment");
        }
        return exact_envelope.is_outside(edge[0], edge[1]);
    }
    static thread_local std::vector<Vector3d> pts;
    pts.clear();

    // N > L / sampling_dist, so the spacing L/N is strictly below sampling_dist and every
    // point of the segment is within sampling_dist/2 of a sample. eps2_edge is the radius
    // that turns "every sample inside" into "the whole segment inside eps"; see the header.
    const int N = (edge[0] - edge[1]).norm() / sampling_dist + 1;
    pts.reserve(N);

    for (int n = 0; n <= N; n++) {
        Vector3d tmp = edge[0] * (double(n) / N) + edge[1] * (double(N - n) / N);
        pts.push_back(tmp);
    }

    double sq_dist;
    Vector3d nearest_point;
    for (size_t i = 0; i < pts.size(); ++i) {
        m_bvh->nearest_facet(pts[i], nearest_point, sq_dist);
        if (sq_dist > eps2_edge) {
            wmtk::logger().trace("fail envelope check 5");
            return true;
        }
    }

    return false;
}

bool SampleEnvelope::is_outside(const std::array<Vector2d, 2>& edge) const
{
    if (!disabled && use_exact && m_kind == Kind::Edges2d) {
        return exact_envelope_2d.is_outside(edge[0], edge[1]);
    }
    std::array<Vector3d, 2> e;
    for (size_t i = 0; i < edge.size(); ++i) {
        e[i] = Vector3d(edge[i][0], edge[i][1], 0);
    }
    return is_outside(e);
}


double SampleEnvelope::nearest_point(const Eigen::Vector3d& pts, Eigen::Vector3d& result) const
{
    double dist;
    m_bvh->nearest_facet(pts, result, dist);
    return dist;
}
double SampleEnvelope::nearest_point(const Eigen::Vector2d& pts, Eigen::Vector2d& result) const
{
    double dist;
    m_bvh->nearest_facet(pts, result, dist);
    return dist;
}

double SampleEnvelope::squared_distance(const Eigen::Vector3d& p) const
{
    double d2;
    SimpleBVH::Vector3d out;
    m_bvh->nearest_facet(p, out, d2);
    return d2;
}

double SampleEnvelope::squared_distance(const Eigen::Vector2d& p) const
{
    double d2;
    SimpleBVH::Vector2d out;
    m_bvh->nearest_facet(p, out, d2);
    return d2;
}

} // namespace wmtk