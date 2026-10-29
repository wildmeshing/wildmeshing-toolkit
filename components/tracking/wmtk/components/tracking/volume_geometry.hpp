#pragma once

#include "volume_complex.hpp"

#include <Eigen/Core>

#include <map>
#include <optional>
#include <vector>

/// The geometry of a volume map's local maps: patches embedded star-shaped, and the radial map
/// between two of them.
namespace wmtk::components::tracking::volume {

/// Positions by vertex id (local ids, then virtual ones); a vertex a patch does not use may hold
/// anything.
using Positions = std::vector<Eigen::Vector3d>;

/// The sign of det(b - a, c - a, d - a), exactly.
int orient(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d);

/// Whether p is in the closed tet (a, b, c, d), exactly.
bool inside(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d);
/// p's barycentric coordinates in the tet (a, b, c, d), by a backward stable solve: in a tet
/// that is nearly flat they put p back where it is, to rounding, which an inverse of the edge
/// matrix computed once does not.
Eigen::Vector4d barycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d);
/// barycentric(), exactly (rational volume ratios), rounded once: for a nearly flat tet, whose
/// coordinates in doubles are only right as a whole. -inf for a point that is not a number.
Eigen::Vector4d exact_barycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d);
/// The planes of the tet's four faces p is outside of, exactly, each with p's distance to it:
/// the face opposite corner k for bit k of `faces`; all four for a flat tet.
struct Outside
{
    int faces = 0;
    double distance = 0; // the largest
};
Outside outside_of(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d);
/// The tet of `T` (positions X) holding p, among those `accept` takes: one that holds it
/// exactly; else, of those p is outside of by a rounding (its distance to each face plane it is
/// outside of below 1e-9 of the tet's size), the one whose smallest exact barycentric
/// coordinate is largest; nothing if p is farther out of them all. With p's coordinates in it,
/// clamped to it, and that smallest coordinate before clamping (0 if held exactly); nothing for
/// a point that is not a number. Coordinates that doubles get wrong -- outside the tet for a
/// point in it, as in a nearly flat tet -- are computed exactly.
struct Location
{
    size_t tet;
    Eigen::Vector4d b;
    double min_coordinate;
};
template <typename Accept>
std::optional<Location>
locate_in(const Tets& T, const Positions& X, const Eigen::Vector3d& p, const Accept& accept)
{
    if (!p.allFinite()) return std::nullopt;
    std::vector<size_t> within_rounding;
    for (size_t k = 0; k < T.size(); ++k) {
        if (!accept(k)) continue;
        const auto& t = T[k];
        // only a tet whose box (grown by a rounding) holds p can hold it or be a rounding off
        Eigen::Vector3d lo = X[t[0]], hi = lo;
        for (int c = 1; c < 4; ++c) {
            lo = lo.cwiseMin(X[t[c]]);
            hi = hi.cwiseMax(X[t[c]]);
        }
        const double size = (hi - lo).norm(), eps = 1e-9 * size;
        if (((p.array() < lo.array() - eps) || (p.array() > hi.array() + eps)).any()) continue;
        const Outside o = outside_of(p, X[t[0]], X[t[1]], X[t[2]], X[t[3]]);
        if (o.faces == 0) {
            Eigen::Vector4d b = barycentric(p, X[t[0]], X[t[1]], X[t[2]], X[t[3]]);
            if (!(b.minCoeff() >= -1e-9 && b.maxCoeff() <= 1 + 1e-9)) {
                b = exact_barycentric(p, X[t[0]], X[t[1]], X[t[2]], X[t[3]]);
            }
            b = b.cwiseMax(0.);
            return Location{k, b / b.sum(), 0.};
        }
        if (o.distance <= eps) within_rounding.push_back(k);
    }
    std::optional<Location> out;
    if (!within_rounding.empty()) {
        for (const size_t k : within_rounding) {
            const auto& t = T[k];
            const Eigen::Vector4d b = exact_barycentric(p, X[t[0]], X[t[1]], X[t[2]], X[t[3]]);
            if (!out || b.minCoeff() > out->min_coordinate) out = Location{k, b, b.minCoeff()};
        }
    }
    if (out) {
        out->b = out->b.cwiseMax(0.);
        out->b /= out->b.sum();
    }
    return out;
}

/**
 * @brief Tets embedded with every one the same way round, and a centre that sees every
 * boundary face from inside: a ray from it leaves the patch through one face, once.
 */
class StarPatch
{
public:
    /// Nothing if a tet is flat, or the tets are not all the same way round (exactly).
    static std::optional<StarPatch> make(const Tets& T, const Positions& P);

    const Tets& tets() const { return m_T; }
    const Positions& positions() const { return m_P; }
    /// The boundary faces, sorted, and the same faces oriented outwards.
    const std::vector<Tri>& faces() const { return m_F; }
    /// The diameter of the patch's bounding box.
    double scale() const { return m_scale; }
    /// Whether every vertex is the apex of a cone: shared by every tet and on the boundary.
    std::optional<Vid> boundary_apex() const;
    /// The vertex every tet shares, inside the patch (the star of a vertex).
    std::optional<Vid> interior_apex() const;

    /// Whether `s` is strictly inside every boundary face's plane, exactly.
    bool sees_all(const Eigen::Vector3d& s) const;
    /// Sets the centre; false (and nothing set) if it does not see every face.
    bool set_centre(const Eigen::Vector3d& s);
    const Eigen::Vector3d& centre() const { return m_s; }
    /// The centre's distance to the nearest face plane, over scale().
    double margin() const;

    /// Where the ray from the centre through x leaves: the face (an index into faces()), the
    /// exit point's barycentric coordinates in its sorted corners, and |x - s| / |q - s|.
    struct Exit
    {
        size_t face;
        Eigen::Vector3d w;
        double t;
    };
    Exit exit(const Eigen::Vector3d& x) const;
    /// The tet holding y (the one whose smallest barycentric coordinate is largest) and the
    /// coordinates, in the order of tets()[tet], clamped to the tet.
    struct Located
    {
        size_t tet;
        Eigen::Vector4d b;
    };
    Located locate(const Eigen::Vector3d& y) const;
    /// The index in tets() of a tet given by its vertices, any order.
    std::optional<size_t> index_of(const Tet& t) const;

    /// The outward normal and a corner of each face (the plane is n.(x - a) = 0).
    std::vector<Eigen::Vector3d> normals, corners;

private:
    Tets m_T;
    Positions m_P;
    std::vector<Tri> m_F, m_Fo;
    std::vector<std::pair<Tet, uint32_t>> m_index; // by sorted vertices, sorted
    double m_scale = 0;
    Eigen::Vector3d m_s = Eigen::Vector3d::Zero();
};

/**
 * @brief max t subject to N_i . x + t <= b_i, |x - x0|_inf <= L and t <= t_max, by a dense
 * simplex: the largest ball (in the normals' metric) around a point of the polytope. `x0` need
 * not be inside: the start is t at its most negative. Returns (x, t), nothing if the solver
 * fails.
 */
std::optional<std::pair<Eigen::Vector3d, double>> chebyshev(
    const std::vector<Eigen::Vector3d>& N,
    const std::vector<double>& b,
    const Eigen::Vector3d& x0,
    double L,
    double t_max);

/// A centre every patch sees all its faces from, as far from the face planes as can be (the
/// farther, the better conditioned the radial map); checked exactly. Nothing if none.
std::optional<Eigen::Vector3d> kernel(const std::vector<const StarPatch*>& patches);
/// For a cone from a boundary apex c whose faces at c share an inward direction d: c + eps d,
/// eps halved from the patch size until the centre sees every face. Nothing otherwise.
std::optional<Eigen::Vector3d> apex_centre(const StarPatch& S);
/// A centre for S: kernel(), else apex_centre(), else an interior apex that sees every face.
/// `thin` tells whether the kernel was too thin for the LP.
std::optional<Eigen::Vector3d> centre_of(const StarPatch& S, bool* thin = nullptr);

/**
 * @brief A fallback embedding of a cone from a boundary apex c: every vertex keeps its distance
 * from c, and its direction from c is laid out again, on the sphere, so the cone is
 * star-shaped about a point next to c. The directions live in the gnomonic plane of a direction
 * d; the link's triangles rest in the shapes of their real spherical triangles (near-degenerate
 * ones opened to a height of `fat` of their longest edge), the outline is kept star-shaped about
 * the origin by a scaffold of fan triangles (SLIM). Then the directions and radii are polished
 * by L-BFGS on the symmetric Dirichlet energy of the tets themselves. Nothing if T is not a
 * cone with a disk for a link.
 */
std::optional<Positions> spherical(const Tets& T, const Positions& P, double fat = 0.05);
/// A cone from a boundary apex: the disk of its faces away from the apex laid out by Tutte on
/// a circle and lifted into a convex bowl (Maxwell-Cremona), the apex above it. Scaled to the
/// size of P's patch.
std::optional<Positions> bowl(const Tets& T, const Positions& P);
/// A convex realization of a ball whose boundary holds every vertex but at most the one apex:
/// Tutte on the boundary sphere, Maxwell-Cremona lift, the apex at the centroid.
std::optional<Positions> convex(const Tets& T, const Positions& P);

} // namespace wmtk::components::tracking::volume
