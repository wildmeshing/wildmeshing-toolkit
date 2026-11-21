#pragma once

#include <wmtk/history/History.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace wmtk::components::tracking {

/// A point on a triangle complex: a face id, and barycentric coordinates with respect to the
/// face's corners in the order the face stores them.
struct SurfacePoint
{
    uint64_t face;
    Eigen::Vector3d bary;
    /// A further term of the coordinates (bary + lo), for a point given more precisely than
    /// doubles hold (a node of a mapped curve, say). A mapped point comes back with lo = 0 where
    /// its doubles tell which point of the other mesh it stands for, and with a further term
    /// where the map is too ill-conditioned for that (see SurfaceMap::to_output); bary alone is
    /// then still the point to within a unit in the last place.
    Eigen::Vector3d lo = Eigen::Vector3d::Zero();
};

/// Further terms of a point's coordinates, past the doubles of its `bary`: coordinate k is
/// bary[k] + sum_i terms[i][k], exactly -- a floating-point expansion, each term below the last
/// place of the one before.
using BaryTail = std::vector<Eigen::Vector3d>;

/// A straight piece of a curve: one face, and the barycentric coordinates of its two ends. The
/// coordinates are homogeneous -- the point is the one they give scaled to sum 1, and a mapped curve
/// keeps them as computed, exactly or rounded, without dividing by their sum -- and an end may have
/// further terms (`tail`); most have none.
struct SurfaceSegment
{
    uint64_t face;
    std::array<Eigen::Vector3d, 2> bary;
    /// The further terms of each end, or null if neither has any.
    std::shared_ptr<const std::array<BaryTail, 2>> tail = nullptr;
};

/// A curve on a triangle complex: segments end to end, each on one face. Where a curve goes from
/// one face to another, it does so on their shared edge or corner.
using SurfaceCurve = std::vector<SurfaceSegment>;

/// Where a map stops being a bijection. Counted, never fatal: the map still answers.
struct MapWarnings
{
    /// A collapse whose patch is not a disk; its faces are mapped by relabelling alone.
    size_t not_a_disk = 0;
    /// A collapse along the boundary or a curve with no triangle after it holding the edge that
    /// replaces the removed vertex's two: flattened onto a chord instead, where a triangle with
    /// all three corners on it goes flat.
    size_t flat_triangle = 0;
    /// An atlas with a triangle that is not positively oriented.
    size_t inverted = 0;
    /// A collapse that merged duplicate faces: two faces map onto one.
    size_t merged_faces = 0;
    /// A mapped point that fell outside every face of the patch it was mapped into, or onto a
    /// face that an operation removed with nothing to map it to.
    size_t unlocated = 0;
    /// Faces a change no operation describes removed or added (history::EditKind::replace_faces):
    /// points there have nowhere to go.
    size_t replaced = 0;
    /// Segments of a curve that have no image: on a face with nowhere to go, or across a local map
    /// that could not be walked. The curve goes on in another part after them.
    size_t broken_segments = 0;
    /// Points, or points of curves, that mapped back do not return to where they started within
    /// the accuracy of SurfaceMapOptions::certify, even computed and stored at the highest
    /// precision there is for them (double-doubles for points).
    size_t imprecise = 0;

    size_t total() const
    {
        return not_a_disk + flat_triangle + inverted + merged_faces + unlocated + replaced +
               broken_segments + imprecise;
    }
};

/// How the map follows a vertex smoothing step (EditKind::move_vertices).
enum class Smoothing {
    /// Points keep their face and barycentric coordinates, riding along with the vertex by at
    /// most its step: a bijection at no cost, as BijectiveRemesh does it.
    barycentric,
    /// The vertex's star before and after the move, flattened jointly, so points stay where they
    /// were on the surface as far as the flattening can: one SLIM solve per move. Measured on a
    /// remeshed textured bunny: a third of barycentric's drift, and the texture's symmetric
    /// Dirichlet energy 10.6 against 232 (the input's 4.1), for twice the time to build the map.
    flatten,
};

/// The scaffold of each joint flattening (Jiang et al. 2017): triangles around the outline the
/// two patches share, which keep it a simple polygon while SLIM lowers their distortion.
enum class Scaffold {
    /// The region between the outline and a box three times the patch, triangulated anew before
    /// every SLIM iteration (box_scaffold, a constrained Delaunay triangulation), each triangle
    /// resting in its current shape, as in the paper: it never pulls the outline back, and
    /// follows the patch wherever SLIM takes it.
    box,
    /// One ring of triangles out to a copy of the outline three times as far, held there, resting
    /// in its first shape: about 2.4 times faster to build the map with, but it pulls the outline
    /// back towards where it started and keeps it from stretching past the ring.
    ring,
};

/// The faces a collapse's local map flattens.
enum class CollapsePatch {
    /// The stars of both ends before, onto the star of the survivor after: the outline is the
    /// link of the edge, away from both, so the layout has room around them; the least distorted.
    both_stars,
    /// The star of the removed vertex before, onto the faces it becomes: the outline is the
    /// removed vertex's link, the survivor on it, and faces that only had the survivor are left
    /// alone. More distorted, since the survivor fans out from the outline, but the faces it
    /// changes are the ones the collapse changes, as a volume map over the surface needs. Where
    /// those faces after are not a disk on the same outline -- a vertex of the boundary or of a
    /// curve collapsed into one off it -- the map takes both stars.
    removed_star,
};

/// How a SurfaceMap is built.
struct SurfaceMapOptions
{
    Smoothing smoothing = Smoothing::flatten;
    Scaffold scaffold = Scaffold::box;
    CollapsePatch collapse_patch = CollapsePatch::both_stars;
    /// Every mapped point and every point of a mapped curve is stored precisely enough to say
    /// which point of the mesh it came from it stands for, and that is checked: the stored value
    /// is mapped back and must return to where it started, to within `source_accuracy` of that
    /// mesh's bounding box diagonal; one that does not is computed and stored more precisely (a
    /// further term, see SurfacePoint::lo and SurfaceSegment::tail) until it does. Mapping then
    /// costs about twice as much. Off: stored by the estimate alone, which can miss where the map
    /// is ill-conditioned.
    /// Off as well for a history with an embed, which maps through space and is a bijection only
    /// as far as the faces before and after it are coplanar.
    bool certify = true;
    /// What certify checks, and what each point is kept to (with certify off as well), as a
    /// fraction of the bounding box diagonal of the mesh the points come from. An accuracy: it
    /// decides how precisely values are computed and stored, never where a point goes. Where the
    /// map is ill-conditioned -- a needle of the input squeezed into a sliver of a local map's
    /// domain, a unit in the last place of a coordinate after a million of them before -- the
    /// doubles of a point no longer tell its preimage to this, and the point gets a further term.
    double source_accuracy = 0x1p-50;
};

/**
 * @brief The map between the first and the last state of a SurfaceHistory.
 *
 * Built from the history alone, one local map per operation (BijectiveRemesh). A split maps its
 * new faces affinely onto the face they subdivide, with the new vertex where the history put it;
 * a collapse and a swap map through a joint embedding of the faces they touch, before and after,
 * on a shared outline, Tutte's lowered in distortion by SLIM within a scaffold (see Scaffold); a
 * vertex move follows the options' smoothing (see Smoothing). A consolidation renumbers faces.
 */
class SurfaceMap
{
public:
    explicit SurfaceMap(const history::SurfaceHistory& h, const SurfaceMapOptions& options = {});

    /// Points on the last state, mapped onto the first. Each step is computed in double-doubles
    /// (exactly where their bound is too large) and a point is kept in double-doubles from step
    /// to step, so rounding does not pile up; it is stored once, at the end, scaled to sum 1 if it
    /// was computed at all.
    ///
    /// How precisely is set in the mesh the points come from, not the one they land on: each
    /// point carries the linear map from a change of its coordinates to the displacement of the
    /// point it started as (the product of the local maps it went through, backwards), and is
    /// computed and stored so that this point is determined to within
    /// SurfaceMapOptions::source_accuracy (2^-50) of that mesh's bounding box diagonal, which the
    /// map then checks by mapping the stored point back (SurfaceMapOptions::certify). Almost
    /// always the doubles of `bary` do; where the map is ill-conditioned (a needle squeezed into a
    /// sliver of a local map's domain: a unit in the last place after is a million before) the
    /// point comes back with a further term, `lo`, and mapping it back with that term returns the
    /// point it started as. `warnings`, if given, gets
    /// what this call found. The map is not changed: any number of threads may map through it at
    /// once.
    std::vector<SurfacePoint> to_input(
        std::vector<SurfacePoint> points,
        MapWarnings* warnings = nullptr) const;
    /// Points on the first state, mapped onto the last; see to_input.
    std::vector<SurfacePoint> to_output(
        std::vector<SurfacePoint> points,
        MapWarnings* warnings = nullptr) const;

    /**
     * @brief Curves on the first state, mapped onto the last, each to the parts it became.
     *
     * The image of a curve under the map, not an approximation of it: every local map is piecewise
     * linear, so the image of a segment is a polyline that bends where the local map does. Each
     * segment is cut at the local map's edges and nothing is straightened, so a point of a curve
     * and its point under to_output() agree up to rounding, and curves grow where the map bends.
     *
     * Exact, with as much precision as each point needs. Every decision -- which piece or
     * triangle holds a point, whether it is on an edge or a corner, which edge or corner a segment
     * leaves through -- is taken by exact predicates (segment_walk.hpp), so the images are embedded
     * as the curves are: they cross where the curves cross and nowhere else. Each segment's image
     * is computed in doubles, double-doubles or exactly, the lowest that decides every sign and
     * gives every point the accuracy it needs, and each point is kept to as many terms
     * (SurfaceSegment::tail) as that takes: its error below 2^-20 of its local feature size -- the
     * shortest piece it ends, and the nearest piece of any curve in the same face that it does not
     * end -- and below 2^-44 of the face -- and at least as precisely as a double-double: a later
     * step may amplify a unit in the last place a millionfold, so a point computed more precisely
     * than doubles hold keeps its next term (most points have two; the exact ones more). Then the
     * image is checked as points are (SurfaceMapOptions::certify): each of its points, mapped back,
     * must lie on the segment it came from, to within source_accuracy of that mesh's diagonal; the
     * segments whose points do not are mapped again, their points kept more precisely, until they
     * do (else MapWarnings::imprecise). Nothing is moved onto an
     * edge or a corner that is not exactly on it, and no tolerance decides where a point goes.
     * Within one step each point lands once, so every copy of it -- the end of one segment and the
     * start of the next, in the face on either side of an edge -- gets the same terms; a point on
     * the outline of a local map keeps its terms as they are; a point on a split edge is exact (a
     * short expansion), so both faces of the edge agree.
     * One part unless a segment has no image (MapWarnings::broken_segments). `warnings`, as in
     * to_output(); the map is not changed.
     */
    std::vector<std::vector<SurfaceCurve>> curves_to_output(
        const std::vector<SurfaceCurve>& curves,
        MapWarnings* warnings = nullptr) const;
    /// Curves on the last state, mapped onto the first; see curves_to_output.
    std::vector<std::vector<SurfaceCurve>> curves_to_input(
        const std::vector<SurfaceCurve>& curves,
        MapWarnings* warnings = nullptr) const;

    /// What building the map found.
    const MapWarnings& build_warnings() const { return m_build_warnings; }
    /// The edits (indices into the history) whose local map raised a build warning.
    const std::vector<size_t>& warned_edits() const { return m_warned_edits; }
    /// A face of a patch with the domain position of each of its corners -- or a piece of one:
    /// a face the map splits virtually appears as several, each with its corners given as
    /// barycentric coordinates in the face.
    struct Face
    {
        uint64_t fid;
        std::array<Eigen::Vector2d, 3> uv;
        std::array<Eigen::Vector3d, 3> sub = {
            {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()}};
    };
    /// One flattened patch: the faces it covers before and after an edit, in a shared domain.
    struct Cell
    {
        std::vector<Face> before, after;
        /// An edge split's cell: the ends a, b of the edge as laid out, and the new vertex
        /// w = t0 a + t1 b on it, exactly (t0 + t1 = 1, both doubles). A point on the edge maps
        /// by these numbers alone, so the two faces of the edge map it alike, bit for bit.
        struct EdgeSplit
        {
            Eigen::Vector2d a, b, w;
            double t0, t1;
        };
        std::optional<EdgeSplit> split;
    };
    /// The local maps of history edit `edit`, one cell per patch flattened (a collapse or a move
    /// has one per side of the curves through it); none for an edit that needed none. A cell
    /// maps a point of a face before to the same point of the domain, on a face after.
    std::vector<const Cell*> cells_of(size_t edit) const;


    /// The complex at the start and at the end, every slot, as the history holds them.
    const std::vector<Eigen::Vector3d>& input_vertices() const { return m_in_vertices; }
    const std::vector<std::array<uint64_t, 3>>& input_faces() const { return m_in_faces; }
    const std::vector<uint8_t>& input_face_alive() const { return m_in_face_alive; }
    const std::vector<Eigen::Vector3d>& output_vertices() const { return m_out_vertices; }
    const std::vector<std::array<uint64_t, 3>>& output_faces() const { return m_out_faces; }
    const std::vector<uint8_t>& output_face_alive() const { return m_out_face_alive; }
    /// The feature edges at the end, smaller vertex id first.
    const std::set<std::array<uint64_t, 2>>& output_feature_edges() const { return m_out_features; }

    /// How many operations needed a local map, by kind (index: history::EditKind).
    const std::array<size_t, 16>& cells_by_kind() const { return m_cells_by_kind; }
    /// Sides of collapses that removed a vertex on the boundary or on a feature curve. Its two
    /// edges along the curve become one; the map splits the triangle holding that new edge
    /// virtually where the removed vertex was, so both sides keep their area.
    size_t curve_collapses() const { return m_curve_collapses; }
    /// Vertex moves whose points keep their barycentric coordinates instead of their place: a
    /// vertex on more than one curve, or whose star is not a disk.
    size_t kept_moves() const { return m_kept_moves; }
    /// Local maps between patches one of which is folded over in 3D: a bijection still, but
    /// stretched, since no flat layout matches a fold.
    size_t folded_cells() const { return m_folded_cells; }

private:
    /// An embed: the faces before and after, each with its corners in space and the faces of
    /// the other side it lies on. A point keeps its place in space.
    struct Embedding
    {
        std::vector<std::array<Eigen::Vector3d, 3>> before, after;
        std::vector<std::vector<uint64_t>> to_after, to_before;
    };
    struct Step
    {
        history::EditKind kind;
        std::vector<Cell> cells;
        /// consolidate (and relabel, which renumbers faces as one does): face renumbering.
        std::vector<uint64_t> old_to_new, new_to_old;
        /// embed only.
        Embedding embedding;
        /// Faces this step removes with nothing to map their points to (a collapse that could
        /// only be mapped by relabelling, a replace_faces).
        std::vector<uint64_t> dead;
        /// Faces this step adds from nothing (a replace_faces): mapped back, their points have
        /// nowhere to go.
        std::vector<uint64_t> born;
        /// The history edit this step maps.
        size_t edit = 0;
    };

    std::vector<SurfacePoint>
    map(std::vector<SurfacePoint> points, bool forward, MapWarnings* warnings) const;
    struct Raw;
    /// map(), before the points are stored and certified. `share`, if given, scales each point's
    /// accuracy at every step.
    Raw map_raw(
        std::vector<SurfacePoint> points,
        bool forward,
        MapWarnings* warnings,
        const std::vector<double>* share) const;
    std::vector<std::vector<SurfaceCurve>>
    map_curves(const std::vector<SurfaceCurve>& curves, bool forward, MapWarnings* warnings) const;
    /// map_curves(), once, before it is certified: `accuracy` scales, per segment given (numbered
    /// through the curves), how precisely the points of its image are kept; `origins` gets, per
    /// piece of the image, the segment it came from.
    std::vector<std::vector<SurfaceCurve>> map_curves_once(
        const std::vector<SurfaceCurve>& curves,
        bool forward,
        MapWarnings* warnings,
        const std::vector<double>& accuracy,
        std::vector<std::vector<std::vector<size_t>>>* origins) const;

    std::vector<Step> m_steps;
    MapWarnings m_build_warnings;
    std::vector<size_t> m_warned_edits;
    std::array<size_t, 16> m_cells_by_kind = {};
    size_t m_curve_collapses = 0;
    size_t m_kept_moves = 0;
    size_t m_folded_cells = 0;

    /// The bounding box diagonals of the first and the last state.
    double m_in_scale = 0, m_out_scale = 0;
    bool m_certify = true;
    double m_source_accuracy = 0x1p-50;
    std::vector<Eigen::Vector3d> m_in_vertices, m_out_vertices;
    std::vector<std::array<uint64_t, 3>> m_in_faces, m_out_faces;
    std::vector<uint8_t> m_in_face_alive, m_out_face_alive;
    std::set<std::array<uint64_t, 2>> m_out_features;
};

} // namespace wmtk::components::tracking
