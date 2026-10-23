#pragma once

#include "Fwd.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace wmtk::history {

/**
 * @brief The kinds of edit a complex goes through.
 *
 * A complex is a surface, made of triangles, or a curve network, made of segments (see
 * History); either way its cells are called faces here. Every kind is described on the complex
 * alone -- vertex and face ids, positions -- never on the mesh class that performed it, so a tet
 * mesh can report the changes to its tracked surface, and a triangle mesh the changes to its
 * tracked curves, in the same terms as a TriMesh reports its own. The ids are the recording
 * mesh's slots: a split names the slots it was handed, and a consolidation renumbers every live
 * vertex and face by rank, as the meshes do.
 */
enum class EditKind : uint8_t {
    /// v = {v0, v1, new_v}. faces = {f, new_f} for each face on the edge: f keeps the v0 half
    /// (v1 renamed to new_v in place) and new_f is f with v0 renamed to new_v. On a curve the
    /// face on the edge is the segment (v0, v1) itself, if it is one.
    split_edge = 1,
    /// Surfaces only. v = {v0, v1, v2, new_v}. faces = {f, new_f1, new_f2}: f with v0, new_f1
    /// with v1 and new_f2 with v2 renamed to new_v, each in place.
    split_face = 2,
    /// v = {removed, kept}. Faces with both endpoints go; `removed` is renamed to `kept`
    /// elsewhere. Duplicate faces this leaves at `kept` merge into the smallest fid, and a vertex
    /// it leaves without faces is retired -- unless, on a curve, it is a feature point.
    collapse_edge = 3,
    /// Surfaces only. v = {v0, v1, a, b}, faces = {fa, fb}: edge (v0,v1) becomes (a,b). In fa,
    /// which holds the apex b, v1 is renamed to a; in fb, which holds a, v0 is renamed to b.
    swap_edge = 4,
    /// Only vertex positions change (smoothing, or a sweep outside any operation).
    move_vertices = 5,
    /// Live vertices and faces renumbered by rank, in id order.
    consolidate = 6,
    /// v = the corners of the faces it adds, a face's worth each; faces = the faces it removes,
    /// then the ones it adds, as many as v has faces. A change no other kind describes -- a
    /// tracked surface re-derived, say -- with no correspondence between what goes and what
    /// comes. A vertex it leaves without faces is retired, as by a collapse; one an added face
    /// names is alive, its position among the moves if it was not alive before.
    replace_faces = 7,
    /// v = the new id of every vertex slot, kNoVertex for one that goes; faces = the new id of
    /// every face slot, kNoVertex for one that goes, or empty; moves = the position of every new
    /// vertex. Vertices are renamed in place in every face -- several may become one -- and a
    /// face left with a repeated corner goes; the faces that stay take the ids `faces` gives, or
    /// are renumbered by rank. Several faces given one id -- duplicates a weld leaves -- become
    /// one, with the first's corners. A weld of coincident vertices, a change of coordinates, a
    /// reordering: each face keeps its corners, so a point keeps its barycentric coordinates.
    relabel = 8,
    /// The complex is replaced by one lying on it -- the same surface or curves, cut differently
    /// (an insertion into a tet mesh, an arrangement of segments). v = the corners of its faces,
    /// a face's worth each, in its own vertex ids from 0; faces = per face, how many faces before
    /// it lies on, then those, and after them its features, how many then their vertices; moves =
    /// the position of every vertex. A point stays where it is in space.
    embed = 9,
};

/// No vertex: in a relabel, the slot of one that goes.
constexpr uint64_t kNoVertex = uint64_t(-1);

struct VertexMove
{
    uint64_t vid;
    Eigen::Vector3d pos;
};

/// One committed operation.
struct Edit
{
    EditKind kind;
    std::vector<uint64_t> v;
    std::vector<uint64_t> faces;
    /// The position, after the operation, of every live vertex it wrote.
    std::vector<VertexMove> moves;
};

/**
 * @brief A complex and the edits it went through, in commit order.
 *
 * Two kinds of complex, by the number of corners N of a face:
 * - a surface (N = 3, SurfaceHistory), made of triangles;
 * - a curve network (N = 2, CurveHistory), made of segments. Its positions are 3D like a
 *   surface's; a 2D application's have z = 0.
 *
 * Replaying the edits from the snapshot (Replay.hpp) reproduces every later state, ids
 * included, which is what a map between the first and the last state is built from.
 * SurfaceHistory and CurveHistory name the two (Fwd.hpp).
 */
template <int N>
struct History
{
    static_assert(N == 2 || N == 3, "a history is of triangles or of segments");
    /// A face's corners: a triangle's three, a segment's two.
    using Face = std::array<uint64_t, N>;
    /// A feature: a cell one dimension down from the faces, which the operations keep -- an edge
    /// of a surface, a point of a curve network.
    using Feature = std::array<uint64_t, N - 1>;

    /// The snapshot: every vertex and face slot, live or not.
    std::vector<Eigen::Vector3d> vertices;
    std::vector<uint8_t> vertex_alive;
    std::vector<Face> faces;
    std::vector<uint8_t> face_alive;
    /**
     * Features, which a map must take onto themselves, each with its vertices sorted. The edits
     * carry them along by fixed rules, so only the snapshot lists them:
     * - a surface's feature edges (label boundaries, say): a split feature edge leaves two
     *   feature halves, a collapse renames the removed endpoint's feature edges to the survivor,
     *   a swap never takes one;
     * - a curve network's feature points (its junctions and ends, points of the input): a split
     *   makes none, a collapse of one renames it to the survivor. A feature point keeps its
     *   vertex alive without any segment: it may lie off the curves.
     */
    std::vector<Feature> features;

    std::vector<Edit> edits;

    /// Free-form text saved with the history; the recording application's parameters, say.
    std::string metadata;

    /**
     * The id the application's output gives each vertex of the last state, kNoVertex for one it
     * does not write -- for an application whose output numbers vertices its own way: a tet
     * mesh's, whose tracked surface is no element of the output, so a point on it is named by
     * the output's three vertices; a triangle mesh's, whose tracked curves are named by two.
     * Empty when the last state's ids are the output's own, as a TriMesh application's are.
     */
    std::vector<uint64_t> output_vertex_ids;

    void save(const std::string& path) const;
    /// Throws if the file holds the other kind of complex (see face_size()).
    static History load(const std::string& path);
};

/// The number of corners of a face of the history saved at `path`: 3 for a surface, 2 for a
/// curve network. Throws if it holds no history.
int face_size(const std::string& path);

/**
 * @brief The history of a pipeline: `h`, then `next`, which starts where `h` ends -- the same
 * complex, ids and positions included (a mesh built from h's last state, recording from there).
 * Appends next's edits. False, appending nothing, if next does not start there.
 */
template <int N>
bool append(History<N>& h, const History<N>& next);

/**
 * @brief `h`, then `next`, which starts on a complex lying on h's last state: the same surface
 * or curves, cut differently (an insertion into a tet mesh, an arrangement). Appends an embed
 * edit onto next's first state -- `sources[f]` the faces of h's last state that next's face f
 * lies on, every one where they overlap -- then next's edits. Every face and vertex of next's
 * first state must be alive.
 */
template <int N>
void append_embedded(
    History<N>& h,
    const History<N>& next,
    const std::vector<std::vector<uint64_t>>& sources);

/**
 * @brief Collects the edits of operations that may commit concurrently.
 *
 * A mesh appends while it still holds the locks of the operation it is committing, so two
 * operations whose regions overlap append in the order they committed, and the order of the
 * edits is a valid serial order of the run.
 *
 * A mesh reports every vertex an operation *wrote*, which includes vertices it only read through
 * non-const access. The recorder keeps the last position it saw for every vertex and drops the
 * moves that leave a vertex where it was, and an edit that only moved vertices and moved none.
 */
template <int N>
class BasicRecorder
{
public:
    explicit BasicRecorder(History<N> start);

    void append(Edit&& e);

    /// The ids were renumbered (a consolidation): `positions` holds every live vertex's position
    /// under its new id, and every other id is unknown.
    void rebase(std::vector<Eigen::Vector3d> positions);

    /// Not synchronised: read it once the recording mesh is done.
    const History<N>& history() const { return m_history; }
    History<N>& history() { return m_history; }

private:
    std::mutex m_mutex;
    History<N> m_history;
    std::vector<Eigen::Vector3d> m_known;
    std::vector<uint8_t> m_known_valid;
};

} // namespace wmtk::history
