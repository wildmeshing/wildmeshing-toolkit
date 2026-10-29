#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

/// The combinatorics of a volume map's local maps: patches of tets on the vertices of one
/// operation (VolumeEdit's local ids, and virtual vertices past them).
namespace wmtk::components::tracking::volume {

using Vid = uint32_t;
using Tet = std::array<Vid, 4>;
using Tri = std::array<Vid, 3>;
using Tets = std::vector<Tet>;
/// Faces by their corners, sorted.
using Faces = std::set<Tri>;

Tri sorted(Tri t);
Tet sorted(Tet t);
bool contains(const Tet& t, Vid v);
bool contains(const Tri& t, Vid v);
/// The four faces of a tet, each sorted.
std::array<Tri, 4> faces_of(const Tet& t);
/// The tets as a set of sorted tets.
std::set<Tet> keys(const Tets& T);

/// The faces of T on one tet only; nothing if a face is on more than two.
std::optional<Faces> boundary(const Tets& T);
/// Whether T's boundary is a connected closed surface of Euler characteristic 2, manifold at
/// every edge and vertex.
bool is_ball(const Tets& T);
/// The boundary loop of a triangulated disk, from its smallest vertex; nothing if F is not a
/// disk; empty for no faces.
std::optional<std::vector<Vid>> disk_cycle(const Faces& F);
/// T with edge (a, b) collapsed into b: the tets on it go, a is renamed b. Nothing if that
/// makes a tet twice.
std::optional<Tets> collapse(const Tets& T, Vid a, Vid b);
/// The tets of T at c, grouped across the faces at c that are not in `surf`.
std::vector<Tets> sides(const Tets& T, Vid c, const Faces& surf);
/// The tets of T grouped across the faces two of them share that are not in `surf`: the sides
/// of the tracked surface among them.
std::vector<Tets> side_groups(const Tets& T, const Faces& surf);
/// `surf` after collapsing (i, j) into j.
Faces surf_after(const Faces& surf, Vid i, Vid j);

/// Two patches a local map takes one onto the other: both balls, whose boundaries differ in
/// tracked faces only -- the cap before and after, disks with one outline -- and agree on the
/// rest, the fixed faces.
struct PairSpec
{
    std::string kind;
    Tets before, after;
    Faces fixed, cap_before, cap_after;
};
/// The pair (Tb, Ta) if it is one; `sb` and `sa` are the tracked faces before and after.
std::optional<PairSpec> make_pair(
    const std::string& kind,
    const Tets& Tb,
    const Tets& Ta,
    const Faces& sb,
    const Faces& sa,
    std::string* why = nullptr);

/// The class of one side M (a group of sides()) of a collapse of i into j: 'b' if every tet has
/// j (the side vanishes), 'c' if it has a chord (j, x), else 'a'.
char side_class(const Tets& M, Vid i, Vid j);
/// The ring vertices x of side M with a triangle (i, j, x) inside it.
std::vector<Vid> chords(const Tets& M, Vid i, Vid j);

/// A collapse of i into j, j on the tracked surface, as steps that are each a pair: every chord
/// (j, x) split at a virtual vertex w (ids from `first_virtual`), the collapse, then each w
/// collapsed into its x. Without chords, the collapse alone. `K` is the tets at i and j.
struct Decomposition
{
    std::vector<PairSpec> pairs;
    /// (w, x) per chord, in the order split.
    std::vector<std::pair<Vid, Vid>> virtuals;
    /// The tets after the splits (K1) and after the collapse, before the unsplits (K2).
    Tets split, collapsed;
};
std::optional<Decomposition> decompose_collapse(
    const Tets& K,
    Vid i,
    Vid j,
    const Faces& surf,
    Vid first_virtual,
    std::string* why);

} // namespace wmtk::components::tracking::volume
