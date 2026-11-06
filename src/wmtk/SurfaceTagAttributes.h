#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace wmtk {

/**
 * @brief Whether a codimension-1 simplex is tracked surface, and which bbox side it lies on.
 *
 * The 3D meshes attach this to faces, the 2D meshes to edges; the four copies were
 * character-identical, `reset()` and `merge()` included.
 *
 * `m_is_bbox_fs` is the bbox side the simplex lies on, or -1 for none: 0/1 = x min/max,
 * 2/3 = y min/max, 4/5 = z min/max. Tagging it is what keeps the bounding box from collapsing.
 */
/// An input set (SurfaceTagAttributes::input_set()): bit i for input i. Its width is the input
/// limit (SurfaceTagAttributes::max_inputs) and the face record's size: change it here only.
using InputSet = uint64_t;

/// A simplex's orientation input by input, as insertion and the tagged-mesh readers hand it to a
/// mesh in a build with WMTK_PER_INPUT_ORIENTATION: (input, count) pairs, inputs ascending, a
/// count against the vertex order the producer names (SurfaceTagAttributes::m_orientation says
/// what a count is). Inputs without a pair have count 0.
using InputOrientations = std::vector<std::pair<int, int>>;

class SurfaceTagAttributes
{
    // Eleven bytes. The values are tiny -- a flag, a bbox side in [-1, 5], a class in {0, 1}, an
    // orientation count that is +-1 or 0 almost everywhere, a 64-bit input set -- but as int the
    // struct was 12 bytes, and the 3D meshes carry four of these per tet slot (the 2D ones three
    // per triangle slot) in storage preallocated several times over: on the largest tetwild
    // inputs that alone was most of a gigabyte. The class and the orientation share a byte as
    // bitfields; both are int8_t so every compiler packs them together. The input set took it
    // from 3 to 5 bytes, 8 more per tet slot: on Thingi10K 46024, the largest tetwild input of
    // the challenging set (156k tets after insertion, two iterations), peak RSS went from 1006
    // to 1017 MB (+1.1%), the simplification's per-face input id included. 64 bits rather than 16
    // so that simwild can give every label its own bit -- each tet tag's region as well as each
    // face tag -- and hold every surface face to its own labels' envelopes: segmented images
    // carry more than 16 labels. That is 6 bytes more per face record: on the same
    // 46024, one input, serial, peak RSS 1001 MB on main (3 bytes) and 1007 MB here (11 bytes).
    // With WMTK_PER_INPUT_ORIENTATION 43 bytes: see m_input_orientation.
public:
    // Bitfields cannot take default member initializers before C++20.
    SurfaceTagAttributes()
        : m_surface_class(0)
#ifndef WMTK_PER_INPUT_ORIENTATION
        , m_orientation(0)
#endif
    {}

    /// Is this simplex part of the tracked surface.
    bool m_is_surface_fs = false;
    /// Which bbox side this simplex is on; -1 for none.
    int8_t m_is_bbox_fs = -1;

    /**
     * @brief Which tracked surface this simplex belongs to, for applications that track more
     * than one. 0 to 3.
     *
     * 0 is the application's primary surface, and is all tetwild and simwild ever use.
     * topological_offset tracks two -- the input complex it must stay in the envelope of, and
     * the offset boundary it is free to move -- and uses 1 for the latter. Meaningless, and
     * left at 0, when m_is_surface_fs is false.
     *
     * The class lives here, on the attribute struct, rather than in a container of its own
     * because the shared operations copy face attributes WHOLESALE -- assignment in the split
     * and collapse caches, merge() in collapse_edge_before, reset() in the swap tracker. A
     * field here is carried by all of that for free; a parallel container would be silently
     * dropped by every one of those operations.
     */
    int8_t m_surface_class : 3;

    /**
     * @brief The input's orientation on this simplex: a signed count of input sheets lying on
     * it, measured against its vertices in ASCENDING id order. -15 to 15.
     *
     * For a face (a < b < c), +k means a net k sheets cover it with normal along (b-a) x (c-a);
     * for an edge (a < b), +k means a net k run a -> b. A count rather than a sign so coincident
     * sheets combine the way they do in a winding number: two opposite ones cancel to 0 (a solid
     * resting on another, a fold), two equal ones give 2. A tracked simplex can therefore have
     * orientation 0 and still be constrained; m_is_surface_fs keeps that meaning, and the
     * orientation only feeds winding numbers.
     *
     * At insertion a SHEET is a set of coincident input triangles (segments) of one orientation:
     * duplicates of one triangle count once, as do the triangles of one coplanar group
     * overlapping the same way, so the count is +-1 or 0 per coplanar group (per input curve in
     * 2D). A collapse merging two separate sheets can make it larger. See
     * embed_triangles_in_tets and embed_segments.
     *
     * Ascending ids because the value then depends on nothing but the simplex's vertex set:
     * attribute copies keyed by the sorted vertex tuple (the swap trackers), a different slot
     * becoming the simplex's canonical one, and consolidate_mesh (which renumbers monotonically)
     * all leave it valid. Anything that moves a value onto a simplex with a DIFFERENT vertex set
     * -- the halves of a split, the faces a collapse merges or renames in place, the faces a
     * surface flip creates -- must convert it, through set_orientation_from /
     * add_orientation_from. merge() does not touch it for that reason: it cannot know the two
     * vertex orders.
     *
     * Meaningful only on a mesh whose application set it at insertion
     * (TetOptimizerMesh / TriOptimizerMesh::m_tracks_orientation); 0 elsewhere. Writes clamp to
     * the range rather than wrap: a face would need 16 net coincident sheets to reach it.
     *
     * One count for all the inputs of the simplex: where two inputs coincide, it is of both, and
     * neither input's own is known. The CMake option WMTK_PER_INPUT_ORIENTATION keeps one count
     * per input instead (m_input_orientation), and the field is then not there: the total is the
     * sum of the per-input counts, read through orientation(), so that there is one stored value
     * and not two to keep equal. That saves no memory -- the 5 bits share a byte with the 3-bit
     * class -- but leaves nothing to disagree. Every reader of the count goes through
     * orientation() and orientation_along(), every writer through the setters below, in both
     * builds; only the setters' meaning for several inputs differs (see set_orientation()).
     */
#ifdef WMTK_PER_INPUT_ORIENTATION
    static constexpr bool per_input_orientation = true;
    /**
     * @brief Per input i, its own orientation count on this simplex (as m_orientation's, of
     * input i's sheets alone), 4-bit two's complement: input i in the low nibble of byte i / 2
     * for even i, the high one for odd i. -8 to 7, clamped like m_orientation, so the total --
     * their sum -- is exact while each input's count stays in range.
     *
     * Where two inputs' sheets coincide in one coplanar group the insertion counts each input
     * once, so the total can exceed the one count of a build without the option, which counts
     * the group once: two inputs facing alike on a face give 2 here and 1 there.
     *
     * 32 bytes more per record, 43 instead of 11: on Thingi10K 46024 (tetwild, one input,
     * serial, two iterations), peak RSS 868 MB with the option off and 887 MB with it on (+2.2%).
     */
    uint8_t m_input_orientation[sizeof(InputSet) * 4] = {};
#else
    static constexpr bool per_input_orientation = false;
    int8_t m_orientation : 5;
#endif
    static constexpr int max_orientation = per_input_orientation ? 7 : 15;
    static constexpr int min_orientation = per_input_orientation ? -8 : -15;

    /**
     * @brief The INPUT SET: which inputs this simplex covers, bit i for input i. At most
     * max_inputs, the width of InputSet.
     *
     * The invariant it carries: a tracked simplex stays within eps of EVERY input in its set --
     * not merely of the union of all inputs, which is what one envelope around everything
     * enforces. With one envelope the solid's tracked boundary may ride on a sheet that crosses
     * it: measured on Thingi10K 104509 cut by a wavy sheet (eps 0.1956), 12 of the 9532 tracked
     * faces separating the solid's inside from its outside ended up farther than eps from the
     * solid (at most 3.61 eps), 7 of them within eps of the sheet, and kept tets reached a sheet
     * vertex 2.50 eps outside the solid. One bit per simplex usually; several where inputs
     * coincide, and then the simplex is held by all of them.
     *
     * Set at insertion from the arrangement's provenance (tetwild, triwild); 0 on a mesh whose
     * application never set it, which then answers containment without it. The operations carry
     * it like every other field here: a split's halves copy it, merge() -- the faces a collapse
     * makes one -- takes the union, and a surface flip gives both new simplices the union of the
     * two it replaces. Bytes rather than one uint64_t so the struct keeps alignment 1: eleven
     * bytes, not sixteen.
     */
    InputSet input_set() const
    {
        InputSet s = 0;
        for (size_t i = 0; i < sizeof(InputSet); ++i) s |= InputSet(m_input_set[i]) << (8 * i);
        return s;
    }
    void set_input_set(InputSet s)
    {
        for (size_t i = 0; i < sizeof(InputSet); ++i) m_input_set[i] = uint8_t(s >> (8 * i));
    }
    /// One input per bit of InputSet: the width is set there, and only there.
    static constexpr int max_inputs = int(8 * sizeof(InputSet));
    uint8_t m_input_set[sizeof(InputSet)] = {};

    /// +1 if `v` is an even permutation of its ascending sort, -1 if odd. N is 2 or 3.
    template <size_t N>
    static int ascending_parity(const std::array<size_t, N>& v)
    {
        int inversions = 0;
        for (size_t i = 0; i < N; ++i)
            for (size_t j = i + 1; j < N; ++j)
                if (v[i] > v[j]) ++inversions;
        return (inversions % 2 == 0) ? 1 : -1;
    }

    /// The orientation against the ascending vertex order: the count (m_orientation), with
    /// WMTK_PER_INPUT_ORIENTATION the sum of the per-input counts.
    int orientation() const
    {
#ifdef WMTK_PER_INPUT_ORIENTATION
        int o = 0;
        for (int i = 0; i < max_inputs; ++i) o += input_orientation(i);
        return o;
#else
        return m_orientation;
#endif
    }

    /// Whether any input's count is nonzero -- the count, without WMTK_PER_INPUT_ORIENTATION.
    /// Not orientation() != 0: two inputs facing opposite ways sum to 0, and each still has one.
    bool oriented() const
    {
#ifdef WMTK_PER_INPUT_ORIENTATION
        for (const uint8_t b : m_input_orientation)
            if (b != 0) return true;
        return false;
#else
        return m_orientation != 0;
#endif
    }

    /// The orientation measured against the simplex ordered as `v` (which must be its vertices).
    template <size_t N>
    int orientation_along(const std::array<size_t, N>& v) const
    {
        return orientation() * ascending_parity(v);
    }

    /// The orientation of the inputs in `inputs`, against the ascending vertex order: with
    /// WMTK_PER_INPUT_ORIENTATION the sum of their counts; without, the one count if the simplex
    /// is on any of them, else 0 -- theirs only where it is on no other input.
    int orientation_of(InputSet inputs) const
    {
#ifdef WMTK_PER_INPUT_ORIENTATION
        int o = 0;
        for (int i = 0; i < max_inputs; ++i)
            if ((inputs >> i) & 1) o += input_orientation(i);
        return o;
#else
        return (input_set() & inputs) != 0 ? int(m_orientation) : 0;
#endif
    }

    /// Store orientation `o`, against the ascending vertex order, clamped, as the orientation of
    /// each input in `inputs`: with WMTK_PER_INPUT_ORIENTATION each of their counts is `o`, the
    /// other inputs' unchanged; without, the one count is `o`. How a producer that knows one
    /// direction for a row of inputs -- a tagged row without per-input counts, the insertion of
    /// a single input -- stores it.
    void set_orientation(InputSet inputs, int o)
    {
#ifdef WMTK_PER_INPUT_ORIENTATION
        for (int i = 0; i < max_inputs; ++i)
            if ((inputs >> i) & 1) set_input_orientation(i, o);
#else
        (void)inputs;
        m_orientation = int8_t(std::clamp(o, min_orientation, max_orientation));
#endif
    }

    /// set_orientation(), with `o` measured against the simplex ordered as `v`.
    template <size_t N>
    void set_orientation_along(InputSet inputs, const std::array<size_t, N>& v, int o)
    {
        set_orientation(inputs, o * ascending_parity(v));
    }

    /**
     * @brief Take the orientation of `src`, simplex `from`, as this simplex's, `to` -- every
     * input's count with WMTK_PER_INPUT_ORIENTATION. `from` and `to` are the two simplices'
     * vertices in corresponding order (a split half, a renamed face): a count along `from` is the
     * same count along `to`. `src` may be this record.
     */
    template <size_t N>
    void set_orientation_from(
        const SurfaceTagAttributes& src,
        const std::array<size_t, N>& from,
        const std::array<size_t, N>& to)
    {
        const int s = ascending_parity(from) * ascending_parity(to);
#ifdef WMTK_PER_INPUT_ORIENTATION
        for (int i = 0; i < max_inputs; ++i) set_input_orientation(i, s * src.input_orientation(i));
#else
        m_orientation = int8_t(std::clamp(s * src.m_orientation, min_orientation, max_orientation));
#endif
    }

    /// set_orientation_from(), adding to this simplex's orientation rather than replacing it:
    /// the face a collapse merges two into covers what both covered.
    template <size_t N>
    void add_orientation_from(
        const SurfaceTagAttributes& src,
        const std::array<size_t, N>& from,
        const std::array<size_t, N>& to)
    {
        const int s = ascending_parity(from) * ascending_parity(to);
#ifdef WMTK_PER_INPUT_ORIENTATION
        for (int i = 0; i < max_inputs; ++i)
            set_input_orientation(i, input_orientation(i) + s * src.input_orientation(i));
#else
        m_orientation = int8_t(
            std::clamp(m_orientation + s * src.m_orientation, min_orientation, max_orientation));
#endif
    }

    /// Whether this simplex, ordered as `v`, and `other`, ordered as `w`, carry the same
    /// orientation along those orders -- every input's count with WMTK_PER_INPUT_ORIENTATION.
    template <size_t N>
    bool same_orientation(
        const std::array<size_t, N>& v,
        const SurfaceTagAttributes& other,
        const std::array<size_t, N>& w) const
    {
        const int s = ascending_parity(v) * ascending_parity(w);
#ifdef WMTK_PER_INPUT_ORIENTATION
        for (int i = 0; i < max_inputs; ++i)
            if (input_orientation(i) != s * other.input_orientation(i)) return false;
        return true;
#else
        return m_orientation == s * other.m_orientation;
#endif
    }

#ifdef WMTK_PER_INPUT_ORIENTATION
    /// Input i's count, against the ascending vertex order.
    int input_orientation(int i) const
    {
        const int nibble = (m_input_orientation[i / 2] >> (4 * (i % 2))) & 0xF;
        return nibble >= 8 ? nibble - 16 : nibble;
    }
    /// Input i's count, measured against the simplex ordered as `v`.
    template <size_t N>
    int input_orientation_along(int i, const std::array<size_t, N>& v) const
    {
        return input_orientation(i) * ascending_parity(v);
    }
    /// Store input i's count, against the ascending vertex order, clamped.
    void set_input_orientation(int i, int o)
    {
        const int shift = 4 * (i % 2);
        const uint8_t nibble = uint8_t(std::clamp(o, min_orientation, max_orientation)) & 0xF;
        uint8_t& byte = m_input_orientation[i / 2];
        byte = uint8_t((byte & ~(0xF << shift)) | (nibble << shift));
    }
    /// Store every input's count from `o`, against the ascending vertex order (times `sign`);
    /// the inputs without a pair get 0.
    void set_input_orientations(const InputOrientations& o, int sign = 1)
    {
        std::fill(std::begin(m_input_orientation), std::end(m_input_orientation), uint8_t(0));
        for (const auto& [i, c] : o)
            if (i >= 0 && i < max_inputs) set_input_orientation(i, sign * c);
    }
    /// set_input_orientations(), with `o` measured against the simplex ordered as `v`.
    template <size_t N>
    void set_input_orientations_along(const std::array<size_t, N>& v, const InputOrientations& o)
    {
        set_input_orientations(o, ascending_parity(v));
    }
#endif

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
        m_surface_class = 0;
#ifdef WMTK_PER_INPUT_ORIENTATION
        std::fill(std::begin(m_input_orientation), std::end(m_input_orientation), uint8_t(0));
#else
        m_orientation = 0;
#endif
        set_input_set(0);
    }

    /// Does NOT combine m_orientation -- see there; the caller converts and sums it. The input
    /// sets unite: the one simplex left covers what both covered.
    void merge(const SurfaceTagAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
        if (attr.m_surface_class != 0) m_surface_class = attr.m_surface_class;
        set_input_set(input_set() | attr.input_set());
    }
};

static_assert(
    sizeof(SurfaceTagAttributes) == 3 + sizeof(InputSet) +
                                        (SurfaceTagAttributes::per_input_orientation
                                             ? SurfaceTagAttributes::max_inputs / 2
                                             : 0),
    "keep the surface tags byte-sized");

} // namespace wmtk
