#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

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
class SurfaceTagAttributes
{
    // Five bytes. The values are tiny -- a flag, a bbox side in [-1, 5], a class in {0, 1}, an
    // orientation count that is +-1 or 0 almost everywhere, a 16-bit input set -- but as int the
    // struct was 12 bytes, and the 3D meshes carry four of these per tet slot (the 2D ones three
    // per triangle slot) in storage preallocated several times over: on the largest tetwild
    // inputs that alone was most of a gigabyte. The class and the orientation share a byte as
    // bitfields; both are int8_t so every compiler packs them together. The input set took it
    // from 3 to 5 bytes, 8 more per tet slot: on Thingi10K 46024, the largest tetwild input of
    // the challenging set (156k tets after insertion, two iterations), peak RSS went from 1006
    // to 1017 MB (+1.1%), the simplification's per-face input id included.
public:
    // Bitfields cannot take default member initializers before C++20.
    SurfaceTagAttributes()
        : m_surface_class(0)
        , m_orientation(0)
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
     * surface flip creates -- must convert it, through orientation_along / set_orientation_along.
     * merge() does not touch it for that reason: it cannot know the two vertex orders.
     *
     * Meaningful only on a mesh whose application set it at insertion
     * (TetOptimizerMesh / TriOptimizerMesh::m_tracks_orientation); 0 elsewhere. Writes clamp to
     * the range rather than wrap: a face would need 16 net coincident sheets to reach it.
     */
    int8_t m_orientation : 5;
    static constexpr int max_orientation = 15;

    /**
     * @brief The INPUT SET: which inputs this simplex covers, bit i for input i. 16 inputs at
     * most.
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
     * two it replaces. Two bytes rather than one uint16_t so the struct keeps alignment 1: five
     * bytes, not six.
     */
    uint16_t input_set() const { return uint16_t(m_input_set_lo | (m_input_set_hi << 8)); }
    void set_input_set(uint16_t s)
    {
        m_input_set_lo = uint8_t(s & 0xff);
        m_input_set_hi = uint8_t(s >> 8);
    }
    static constexpr int max_inputs = 16;
    uint8_t m_input_set_lo = 0;
    uint8_t m_input_set_hi = 0;

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

    /// Store an orientation already measured against the ascending vertex order, clamped.
    void set_orientation(int o)
    {
        m_orientation = int8_t(std::clamp(o, -max_orientation, max_orientation));
    }

    /// The orientation measured against the simplex ordered as `v` (which must be its vertices).
    template <size_t N>
    int orientation_along(const std::array<size_t, N>& v) const
    {
        return m_orientation * ascending_parity(v);
    }

    /// Store orientation `o`, measured against the simplex ordered as `v`.
    template <size_t N>
    void set_orientation_along(const std::array<size_t, N>& v, int o)
    {
        set_orientation(o * ascending_parity(v));
    }

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
        m_surface_class = 0;
        m_orientation = 0;
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

static_assert(sizeof(SurfaceTagAttributes) == 5, "keep the surface tags byte-sized");

} // namespace wmtk
