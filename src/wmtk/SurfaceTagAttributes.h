#pragma once

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
public:
    /// Is this simplex part of the tracked surface.
    bool m_is_surface_fs = false;
    /// Which bbox side this simplex is on; -1 for none.
    int m_is_bbox_fs = -1;

    /**
     * @brief Which tracked surface this simplex belongs to, for applications that track more
     * than one.
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
    int m_surface_class = 0;

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
        m_surface_class = 0;
    }

    void merge(const SurfaceTagAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
        if (attr.m_surface_class != 0) m_surface_class = attr.m_surface_class;
    }
};

} // namespace wmtk
