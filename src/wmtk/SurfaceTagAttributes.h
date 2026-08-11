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

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
    }

    void merge(const SurfaceTagAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};

} // namespace wmtk
