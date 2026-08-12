
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace wmtk::components::topological_offset {

/**
 * The swap operations themselves -- the 3-2, 4-4 and 5-6 edge swaps, the 2-3 face swap, their
 * drivers, the face-attribute tracker and the surface diagonal flip -- are
 * wmtk::TetOptimizerMesh's. What is left here is only what is the offset's own: keeping the
 * region tags consistent, and keeping the offset surface faithful to the implicit offset field
 * of the input complex.
 */

bool TopoOffsetTetMesh::swap_capture_tag(const std::vector<size_t>& tids)
{
    std::map<CellTag, size_t> tag_count;
    for (const size_t t : tids) {
        tag_count[m_tet_attribute[t].tag]++;
    }
    // Refuse any swap whose ring spans more than one tag.
    //
    // The rule used to be "at most two tags, and the majority wins". That is what tore the
    // offset region: a face between differently tagged tets IS the offset surface, so a ring
    // spanning two tags has the surface running through it, and collapsing all the new cells
    // onto the majority tag moves that surface -- merging the two regions wherever the
    // minority lost. Measured, the swap pass alone was enough to make the region
    // non-manifold; with this it is not.
    //
    // Conservative: a genuine surface flip could keep both tags by assigning each new cell the
    // tag of the side it lands on. That is worth doing, and would recover the flips this
    // refuses, but it needs the side-of-surface test to be exact.
    if (tag_count.size() > 1) {
        return false;
    }

    size_t max_count = 0;
    CellTag max_tag;
    for (const auto& [tag, count] : tag_count) {
        if (count > max_count) {
            max_count = count;
            max_tag = tag;
        }
    }
    m_swap_tag.local() = max_tag;
    return true;
}

bool TopoOffsetTetMesh::swap_before_interior(const std::vector<size_t>& tids)
{
    return swap_capture_tag(tids);
}

bool TopoOffsetTetMesh::swap_before_surface(
    const std::vector<size_t>& tids,
    const size_t a,
    const size_t b,
    const size_t c,
    const size_t d)
{
    if (!swap_capture_tag(tids)) {
        return false;
    }

    // The flip replaces the two current surface faces (a,b,c) and (a,b,d) with (a,c,d),(b,c,d).
    // Both must belong to the SAME tracked surface: a mixed flip would hand a triangle of the
    // input complex to the offset boundary, or the reverse, moving the line where one meets the
    // other.
    const auto [f_abc, fid_abc] = tuple_from_face(std::array<size_t, 3>{{a, b, c}});
    const auto [f_abd, fid_abd] = tuple_from_face(std::array<size_t, 3>{{a, b, d}});
    if (fid_abc == static_cast<size_t>(-1) || fid_abd == static_cast<size_t>(-1)) {
        return false;
    }
    if (m_face_attribute[fid_abc].m_surface_class != m_face_attribute[fid_abd].m_surface_class) {
        return false;
    }

    if (m_face_attribute[fid_abc].m_surface_class != OFFSET_SURFACE_CLASS) {
        // Input-complex flip. Containment in the envelope is checked by the shared operation.
        return true;
    }
    return offset_swap_normal_deviation_ok(f_abc, f_abd, a, b, c, d);
}

bool TopoOffsetTetMesh::swap_after_cells(const std::vector<size_t>& tids, bool)
{
    const CellTag& tag = m_swap_tag.local();
    for (const size_t t : tids) {
        m_tet_attribute[t].tag = tag;
    }
    return true;
}

bool TopoOffsetTetMesh::offset_swap_normal_deviation_ok(
    const Tuple& face_abc,
    const Tuple& face_abd,
    size_t a,
    size_t b,
    size_t c,
    size_t d) const
{
    // pool the 4 target-normal samples from both current offset faces, matching the
    // reference's 8-sample pool (4 per triangle)
    std::vector<OffsetSurfaceSample> samples;
    for (const OffsetSurfaceSample& s : offset_surface_samples(face_abc)) samples.push_back(s);
    for (const OffsetSurfaceSample& s : offset_surface_samples(face_abd)) samples.push_back(s);

    const Vector3d pa = m_vertex_attribute[a].m_posf;
    const Vector3d pb = m_vertex_attribute[b].m_posf;
    const Vector3d pc = m_vertex_attribute[c].m_posf;
    const Vector3d pd = m_vertex_attribute[d].m_posf;

    // spread (max-min) of the angle between `dir` and every pooled sample, same formula as
    // collapse_normal_deviation()
    auto spread = [&samples](const Vector3d& dir) {
        double min_angle = std::numeric_limits<double>::max();
        double max_angle = std::numeric_limits<double>::lowest();
        for (const OffsetSurfaceSample& s : samples) {
            if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no normal direction
            const double dot = std::clamp(dir.dot(s.normal), -1., 1.);
            const double angle = (180. / M_PI) * std::acos(dot);
            min_angle = std::min(min_angle, angle);
            max_angle = std::max(max_angle, angle);
        }
        if (min_angle > max_angle) return 0.; // no samples found at all
        return max_angle - min_angle;
    };

    const double nd_old = spread((pb - pa).normalized()); // current diagonal (a,b)
    const double nd_new = spread((pd - pc).normalized()); // diagonal after the flip (c,d)

    if (nd_old >= m_max_normal_deviation_swap_max_deg ||
        nd_new >= m_max_normal_deviation_swap_max_deg) {
        // something might be off here, better don't flip
        return false;
    }

    // only reject a regression: an already-poor alignment doesn't block the flip
    if (nd_old < m_offset_params.max_normal_deviation_deg &&
        nd_new >= m_offset_params.max_normal_deviation_deg) {
        return false;
    }
    return true;
}

} // namespace wmtk::components::topological_offset
