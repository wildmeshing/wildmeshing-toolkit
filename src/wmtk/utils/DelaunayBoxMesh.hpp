#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/Delaunay.hpp>

#include <vector>

namespace wmtk::utils {

/**
 * @brief Build the Delaunay background mesh the triangle insertion embeds into.
 *
 * Seeds a voxel lattice over the input's bounding box grown by diag/15, drops lattice points
 * that crowd the input surface, and Delaunay-triangulates the result.
 *
 * The bounding box is a **parameter, not derived from `points`**, because the two callers grow
 * it from different things and both are load-bearing:
 *
 *   * tetwild passes `m_params.min/max` and `m_params.diag_l`, which come from the ORIGINAL
 *     input surface -- but calls this with the SIMPLIFIED vertices, so the box is deliberately
 *     larger than the point set it is triangulating. It then copies the padded box back into
 *     `m_params.box_min/box_max`, which is what its bbox-face tagging compares against.
 *   * simwild passes the bounding box of the very vertices it hands in.
 *
 * @param envelope   used only to reject lattice points closer than voxel_resolution^2/4 to the
 *                   input surface; they would crowd constraints the arrangement is about to
 *                   refine anyway
 * @param bbox_min, bbox_max  the UNPADDED box to grow from
 * @param diag       the diagonal driving both the padding (diag/15) and the voxel size (diag/20)
 * @param[in,out] points  in: the input vertices as Point3D. out: those plus the 8 box corners
 *                   and the surviving lattice points
 * @param[out] tets  the Delaunay tetrahedralization of `points`
 * @param[out] box_min, box_max  the PADDED box actually used
 */
void delaunay_box_mesh(
    const SampleEnvelope& envelope,
    const Vector3d& bbox_min,
    const Vector3d& bbox_max,
    double diag,
    std::vector<delaunay::Point3D>& points,
    std::vector<delaunay::Tetrahedron>& tets,
    Vector3d& box_min,
    Vector3d& box_max);

} // namespace wmtk::utils
