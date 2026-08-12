
#include "Quadrics.hpp"
#include "TopoOffsetTetMesh.h"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/predicates/predicates.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <optional>
#include <set>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    // rounds the vertex and refuses the bounding box
    if (!TetOptimizerMesh::smooth_before(t)) {
        return false;
    }
    // the input complex must stay exactly where it is: it is the geometry the offset is
    // measured against, not something to be improved
    return !m_vertex_extra[t.vid(*this)].m_is_on_input;
}

bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    // An offset-surface vertex is placed by the quadrics of the input complex's implicit
    // offset field, which is what keeps the offset faithful. Every other vertex is an ordinary
    // interior one and gets the shared two-stage AMIPS smoother.
    if (!get_offset_surface_faces_for_vertex(t).empty()) {
        return smooth_after_offset_surface(t);
    }
    return TetOptimizerMesh::smooth_after(t);
}

bool TopoOffsetTetMesh::is_offset_face(const Tuple& f) const
{
    return is_offset_face(f.fid(*this));
}

bool TopoOffsetTetMesh::is_offset_face(const size_t fid) const
{
    return face_is_offset(fid);
}

std::vector<TopoOffsetTetMesh::Tuple> TopoOffsetTetMesh::get_offset_surface_faces_for_vertex(
    const Tuple& t) const
{
    std::vector<Tuple> result;
    std::set<size_t> seen_fids;

    const size_t vid = t.vid(*this);
    for (const size_t tid : get_one_ring_tids_for_vertex(t)) {
        const auto tet_vids = oriented_tet_vids(tid);

        // the 3 faces of the tet incident to vid are those obtained by omitting one of the
        // *other* 3 vertices; omitting vid itself gives the one face that does not contain it
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[skip] == vid) {
                continue;
            }

            std::array<size_t, 3> face_vids;
            int k = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != skip) {
                    face_vids[k++] = tet_vids[j];
                }
            }

            const auto [face_tuple, unused_tid] = tuple_from_face(face_vids);
            const size_t fid = face_tuple.fid(*this);
            if (!seen_fids.insert(fid).second) {
                continue;
            }

            if (is_offset_face(face_tuple)) {
                result.push_back(face_tuple);
            }
        }
    }
    return result;
}

std::array<OffsetSurfaceSample, 4> TopoOffsetTetMesh::offset_surface_samples(const Tuple& f) const
{
    const std::array<Tuple, 3> fv = get_face_vertices(f);
    const Vector3d p0 = m_vertex_attribute[fv[0].vid(*this)].m_posf;
    const Vector3d p1 = m_vertex_attribute[fv[1].vid(*this)].m_posf;
    const Vector3d p2 = m_vertex_attribute[fv[2].vid(*this)].m_posf;
    const Vector3d p_mid = (p0 + p1 + p2) / 3.;

    constexpr double u = 0.1;
    std::array<OffsetSurfaceSample, 4> samples;
    samples[0].point = p_mid;
    samples[0].weight = 1.;
    samples[1].point = (1 - u) * p0 + u * p_mid;
    samples[1].weight = u;
    samples[2].point = (1 - u) * p1 + u * p_mid;
    samples[2].weight = u;
    samples[3].point = (1 - u) * p2 + u * p_mid;
    samples[3].weight = u;

    for (OffsetSurfaceSample& s : samples) {
        s.nearest = m_input_complex_bvh.nearest_point(s.point);
        const Vector3d diff = s.point - s.nearest;
        const double dist = diff.norm();

        s.normal = (dist < 1e-12) ? Vector3d::Zero() : Vector3d(diff / dist);
    }
    return samples;
}

bool TopoOffsetTetMesh::smooth_after_offset_surface(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const Vector3d p0 = m_vertex_attribute[vid].m_posf;

    const std::vector<Tuple> offset_faces = get_offset_surface_faces_for_vertex(t);
    assert(!offset_faces.empty());

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);
    auto any_inverted = [&]() {
        for (const Tuple& tet : locs) {
            if (is_inverted(tet)) return true;
        }
        return false;
    };
    // Move the vertex to `target`; if that inverts an incident tet, binary search along the
    // segment from the known-valid p0 to `target` for the furthest point that does not.
    auto move_to = [&](const Vector3d& target) {
        set_vertex_position(vid, target);
        if (!any_inverted()) return;

        double lo = 0.; // m_posf = p0 + lo * (target - p0), always valid
        double hi = 1.; // always invalid
        constexpr int max_iters = 10;
        for (int iter = 0; iter < max_iters; ++iter) {
            const double mid = 0.5 * (lo + hi);
            set_vertex_position(vid, p0 + mid * (target - p0));
            if (any_inverted()) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        set_vertex_position(vid, p0 + lo * (target - p0));
    };

    // Laplacian smoothing, restricted to neighbors that are also on the offset surface --
    // pulling in input-complex or interior neighbors would drag the surface off its shape.
    Vector3d p_laplace = Vector3d::Zero();
    {
        int n_neighs = 0;
        for (const size_t nb : get_one_ring_vids_for_vertex(vid)) {
            if (!m_vertex_extra[nb].m_is_on_offset) continue;
            p_laplace += m_vertex_attribute[nb].m_posf;
            ++n_neighs;
        }
        if (n_neighs == 0) return false;
        p_laplace /= n_neighs;
    }

    // Quadric built from 4 target_distance-offset samples of the input complex per incident
    // offset-surface face (centroid + one near each corner, weighted 1/0.1/0.1/0.1), following
    // Quadrics.cpp's get_triangle_samples_and_area(). Taking several samples rather than just
    // the centroid is what lets the quadric stay feature-aware: near a sharp fold of the input
    // complex, samples on either side pull the per-vertex quadric's minimum toward the fold
    // instead of averaging it away, the same way classic QEM simplification preserves features
    // by summing quadrics from multiple differently-oriented triangles.
    Quadrics q(0., 0., 0., 0.);
    bool any_sample = false;
    for (const Tuple& f : offset_faces) {
        const std::array<Tuple, 3> face_verts = get_face_vertices(f);
        const Vector3d p_a = m_vertex_attribute[face_verts[0].vid(*this)].m_posf;
        const Vector3d p_b = m_vertex_attribute[face_verts[1].vid(*this)].m_posf;
        const Vector3d p_c = m_vertex_attribute[face_verts[2].vid(*this)].m_posf;
        const double area = 0.5 * (p_b - p_a).cross(p_c - p_a).norm();

        Quadrics face_q(0., 0., 0., 0.);
        for (const OffsetSurfaceSample& s : offset_surface_samples(f)) {
            if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no valid normal
            const Vector3d target = s.nearest + m_offset_params.target_distance * s.normal;
            face_q += Quadrics(target, s.normal) * s.weight;
            any_sample = true;
        }
        face_q *= area;

        q += face_q;
    }
    if (!any_sample) return false;

    const Vector3d p_optimal = q.solve(p_laplace, m_offset_params.quadrics_svd_threshold);

    const double w = m_offset_params.smooth_quadrics_weight;
    const double u = m_offset_params.smooth_laplacian_weight;
    const Vector3d p_final = (1 - w - u) * p0 + w * p_optimal + u * p_laplace;

    move_to(p_final);

    // Any remaining inversion (from the binary search's finite precision) is caught by
    // invariants(), called right after this by TetMesh::smooth_vertex.
    return true;
}

} // namespace wmtk::components::topological_offset
