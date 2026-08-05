
#include "Quadrics.hpp"
#include "TopoOffsetTetMesh.h"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <set>

namespace wmtk::components::topological_offset {

double TopoOffsetTetMesh::get_quality(const std::array<size_t, 4>& vids) const
{
    std::array<double, 12> T;
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 3; ++j) {
            T[k * 3 + j] = m_vertex_attribute[vids[k]].m_posf[j];
        }
    }
    return wmtk::AMIPS_energy(T);
}

double TopoOffsetTetMesh::get_quality(const Tuple& t) const
{
    return get_quality(oriented_tet_vids(t));
}

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    // the input surfaces must stay fixed
    return !m_vertex_attribute[t.vid(*this)].m_is_on_surface;
}

bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    if (!get_offset_surface_faces_for_vertex(t).empty()) {
        return smooth_after_offset_surface(t);
    }
    return smooth_after_interior(t);
}

bool TopoOffsetTetMesh::is_offset_surface_face(const Tuple& f) const
{
    const bool t0_offset = m_tet_attribute[f.tid(*this)].label == 2;
    const auto t1 = f.switch_tetrahedron(*this);
    const bool t1_offset = t1.has_value() && m_tet_attribute[t1->tid(*this)].label == 2;
    return t0_offset != t1_offset;
}

std::vector<TopoOffsetTetMesh::Tuple> TopoOffsetTetMesh::get_offset_surface_faces_for_vertex(
    const Tuple& t) const
{
    std::vector<Tuple> result;
    std::set<size_t> seen_fids;

    const size_t vid = t.vid(*this);
    for (const Tuple& tet : get_one_ring_tets_for_vertex(t)) {
        const std::array<size_t, 4> tet_vids = oriented_tet_vids(tet);

        // the 3 faces of the tet incident to vid are those obtained by omitting one of the
        // *other* 3 vertices; omitting vid itself gives the one face that does not contain it
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[skip] == vid) continue;

            std::array<size_t, 3> face_vids;
            int k = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != skip) face_vids[k++] = tet_vids[j];
            }

            const auto [face_tuple, unused_tid] = tuple_from_face(face_vids);
            const size_t fid = face_tuple.fid(*this);
            if (!seen_fids.insert(fid).second) continue;

            if (is_offset_surface_face(face_tuple)) {
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
        // Vector3d::Zero() signals "no normal direction could be recovered"; the sample point
        // landed exactly on the input complex, which is degenerate but not an error.
        s.normal = (dist < 1e-12) ? Vector3d::Zero() : Vector3d(diff / dist);
    }
    return samples;
}

bool TopoOffsetTetMesh::smooth_after_interior(const Tuple& t)
{
    const size_t vid = t.vid(*this);

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);
    assert(!locs.empty());

    double max_quality = 0.;
    for (const Tuple& tet : locs) {
        max_quality = std::max(max_quality, m_tet_attribute[tet.tid(*this)].m_quality);
    }

    // AMIPS wants each tet as 12 doubles with the moving vertex first.
    std::vector<std::array<double, 12>> assembles(locs.size());
    for (size_t i = 0; i < locs.size(); ++i) {
        std::array<size_t, 4> local_verts =
            wmtk::orient_preserve_tet_reorder(oriented_tet_vids(locs[i]), vid);
        for (int k = 0; k < 4; ++k) {
            for (int j = 0; j < 3; ++j) {
                assembles[i][k * 3 + j] = m_vertex_attribute[local_verts[k]].m_posf[j];
            }
        }
    }

    if (!m_smooth_solver) {
        m_smooth_solver = optimization::create_basic_solver();
    }

    optimization::AMIPSEnergy3D amips_energy(assembles);
    VectorXd x = m_vertex_attribute[vid].m_posf;
    try {
        m_smooth_solver->minimize(amips_energy, x);
    } catch (const std::exception&) {
        // polysolve reports a failed line search by throwing; the position it reached is
        // still the best it found, and the checks below decide whether to keep it.
    }
    m_vertex_attribute[vid].m_posf = x;

    // Inversion is caught by invariants(), called right after this by TetMesh::smooth_vertex.
    // Only the quality veto needs to be checked here.
    double max_after_quality = 0.;
    for (const Tuple& tet : locs) {
        const size_t tid = tet.tid(*this);
        const double q = get_quality(tet);
        m_tet_attribute[tid].m_quality = q;
        max_after_quality = std::max(max_after_quality, q);
    }

    return max_after_quality <= max_quality;
}

bool TopoOffsetTetMesh::smooth_after_offset_surface(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const Vector3d p0 = m_vertex_attribute[vid].m_posf;

    const std::vector<Tuple> offset_faces = get_offset_surface_faces_for_vertex(t);
    assert(!offset_faces.empty());

    // Laplacian smoothing, restricted to neighbors that are also on the offset surface --
    // pulling in input-complex or interior neighbors would drag the surface off its shape.
    Vector3d p_laplace = Vector3d::Zero();
    {
        int n_neighs = 0;
        for (const size_t nb : get_one_ring_vids_for_vertex(vid)) {
            if (get_offset_surface_faces_for_vertex(tuple_from_vertex(nb)).empty()) continue;
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
    int n_faces = 0;
    for (const Tuple& f : offset_faces) {
        const std::array<Tuple, 3> face_verts = get_face_vertices(f);
        const Vector3d p_a = m_vertex_attribute[face_verts[0].vid(*this)].m_posf;
        const Vector3d p_b = m_vertex_attribute[face_verts[1].vid(*this)].m_posf;
        const Vector3d p_c = m_vertex_attribute[face_verts[2].vid(*this)].m_posf;
        const double area = 0.5 * (p_b - p_a).cross(p_c - p_a).norm();

        Quadrics face_q(0., 0., 0., 0.);
        bool any_sample = false;
        for (const OffsetSurfaceSample& s : offset_surface_samples(f)) {
            if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no normal direction
            const Vector3d target = s.nearest + m_params.target_distance * s.normal;
            face_q += Quadrics(target, s.normal) * s.weight;
            any_sample = true;
        }
        if (!any_sample) continue;
        face_q *= area;

        q += face_q;
        ++n_faces;
    }
    if (n_faces == 0) return false;

    const Vector3d p_optimal = q.solve(p_laplace);
    const double w = 0.5; // blend toward the quadrics optimum, matching the reference
    const Vector3d p_final = (1 - w) * p0 + w * p_optimal;

    m_vertex_attribute[vid].m_posf = p_final;

    // Inversion is caught by invariants(), called right after this by TetMesh::smooth_vertex.
    // Unlike the reference, there is no bisection fallback toward p0 on rejection yet.
    return true;
}

void TopoOffsetTetMesh::smooth_all_vertices(size_t n_iters)
{
    // mirrors SimWild::smooth_all_vertices (Smooth.cpp)
    for (size_t i = 0; i < n_iters; ++i) {
        std::vector<std::pair<std::string, Tuple>> ops;
        for (const Tuple& v : get_vertices()) {
            ops.emplace_back("vertex_smooth", v);
        }

        if (NUM_THREADS > 0) {
            compute_vertex_partition();
            auto executor = wmtk::ExecutePass<TopoOffsetTetMesh>(wmtk::ExecutionPolicy::kPartition);
            executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
                return m.try_set_vertex_mutex_one_ring(e, task_id);
            };
            executor.num_threads = NUM_THREADS;
            executor(*this, ops);
        } else {
            auto executor = wmtk::ExecutePass<TopoOffsetTetMesh>(wmtk::ExecutionPolicy::kSeq);
            executor(*this, ops);
        }
    }
}

} // namespace wmtk::components::topological_offset
