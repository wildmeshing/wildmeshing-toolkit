#include <queue>
#include "Circle.hpp"
#include "Sphere.hpp"
#include "TopoOffsetTetMesh.h"
#include "TopoOffsetTriMesh.h"


namespace wmtk::components::topological_offset {


bool TopoOffsetTetMesh::tet_is_in_offset_conservative(const size_t t_id, const double threshold_r)
    const
{
    std::queue<Sphere> q;
    q.emplace(*this, t_id);

    bool initial = true;
    while (!q.empty()) {
        Sphere sph = q.front();
        q.pop();

        if (sph.radius() < threshold_r) {
            if (initial) {
                logger().warn(
                    "Initial approximating sphere for tet {} is smaller than the threshold "
                    "radius; deciding it with the offset potential at its centre instead. "
                    "Decrease relative_ball_threshold if this is frequent.",
                    t_id);
            }
            // SUB-THRESHOLD: subdivision has run out of budget on a sphere the Euclidean bracket
            // could not decide. This used to answer "outside" and stop, which is conservative in
            // the sense of never growing too far but is a trap: the bracket is undecided exactly
            // in a shell of width 2r around the offset, so lowering relative_ball_threshold moves
            // the boundary and the region a run produces depends on a subdivision budget rather
            // than on the geometry.
            //
            // Decide it with the potential instead. Phi is defined at a point, so there is no
            // budget: the sphere's centre either is inside the level set or is not, and at this
            // radius the difference between the two answers is below the threshold anyway.
            //
            // Note the whole bracket above IS the conservative form of this test: Phi(x) >= b(d(x))
            // because the closest feature is always feasible, and b(delta) = c, so d + r < delta
            // implies Phi > c over the whole sphere. The two agree; this only replaces the
            // give-up branch.
            if (m_offset_potential &&
                m_offset_potential->value(sph.center()) >= m_offset_potential->target_level()) {
                continue; // centre is inside the level set: treat this sphere as inside
            }
            return false;
        }
        initial = false;

        const bool overlap = sph.overlaps_tet(*this, t_id);
        if (!overlap) {
            continue;
        }

        const double d = m_input_complex_bvh.dist(sph.center());
        if ((d - sph.radius()) > m_offset_params.target_distance) {
            return false; // sphere outside --> tet outside offset
        }
        if ((d + sph.radius()) < m_offset_params.target_distance) {
            continue; // sphere inside --> check next circle
        }

        sph.refine(q); // undetermined --> refine further
    }

    return true; // all spheres inside --> tet inside offset
}

bool TopoOffsetTetMesh::tet_is_in_offset_aggressive(const size_t t_id) const
{
    const auto vs = oriented_tet_vids(t_id);
    for (const size_t& v_id : vs) {
        const double d = m_input_complex_bvh.dist(m_vertex_attribute[v_id].m_posf);
        if (d > m_offset_params.target_distance) {
            return false;
        }
    }
    return true;
}


bool TopoOffsetTetMesh::offset_tet_consistent_topology(const size_t t_id) const
{
    if (m_tet_attribute[t_id].label != 0) {
        log_and_throw_error("non label-0 tet id={} given to consistent topology check", t_id);
    }

    // collect vertices in input/offset
    auto vs = oriented_tet_vids(t_id);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_extra[vs[i]].label == 1) { // bad
            log_and_throw_error("Input adjacent tet given to offset_tet_consistent_topology");
        }
        if (m_vertex_extra[vs[i]].label == 2) { // otherwise if label is 2 (offset)
            vs_in.push_back(vs[i]);
        }
    }

    bool offset_consistent;
    if (vs_in.size() == 3) { // must have exactly one face in input
        int num_faces_in_input = 0;
        for (int i = 0; i < 4; i++) {
            num_faces_in_input += (m_face_extra[tuple_from_face(t_id, i).fid(*this)].label == 2);
        }
        offset_consistent = (num_faces_in_input == 1);
    } else if (vs_in.size() == 4) { // must have two or three faces, no isolated edges in input
        std::vector<Tuple> faces_in_input;
        for (int i = 0; i < 4; i++) {
            Tuple f = tuple_from_face(t_id, i);
            if (m_face_extra[f.fid(*this)].label == 2) {
                faces_in_input.push_back(f);
            }
        }
        if (faces_in_input.size() == 3) {
            offset_consistent = true;
        } else if (faces_in_input.size() == 2) { // must guarantee no floating edge
            std::vector<size_t> face1_vs;
            face1_vs.push_back(faces_in_input[0].vid(*this));
            face1_vs.push_back(faces_in_input[0].switch_vertex(*this).vid(*this));
            face1_vs.push_back(
                faces_in_input[0].switch_edge(*this).switch_vertex(*this).vid(*this));
            std::sort(face1_vs.begin(), face1_vs.end());
            std::vector<size_t> face2_vs;
            face2_vs.push_back(faces_in_input[1].vid(*this));
            face2_vs.push_back(faces_in_input[1].switch_vertex(*this).vid(*this));
            face2_vs.push_back(
                faces_in_input[1].switch_edge(*this).switch_vertex(*this).vid(*this));
            std::sort(face2_vs.begin(), face2_vs.end());
            std::vector<size_t> shared_edge_vids = set_intersection(face1_vs, face2_vs);
            simplex::Tet tet_simp = simplex_from_tet(t_id);
            simplex::Edge oppo_edge =
                tet_simp.opposite_edge(simplex::Edge(shared_edge_vids[0], shared_edge_vids[1]));
            size_t opp_eid = tuple_from_edge(oppo_edge.vertices()).eid(*this);
            offset_consistent = (m_edge_attribute[opp_eid].label == 0);
        } else { // no bueno
            offset_consistent = false;
        }
    } else { // topology would be changed
        offset_consistent = false;
    }

    if (!offset_consistent) {
        return false;
    }

    // check if any topologies would be changed
    if (m_offset_params.respect_all_topologies) {
        for (const int64_t& tag : m_tet_attribute[t_id].tag) {
            if (!tag_tet_consistent_topology(t_id, tag)) {
                return false;
            }
        }
    }

    return true;
}


bool TopoOffsetTetMesh::tag_tet_consistent_topology(size_t t_id, int64_t tag) const
{
    // look at same inline function in tag_tri_consistent_topology for explanation of why we need
    // this. Basically this function is only called if in respect_all_topologies mode, in which case
    // we need to 'simulate' all offset tets as being a part of the offset and the offset only.
    auto get_tags = [this](size_t _t_id) {
        if (m_tet_attribute[_t_id].label != 0) {
            return TEMP_OFFSET_TET_TAG_SET;
        } else {
            return m_tet_attribute[_t_id].tag;
        }
    };

    // The DOMAIN BOUNDARY bounds the tag region too, and the boundary-FACE test further down
    // already says so: its no-opposite-tet branch calls such a face a boundary face of the tag
    // whenever the single incident tet carries it. The vertex and edge tests below compared only
    // tag membership around the one-ring and never looked at the domain boundary, so the halves
    // of the disk condition disagreed -- a vertex or edge sitting ON the box with a uniform
    // one-ring counted as INTERIOR while the box face beside it counted as boundary -- and a tet
    // wedged between the offset front and the box could satisfy the condition and be absorbed.
    //
    // In 2D that let conservative growth eat the ambient region right up to the bounding box, one
    // locally-legal step at a time, until the offset touched the box and the ambient band
    // separating them was gone: 29 of 164 box-touching candidates admitted, 101 offset vertices
    // left pinned on the box. Only bites when the offset actually reaches the box, i.e. when
    // target_distance exceeds the clearance to the domain boundary.
    //
    // True if any tagged tet in `tids` has a domain-boundary face containing every vertex in
    // `must_contain`. Guarded by on_bbox_faces so the walk only runs for simplices that could
    // possibly be on the box.
    auto on_tagged_domain_boundary = [&](const std::vector<size_t>& tids,
                                         const std::vector<size_t>& must_contain) {
        for (const size_t need : must_contain) {
            if (m_vertex_attribute[need].on_bbox_faces.empty()) return false;
        }
        for (const size_t tid : tids) {
            if (get_tags(tid).count(tag) == 0) continue;
            const auto tv = oriented_tet_vids(tid);
            for (int skip = 0; skip < 4; ++skip) {
                std::array<size_t, 3> fv;
                int k = 0;
                for (int j = 0; j < 4; ++j) {
                    if (j != skip) fv[k++] = tv[j];
                }
                bool contains_all = true;
                for (const size_t need : must_contain) {
                    if (fv[0] != need && fv[1] != need && fv[2] != need) {
                        contains_all = false;
                        break;
                    }
                }
                if (!contains_all) continue;
                const auto [ftup, unused_fid] = tuple_from_face(fv);
                if (!ftup.switch_tetrahedron(*this)) return true;
            }
        }
        return false;
    };

    // collect boundary vertices
    auto vs = oriented_tet_vids(t_id);
    std::vector<size_t> boundary_vs;
    for (const size_t& v : vs) {
        auto one_ring_tids = get_one_ring_tids_for_vertex(v);
        bool v_in = false;
        bool first_in = get_tags(one_ring_tids[0]).count(tag) != 0;
        for (const size_t& tid : one_ring_tids) {
            bool t_in = get_tags(tid).count(tag) != 0;
            if (t_in != first_in) {
                v_in = true;
                break;
            }
        }
        if (!v_in) {
            v_in = on_tagged_domain_boundary(one_ring_tids, {v});
        }
        if (v_in) {
            boundary_vs.push_back(v);
        }
    }

    // collect boundary edges
    std::map<simplex::Edge, bool> boundary_edges;
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            simplex::Edge e(vs[i], vs[j]);
            Tuple etup = tuple_from_edge(e.vertices());
            auto incident_tids = get_incident_tids_for_edge(etup);
            bool e_in = false;
            bool first_in = get_tags(incident_tids[0]).count(tag) != 0;
            for (const size_t& tid : incident_tids) {
                bool t_in = get_tags(tid).count(tag) != 0;
                if (t_in != first_in) {
                    e_in = true;
                    break;
                }
            }
            if (!e_in) {
                e_in = on_tagged_domain_boundary(incident_tids, {vs[i], vs[j]});
            }
            boundary_edges[e] = e_in;
        }
    }

    // collect bounndary faces
    std::map<simplex::Face, bool> boundary_faces;
    for (int i = 0; i < 4; i++) {
        simplex::Face f(vs[i], vs[(i + 1) % 4], vs[(i + 2) % 4]);
        auto [ftup, fid] = tuple_from_face(f);

        auto other = ftup.switch_tetrahedron(*this);
        if (other) {
            bool t1_in = get_tags(ftup.tid(*this)).count(tag) != 0;
            bool t2_in = get_tags(other.value().tid(*this)).count(tag) != 0;
            boundary_faces[f] = (t1_in && !t2_in) || (!t1_in && t2_in);
        } else {
            boundary_faces[f] = get_tags(ftup.tid(*this)).count(tag) != 0;
        }
    }

    int num_boundary_faces = 0;
    for (const auto& pair : boundary_faces) {
        num_boundary_faces += (pair.second ? 1 : 0);
    }

    // check criteria (same logic as offset_tet_consistent_topology)
    if (boundary_vs.size() == 3) { // must have exactly one face in input
        return (num_boundary_faces == 1);
    } else if (boundary_vs.size() == 4) { // must have two or three faces, no isolated edges
        if (num_boundary_faces == 3) {
            return true;
        } else if (num_boundary_faces == 2) { // must guarantee no floating edge
            std::vector<simplex::Face> boundary_f_simps;
            for (const auto& pair : boundary_faces) {
                if (pair.second) {
                    boundary_f_simps.push_back(pair.first);
                }
            }
            simplex::Tet tet_simp(vs[0], vs[1], vs[2], vs[3]);
            simplex::Edge opp_edge(
                tet_simp.opposite_vertex(boundary_f_simps[0]).id(),
                tet_simp.opposite_vertex(boundary_f_simps[1]).id());
            return (!boundary_edges[opp_edge]);
        } else { // no bueno
            return false;
        }
    } else { // topology would be changed
        return false;
    }
}


bool TopoOffsetTriMesh::tri_is_in_offset_conservative(const size_t f_id, const double threshold_r)
    const
{
    // IS THE WHOLE TRIANGLE INSIDE THE OFFSET REGION { Phi > c }?
    //
    // Deciding that for a SET rather than a point needs a bound over the set, and Phi has no
    // Lipschitz constant to give one -- it diverges on the input complex. The Euclidean distance
    // does (its constant is 1), and it BRACKETS Phi exactly:
    //
    //   Phi(x) >= b(d(x))    the closest feature of the complex is always in the OGC feasible
    //                        region of the point, so it always contributes; every other
    //                        contribution is non-negative.
    //   b is decreasing, and b(delta) = c by the calibration.
    //
    // so over a circle of radius r about p:
    //
    //   d(p) + r <  delta  =>  Phi > c on the whole circle          (inside)
    //   d(p) - r >  delta  =>  the circle MAY still be inside, at a reentrant feature where
    //                          two contributions add -- but it is not inside by this argument,
    //                          so it is treated as outside (conservative: under-grow, never
    //                          overshoot, and the optimization then moves the boundary out to
    //                          the level set, which is exactly what it exists to do).
    //
    // Both branches are therefore the same comparisons this always made, and that is not a
    // coincidence: the Euclidean test IS the conservative form of the Phi test. What has
    // changed is the bail-out below.
    std::queue<Circle> q;
    q.emplace(*this, f_id);

    while (!q.empty()) {
        Circle circ = q.front();
        q.pop();

        const bool overlap = circ.overlaps_tri(*this, f_id);
        if (!overlap) {
            continue;
        }

        if (circ.radius() < threshold_r) {
            // Too small to bracket anything: DECIDE it, by evaluating Phi at the centre.
            //
            // This used to return false -- "circle too small, therefore the triangle is
            // outside" -- with a warning telling the user to decrease relative_ball_threshold.
            // That turns a resolution limit into a geometric verdict, always in the same
            // direction, and it is why the threshold's default of 0.1 could reject triangles
            // wholesale and produce an offset that ignored target_distance. At this radius the
            // circle is smaller than the resolution the offset is being built at, so the point
            // test is the honest answer and it is unbiased.
            if (m_offset_potential->value(circ.center().head<2>()) >
                m_offset_potential->target_level()) {
                continue;
            }
            return false;
        }

        const double d = m_input_complex_bvh.dist(circ.center());
        if ((d - circ.radius()) > m_offset_params.target_distance) {
            return false; // circle is outside --> tri outside offset
        }
        if ((d + circ.radius()) < m_offset_params.target_distance) {
            continue; // circle is inside --> check next circle
        }

        circ.refine(q); // undetermined --> subdivide
    }

    return true; // all circles inside --> tri inside offset
}


bool TopoOffsetTriMesh::offset_tri_consistent_topology(const size_t f_id) const
{
    if (m_face_extra[f_id].label != 0) {
        log_and_throw_error("non label-0 tri id={} given to consistent topology check", f_id);
    }

    bool offset_consistent;

    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> vs_in;
    for (const size_t& v : vs) {
        if (m_vertex_extra[v].label == 1) {
            log_and_throw_error(
                "Input adjacent tri (id {}) given to offset_tri_consistent_topology",
                f_id);
        }
        if (m_vertex_extra[v].label == 2) {
            vs_in.push_back(v);
        }
    }

    if (vs_in.size() == 3) { // must have exactly two edges in
        int num_es_in = 0;
        for (int i = 0; i < 3; i++) {
            num_es_in += (m_edge_extra[tuple_from_edge(f_id, i).eid(*this)].label == 2);
        }
        // logger().info("\t{}", num_es_in == 2);
        offset_consistent = (num_es_in == 2);
    } else if (vs_in.size() == 2) { // must have exactly one edge in (between two 'in' verts)
        simplex::Edge e(vs_in[0], vs_in[1]);
        offset_consistent = (m_edge_extra[edge_id_from_simplex(e)].label == 2);
    } else { // 0 or 1 vertex in, topology would be changed
        offset_consistent = false;
    }

    if (!offset_consistent) { // definitely cannot add
        return false;
    }

    // check if any topologies changed by adding this tri to offset
    if (m_offset_params.respect_all_topologies) {
        for (const int64_t& id : m_face_attribute[f_id].tags) {
            if (!tag_tri_consistent_topology(f_id, id)) {
                return false;
            }
        }
    }

    return true;
}

// NOTE: this is only called if in 'respect_all_topologies' mode
bool TopoOffsetTriMesh::tag_tri_consistent_topology(size_t f_id, int64_t tag) const
{
    // Basically if we are in respect_all_topologies mode, if a tet is in the offset, we must
    // consider it as only having the tag TMP_TRI_OFFSET_TAG (ie, as if it were to overwrite all
    // tags)
    auto get_tags = [this](size_t _f_id) {
        if (m_face_extra[_f_id].label != 0) {
            return TEMP_OFFSET_TRI_TAG_SET;
        } else {
            return m_face_attribute[_f_id].tags;
        }
    };

    // collect boundary vertices
    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> boundary_vs;
    for (const size_t& v : vs) {
        auto one_ring_fids = get_one_ring_fids_for_vertex(v);
        bool v_in = false;
        bool first_in = get_tags(one_ring_fids[0]).count(tag) != 0;
        for (const size_t& fid : one_ring_fids) {
            bool f_in = get_tags(fid).count(tag) != 0;
            if (f_in != first_in) {
                v_in = true;
                break;
            }
        }
        // The DOMAIN BOUNDARY bounds the tag region too, and this test has to say so because
        // the edge test below already does: its no-opposite-face branch calls such an edge a
        // boundary edge of the tag whenever the single incident face carries it. Without the
        // same rule here the two halves of the disk condition disagreed -- a vertex sitting ON
        // the box with an all-ambient one-ring counted as INTERIOR while the box edge beside it
        // counted as boundary -- so a face wedged between the offset front and the box could
        // satisfy the condition and be absorbed. Ambient was then eaten right up to the box, one
        // locally-legal step at a time, until the offset touched the domain boundary and the
        // ambient band separating them was gone. Measured at target_distance 0.1: 29 of 164
        // box-touching candidates were admitted, leaving 101 offset vertices pinned on the box;
        // with this, zero, and ambient owns the whole boundary again.
        if (!v_in) {
            for (const Tuple& e : get_one_ring_edges_for_vertex(v)) {
                if (!e.switch_face(*this) && get_tags(e.fid(*this)).count(tag) != 0) {
                    v_in = true;
                    break;
                }
            }
        }
        if (v_in) {
            boundary_vs.push_back(v);
        }
    }

    // collect boundary edges
    std::map<simplex::Edge, bool> boundary_edges;
    for (int i = 0; i < 3; i++) {
        simplex::Edge e(vs[i], vs[(i + 1) % 3]);
        auto [etup, e_id] = tuple_from_edge(e.vertices());

        auto other = etup.switch_face(*this);
        if (other) {
            bool f1_in = get_tags(etup.fid(*this)).count(tag) != 0;
            bool f2_in = get_tags(other.value().fid(*this)).count(tag) != 0;
            boundary_edges[e] = (f1_in && !f2_in) || (!f1_in && f2_in);
        } else {
            boundary_edges[e] = get_tags(etup.fid(*this)).count(tag) != 0;
        }
    }

    // check criteria (same logic as offset_tri_consistent_topology)
    if (boundary_vs.size() == 3) {
        int num_boundary_edges = 0;
        for (const auto& pair : boundary_edges) {
            if (pair.second) {
                num_boundary_edges++;
            }
        }
        return (num_boundary_edges == 2);
    } else if (boundary_vs.size() == 2) {
        simplex::Edge e(boundary_vs[0], boundary_vs[1]);
        return boundary_edges[e];
    } else {
        return false;
    }
}


} // namespace wmtk::components::topological_offset
