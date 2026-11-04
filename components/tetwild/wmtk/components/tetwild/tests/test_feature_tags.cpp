#include <wmtk/TetMesh.h>
#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <wmtk/components/tetwild/Parameters.h>
#include <wmtk/components/tetwild/TetWildMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>
#include <set>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

using namespace wmtk;
using namespace components::tetwild;

// Feature-edge tag PROPAGATION through the shared operations.
//
// The failure this hunts is silent: an edge attribute's slot is (lowest incident tet) * 6 +
// local index, so a neighbouring operation that deletes that tet moves the slot, and a
// propagation bug leaves the tag at the dead one -- nothing crashes, the curve just loses a
// link. So the assertions here are about the TAGGED SET as a whole, re-read from the live
// slots after every single operation:
//
//  * splits + swaps only (phase 1): the tagged set must still tile every input edge exactly
//    -- collinear, contiguous, endpoint to endpoint, in rationals. Splits along the curve
//    subdivide the tiling legally; swaps that would delete a tagged edge are vetoed; nothing
//    else may change it. A dropped tag breaks coverage, a stale slot breaks collinearity.
//  * with collapses (phase 2): collapses legally erode the curve (no anchors yet), so the
//    invariant weakens to: every tagged edge lies EXACTLY on some input segment, and both its
//    endpoints carry the vertex flag. Tags may vanish; they may never leak off the curve.

namespace {

struct FeatureScene
{
    Parameters params; // must outlive the mesh (held by reference)
    std::shared_ptr<TetWildMesh> mesh;
    std::vector<Vector3d> fe_verts;
    std::vector<std::array<size_t, 2>> fe;
    std::shared_ptr<components::shortest_edge_collapse::ShortestEdgeCollapse> surf;
};

FeatureScene build_scene()
{
    FeatureScene sc;
    // Cube [0,2]^3 with an interior 2-segment feature chain.
    MatrixXd V(8, 3);
    V << 0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
        0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2;
    MatrixXi F(12, 3);
    F << 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, //
        2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6, 5, 0, 4, 7, 0, 7, 3;

    std::vector<Vector3d> vertices;
    std::vector<std::array<size_t, 3>> faces;
    VF_to_vectors(V, F, vertices, faces);
    sc.params.init(vertices, faces);

    sc.surf =
        std::make_shared<components::shortest_edge_collapse::ShortestEdgeCollapse>(vertices, 0);
    {
        std::vector<size_t> frozen_verts;
        sc.surf->create_mesh(vertices.size(), faces, frozen_verts, 0.1);
    }
    auto* env_ptr = &sc.surf->m_envelope;
    const std::shared_ptr<Envelope> env(env_ptr, [](Envelope*) {});

    sc.fe_verts = {{0.5, 0.5, 0.5}, {1.0, 1.0, 1.0}, {1.5, 1.0, 1.5}};
    sc.fe = {{{0, 1}}, {{1, 2}}};

    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;
    utils::EmbedFeaturesResult features_out;
    {
        TetWildMesh insertion_mesh(sc.params, env, 0);
        insertion_mesh.insertion_by_volumeremesher(
            vertices,
            faces,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            /*tet_face_orientation=*/nullptr,
            sc.fe_verts,
            sc.fe,
            {},
            &features_out);
    }
    sc.mesh = std::make_shared<TetWildMesh>(sc.params, env, 0);
    {
        std::vector<Eigen::Vector2i> fe_env(sc.fe.size());
        for (size_t i = 0; i < sc.fe.size(); ++i) {
            fe_env[i] = Eigen::Vector2i(int(sc.fe[i][0]), int(sc.fe[i][1]));
        }
        sc.mesh->m_feature_envelope = std::make_shared<Envelope>();
        sc.mesh->m_feature_envelope->init(sc.fe_verts, fe_env, 0.05);
    }
    // The init anchors the chain's endpoints, its tagged edges' ends, in a ball of the tube's
    // radius.
    sc.params.feature_envelope_ratio = 0.05 / (sc.params.epsr * sc.params.diag_l);
    sc.mesh->init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface,
        /*tet_face_orientation=*/nullptr,
        &features_out);
    return sc;
}

/// All currently tagged edges, re-read from the live slots.
std::vector<std::array<size_t, 2>> tagged_edges(TetWildMesh& m)
{
    std::vector<std::array<size_t, 2>> out;
    for (const auto& e : m.get_edges()) {
        if (!m.m_feature_edge_attribute[e.eid(m)].is_tagged()) {
            continue;
        }
        size_t a = e.vid(m);
        size_t b = e.switch_vertex(m).vid(m);
        if (a > b) {
            std::swap(a, b);
        }
        out.push_back({{a, b}});
    }
    return out;
}

/// Exact parameter of vertex `vid` on segment (A,B), or nullopt if not exactly on it.
std::optional<Rational>
param_on_segment(TetWildMesh& m, const size_t vid, const Vector3d& A, const Vector3d& B)
{
    const Vector3r a{Rational(A[0]), Rational(A[1]), Rational(A[2])};
    const Vector3r d{Rational(B[0] - A[0]), Rational(B[1] - A[1]), Rational(B[2] - A[2])};
    const Vector3r p = m.m_vertex_attribute[vid].pos() - a;
    const Vector3r c = p.cross(d);
    if (c[0] != Rational(0) || c[1] != Rational(0) || c[2] != Rational(0)) {
        return std::nullopt;
    }
    const Rational t = p.dot(d) / d.dot(d);
    if (t < Rational(0) || t > Rational(1)) {
        return std::nullopt;
    }
    return t;
}

/// Strict invariant (no collapses have run): every tagged edge lies EXACTLY on some input
/// segment, with flagged endpoints.
void require_tags_on_curve(FeatureScene& sc)
{
    for (const auto& e : tagged_edges(*sc.mesh)) {
        bool on_some_segment = false;
        for (const auto& seg : sc.fe) {
            const auto t0 =
                param_on_segment(*sc.mesh, e[0], sc.fe_verts[seg[0]], sc.fe_verts[seg[1]]);
            const auto t1 =
                param_on_segment(*sc.mesh, e[1], sc.fe_verts[seg[0]], sc.fe_verts[seg[1]]);
            if (t0.has_value() && t1.has_value()) {
                on_some_segment = true;
                break;
            }
        }
        REQUIRE(on_some_segment);
        REQUIRE(sc.mesh->vertex_has_feature_edge(e[0]));
        REQUIRE(sc.mesh->vertex_has_feature_edge(e[1]));
    }
}

/// Phase-2 invariant: collapses may re-route the curve inside its tube, so exact
/// collinearity no longer holds -- containment in the feature envelope is the contract the
/// collapse guard enforces, and is what is asserted. The anchors add the coverage half:
/// every anchor keeps a live vertex within its ball, after every single collapse.
void require_tags_in_tube(FeatureScene& sc)
{
    TetWildMesh& m = *sc.mesh;
    for (const auto& e : tagged_edges(m)) {
        REQUIRE(!sc.mesh->m_feature_envelope->is_outside(
            std::array<Eigen::Vector3d, 2>{
                {m.m_vertex_attribute[e[0]].m_posf, m.m_vertex_attribute[e[1]].m_posf}}));
        REQUIRE(m.vertex_has_feature_edge(e[0]));
        REQUIRE(m.vertex_has_feature_edge(e[1]));
    }
    const auto [kept, total] = m.feature_retention();
    REQUIRE(total == 2);
    REQUIRE(kept == 2);
}

/// Phase-1 invariant: the tagged set tiles every input segment exactly.
void require_full_tiling(FeatureScene& sc)
{
    require_tags_on_curve(sc);
    const auto tagged = tagged_edges(*sc.mesh);
    for (const auto& seg : sc.fe) {
        const Vector3d& A = sc.fe_verts[seg[0]];
        const Vector3d& B = sc.fe_verts[seg[1]];
        std::vector<std::pair<Rational, Rational>> intervals;
        for (const auto& e : tagged) {
            const auto t0 = param_on_segment(*sc.mesh, e[0], A, B);
            const auto t1 = param_on_segment(*sc.mesh, e[1], A, B);
            if (!t0.has_value() || !t1.has_value()) {
                continue;
            }
            auto lo = *t0, hi = *t1;
            if (hi < lo) {
                std::swap(lo, hi);
            }
            if (lo == hi) {
                continue; // an endpoint shared with the neighbouring input segment
            }
            intervals.emplace_back(lo, hi);
        }
        REQUIRE(!intervals.empty());
        std::sort(intervals.begin(), intervals.end(), [](const auto& x, const auto& y) {
            return x.first < y.first;
        });
        REQUIRE(intervals.front().first == Rational(0));
        for (size_t i = 1; i < intervals.size(); ++i) {
            REQUIRE(intervals[i].first == intervals[i - 1].second);
        }
        REQUIRE(intervals.back().second == Rational(1));
    }
}

} // namespace

TEST_CASE("feature-tag-propagation-fuzz", "[tetwild_operation][features]" + release_only)
{
    FeatureScene sc = build_scene();
    TetWildMesh& m = *sc.mesh;
    require_full_tiling(sc);

    std::mt19937 rng(7);
    // Half the picks are edges incident to a curve vertex: slot churn only endangers tags
    // near the curve, and uniform picks on a growing mesh rarely go near it. Verified by
    // negative control -- with uniform picks, a disabled split restore survived the fuzz.
    const auto random_edge = [&]() -> TetMesh::Tuple {
        if (rng() % 2 == 0) {
            const auto tagged = tagged_edges(m);
            if (!tagged.empty()) {
                const auto& te = tagged[rng() % tagged.size()];
                const auto ring = m.get_one_ring_tets_for_vertex(m.tuple_from_vertex(te[0]));
                if (!ring.empty()) {
                    const auto& t = ring[rng() % ring.size()];
                    return m.tuple_from_edge(t.tid(m), int(rng() % 6));
                }
            }
        }
        const auto edges = m.get_edges();
        return edges[rng() % edges.size()];
    };

    // Phase 1: splits and swaps only. The curve must stay fully tiled after every operation.
    size_t done = 0;
    for (int i = 0; i < 1500 && done < 250; ++i) {
        const TetMesh::Tuple e = random_edge();
        if (!e.is_valid(m)) {
            continue;
        }
        bool changed = false;
        std::vector<TetMesh::Tuple> new_tets;
        switch (rng() % 4) {
        case 0: changed = m.split_edge(e, new_tets); break;
        case 1: changed = m.swap_edge(e, new_tets); break;
        case 2: changed = m.swap_edge_44(e, new_tets); break;
        case 3: changed = m.swap_edge_56(e, new_tets); break;
        }
        if (!changed) {
            continue;
        }
        ++done;
        require_full_tiling(sc);
    }
    REQUIRE(done >= 100); // the fuzz must actually have exercised operations

    // Phase 2: collapses join. Tags may erode; they may never leak off the curve.
    size_t collapsed = 0;
    for (int i = 0; i < 400 && collapsed < 40; ++i) {
        const TetMesh::Tuple e = random_edge();
        if (!e.is_valid(m)) {
            continue;
        }
        std::vector<TetMesh::Tuple> new_tets;
        if (!m.collapse_edge(e, new_tets)) {
            continue;
        }
        ++collapsed;
        require_tags_in_tube(sc);
    }
    REQUIRE(collapsed >= 10);
}

TEST_CASE("open-boundary-tags-drop-when-glued", "[tetwild_operation][order2]" + release_only)
{
    // Two open rectangles in z = 0, [0, 0.5 - w/2] x [0, 1] and [0.5 + w/2, 1] x [0, 1], with a
    // slit of width w = r / 4 between them, r the open-boundary tube's half-width. Without
    // preserve_topology the collapses may glue the slit shut, and the edges along the seam then
    // have two faces: they are open boundaries no longer, so they lose their order2 tag
    // (drop_stale_order2_tags()). Every edge still tagged at the end has exactly one face.
    for (const double epsr : {1e-3, 2e-3, 1e-2}) {
        CAPTURE(epsr);
        Parameters params;
        params.epsr = epsr;
        const double r = params.order2_envelope_ratio * epsr * std::sqrt(2.0); // bbox [0,1]^2
        const double a = 0.5 - r / 8, b = 0.5 + r / 8;
        const std::vector<Vector3d> v = {
            {0, 0, 0},
            {a, 0, 0},
            {a, 1, 0},
            {0, 1, 0},
            {b, 0, 0},
            {1, 0, 0},
            {1, 1, 0},
            {b, 1, 0}};
        const std::vector<std::array<size_t, 3>> f = {
            {{0, 1, 2}},
            {{0, 2, 3}},
            {{4, 5, 6}},
            {{4, 6, 7}}};
        params.init(v, f);
        std::vector<Eigen::Vector3i> F;
        for (const auto& t : f) F.emplace_back(int(t[0]), int(t[1]), int(t[2]));
        auto env = std::make_shared<Envelope>();
        env->init(v, F, params.eps);

        std::vector<Vector3r> v_rational;
        std::vector<std::array<size_t, 3>> facets;
        std::vector<bool> is_v_on_input;
        std::vector<std::array<size_t, 4>> tets;
        std::vector<bool> tet_face_on_input_surface;
        std::vector<int8_t> tet_face_orientation;
        {
            TetWildMesh mesh_insertion(params, env, 0);
            mesh_insertion.insertion_by_volumeremesher(
                v,
                f,
                v_rational,
                facets,
                is_v_on_input,
                tets,
                tet_face_on_input_surface,
                &tet_face_orientation);
        }
        TetWildMesh mesh(params, env, 0);
        mesh.init_from_Volumeremesher(
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            &tet_face_orientation);
        REQUIRE(mesh.m_track_feature_edges);
        mesh.mesh_improvement(3);

        size_t n_tagged = 0, n_stale = 0;
        for (const auto& e : mesh.get_edges()) {
            if (!mesh.m_feature_edge_attribute[e.eid(mesh)].has(
                    TetWildMesh::FeatureEdgeAttributes::order2)) {
                continue;
            }
            ++n_tagged;
            const size_t v0 = e.vid(mesh), v1 = e.switch_vertex(mesh).vid(mesh);
            n_stale += mesh.get_num_surface_faces_for_edge({{v0, v1}}) != 1;
        }
        CAPTURE(n_tagged);
        CHECK(n_tagged > 0);
        CHECK(n_stale == 0);
    }
}
