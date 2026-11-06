#include <catch2/catch_test_macros.hpp>

#include <jse/jse.h>
#include <wmtk/components/simwild/SimWildMesh.h>
#include <simwild_spec.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

using namespace wmtk;
using namespace wmtk::components::simwild;

namespace {

/// A flat, open n x n grid on [0, 1]^2: a square outline of 4 * (n - 1) boundary edges.
std::filesystem::path write_open_patch(const int n)
{
    // A name of its own, so concurrent runs sharing the temp directory do not collide.
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("wmtk_simwild_open_patch_" + std::to_string(std::random_device{}()) + ".obj");
    std::ofstream out(path);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            out << "v " << double(i) / (n - 1) << " " << double(j) / (n - 1) << " 0\n";
        }
    }
    const auto id = [n](int i, int j) { return i * n + j + 1; };
    for (int i = 0; i + 1 < n; ++i) {
        for (int j = 0; j + 1 < n; ++j) {
            out << "f " << id(i, j) << " " << id(i + 1, j) << " " << id(i + 1, j + 1) << "\n";
            out << "f " << id(i, j) << " " << id(i + 1, j + 1) << " " << id(i, j + 1) << "\n";
        }
    }
    return path;
}

nlohmann::json params(const std::filesystem::path& input, const bool boundary_envelope)
{
    nlohmann::json j;
    j["application"] = "simwild";
    j["input"] = {input.string()};
    j["preserve_topology"] = false; // the only route that simplifies before insertion
    j["eps_simplify_rel"] = 1e-2;
    j["simplify_boundary_envelope"] = boundary_envelope;
    // So that the optimizer's envelope is the simplification, which boundary_edges() reads; around
    // the input, the default, it is the input as read.
    j["envelope_reference"] = "simplified";
    const auto spec = jse::embed::wmtk_simwild_spec::simwild_spec::spec();
    jse::JSE engine;
    REQUIRE(engine.verify_json(j, spec));
    return engine.inject_defaults(j, spec);
}

/// The boundary edges of the simplified surface read_mesh hands to the optimizer's envelope.
std::vector<std::array<Vector3d, 2>> boundary_edges(const InputData& data)
{
    std::map<std::pair<int, int>, int> count;
    for (int f = 0; f < data.F_envelope.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const int a = data.F_envelope(f, k), b = data.F_envelope(f, (k + 1) % 3);
            ++count[{std::min(a, b), std::max(a, b)}];
        }
    }
    std::vector<std::array<Vector3d, 2>> out;
    for (const auto& [e, c] : count) {
        if (c == 1) {
            out.push_back({{data.V_envelope.row(e.first), data.V_envelope.row(e.second)}});
        }
    }
    return out;
}

double dist_to_square_outline(const Vector3d& p)
{
    const double x = std::clamp(p[0], 0.0, 1.0), y = std::clamp(p[1], 0.0, 1.0);
    const double in_plane = std::min({x, 1 - x, y, 1 - y});
    const double off = (Vector3d(x, y, 0) - p).norm();
    return std::sqrt(in_plane * in_plane + off * off);
}

/// A tagged tet mesh of [0, 1]^3, as a tagged .msh gives one: n x n x 2 boxes, each split into
/// six tets along its diagonal (Kuhn's split, which conforms across boxes), the lower layer
/// tagged 0 and the upper 1. Their interface, the surface simwild holds, is the sheet z = 1/2,
/// with its centre vertex `tip` raised by `spike`: below 1/2, the layers' height, a vertical move
/// keeps every tet positive.
struct TaggedSlab
{
    MatrixXd V;
    MatrixXi T;
    MatrixSi T_tags;
    int tip = -1;
};

TaggedSlab tagged_slab(const int n, const double spike)
{
    TaggedSlab m;
    const auto id = [n](const std::array<int, 3>& c) {
        return (c[2] * (n + 1) + c[1]) * (n + 1) + c[0];
    };
    m.V.resize(3 * (n + 1) * (n + 1), 3);
    for (int k = 0; k < 3; ++k) {
        for (int j = 0; j <= n; ++j) {
            for (int i = 0; i <= n; ++i) {
                m.V.row(id({{i, j, k}})) << double(i) / n, double(j) / n, 0.5 * k;
            }
        }
    }
    std::vector<std::array<int, 4>> tets;
    std::vector<Eigen::Triplet<int>> tags;
    for (int k = 0; k < 2; ++k) {
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                std::array<int, 3> axes = {{0, 1, 2}};
                do {
                    // A path from the box's corner to the opposite one, one axis at a time.
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int, 4> t;
                    t[0] = id(c);
                    for (int s = 0; s < 3; ++s) {
                        ++c[axes[s]];
                        t[s + 1] = id(c);
                    }
                    const Vector3d a = m.V.row(t[0]).transpose(), b = m.V.row(t[1]).transpose(),
                                   p = m.V.row(t[2]).transpose(), q = m.V.row(t[3]).transpose();
                    if ((b - a).cross(p - a).dot(q - a) < 0) std::swap(t[2], t[3]);
                    tags.emplace_back(int(tets.size()), k, 1);
                    tets.push_back(t);
                } while (std::next_permutation(axes.begin(), axes.end()));
            }
        }
    }
    m.T.resize(tets.size(), 4);
    for (size_t r = 0; r < tets.size(); ++r) {
        m.T.row(r) << tets[r][0], tets[r][1], tets[r][2], tets[r][3];
    }
    m.T_tags.resize(tets.size(), 2);
    m.T_tags.setFromTriplets(tags.begin(), tags.end());
    m.tip = id({{n / 2, n / 2, 1}});
    m.V(m.tip, 2) += spike; // after the orientations: init_from_image() refuses an inverted tet
    return m;
}

} // namespace

TEST_CASE(
    "simwild-simplification-boundary-envelope",
    "[simwild][simplify][boundary]" + release_only)
{
    // With topology preservation off the input is simplified before insertion, and its open
    // boundary either frozen or held in a tube of order2_envelope_ratio * eps_simplify around the
    // input's boundary edges.
    const int n = 21;
    const std::filesystem::path path = write_open_patch(n);

    SECTION("frozen when simplify_boundary_envelope is off")
    {
        const InputData data = read_mesh({path.string()}, "", params(path, false));
        // Every outline vertex survives, so the outline keeps all of its edges.
        CHECK(boundary_edges(data).size() == size_t(4 * (n - 1)));
    }
    SECTION("coarsened in the tube when it is on")
    {
        const nlohmann::json j = params(path, true);
        const InputData data = read_mesh({path.string()}, "", j);
        const double diag = std::sqrt(2.0);
        const double r = double(j["eps_simplify_rel"]) * diag * double(j["order2_envelope_ratio"]);
        const auto be = boundary_edges(data);
        REQUIRE(!be.empty());
        for (const auto& e : be) {
            for (int k = 0; k <= 16; ++k) {
                CHECK(dist_to_square_outline(e[0] + (e[1] - e[0]) * (k / 16.0)) <= r);
            }
        }
        CHECK(be.size() < size_t(4 * (n - 1)) / 2);
    }

    std::filesystem::remove(path);
}

TEST_CASE(
    "simwild-simplification-label-envelopes-around-the-input",
    "[simwild][simplify][envelope]" + release_only)
{
    // A tagged mesh input holds its own surface faces: given no envelope, or one they are not in,
    // init_surfaces_and_boundaries() builds the envelope around them, each row with its labels.
    // Under envelope_reference "input", simplify() rebuilds every envelope around those rows after
    // collapsing: the union, and each label's. Until 2026-11 the rows had no labels there, and
    // init_label_envelopes() read that as "around the mesh's faces as they are now" -- after the
    // simplification, the simplified faces, while the union was around the input.
    //
    // The interface carries both labels, so each label's envelope is around all of it. A spike of
    // height 2 eps stands on it, on a base of inradius 0.044, below eps_simplify / sqrt(3) = 0.052
    // (fast-envelope's prisms are eps / sqrt(3) thick): the simplification may flatten it, and
    // the REQUIRE below says it did. Its tip then lies on the input and 2 eps from the simplified
    // surface: an envelope at eps around the input holds every point within eps / sqrt(3) of it,
    // one around the simplified surface no point beyond eps of that. Before the fix the tip was
    // outside both labels' envelopes and inside the union, with and without an envelope given.
    const int n = 16; // the sheet's spacing 1/16
    const double eps = 0.1, eps_simplify = 0.09; // eps_simplify below eps, as "input" requires
    const TaggedSlab slab = tagged_slab(n, 2 * eps);
    const Vector3d tip = slab.V.row(slab.tip).transpose();

    for (const bool envelope_given : {false, true}) {
        CAPTURE(envelope_given);
        nlohmann::json j;
        j["application"] = "simwild";
        j["input"] = {"slab"};
        j["eps"] = eps;
        j["eps_simplify"] = eps_simplify;
        // Every edge a candidate: the simplification coarsens as far as the envelope and the
        // quality allow.
        j["length"] = 1.0;
        const auto spec = jse::embed::wmtk_simwild_spec::simwild_spec::spec();
        jse::JSE engine;
        REQUIRE(engine.verify_json(j, spec));
        Parameters simwild_params(engine.inject_defaults(j, spec));
        simwild_params.output_enabled = false;
        simwild_params.init(slab.V.colwise().minCoeff(), slab.V.colwise().maxCoeff());

        // As run_3D() drives a tagged mesh input, on the defaults: simplify, preserve_topology,
        // envelope_reference "input", per_input_envelopes.
        SimWildMesh mesh(simwild_params, simwild_params.eps, simwild_params.NUM_THREADS);
        if (envelope_given) {
            // A triangle away from the mesh: its surface faces start outside, so the envelope is
            // rebuilt around them.
            MatrixXd V(3, 3);
            V << 5, 5, 5, 6, 5, 5, 5, 6, 5;
            MatrixXi F(1, 3);
            F << 0, 1, 2;
            mesh.init_envelope(V, F);
        }
        mesh.init_from_image(slab.V, slab.T, slab.T_tags, {"lower", "upper"});
        mesh.consolidate_mesh();
        mesh.simplify();

        std::vector<Vector3d> V_now(mesh.vert_capacity());
        for (size_t v = 0; v < V_now.size(); ++v) V_now[v] = mesh.m_vertex_attribute[v].m_posf;
        std::vector<Vector3i> F_now;
        for (const auto& f :
             mesh.get_faces_by_condition([](const auto& f) { return f.m_is_surface_fs; })) {
            F_now.emplace_back(int(f[0]), int(f[1]), int(f[2]));
        }
        Envelope simplified;
        simplified.init(V_now, F_now, eps);
        const double d = std::sqrt(simplified.squared_distance(tip));
        CAPTURE(d / eps);
        REQUIRE(d > 1.5 * eps);

        REQUIRE(mesh.m_input_envelopes.size() == 2);
        CHECK(!mesh.m_envelope->is_outside(tip));
        for (size_t b = 0; b < 2; ++b) {
            CAPTURE(b);
            CHECK(!mesh.m_input_envelopes.member(b)->is_outside(tip));
        }
    }
}
