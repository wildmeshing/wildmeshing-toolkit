#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/components/surface_map/SurfaceMap.hpp>
#include <wmtk/components/surface_map/surface_io.hpp>
#include <wmtk/components/surface_map/transfer.hpp>
#include <wmtk/history/Replay.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/predicates.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>
#include <random>
#include <set>

using namespace wmtk;
using namespace wmtk::components;
using surface_map::SurfaceMap;
using surface_map::SurfacePoint;

namespace {

/// The history of an isotropic remeshing of `model` (a path, or a name under data/models).
history::SurfaceHistory remesh(const std::string& model, int threads, double len_factor, int its)
{
    MatrixXd V;
    MatrixXi F;
    const std::string path = std::filesystem::path(model).is_absolute()
                                 ? model
                                 : std::string(WMTK_DATA_DIR) + "/models/" + model;
    wmtk::io::read_triangle_mesh(path, V, F, 0);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    VF_to_vectors(V, F, verts, tris);
    isotropic_remeshing::IsotropicRemeshing m(verts, threads);
    m.create_mesh(verts.size(), tris, {}, true, 0, 0);
    const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
    m.uniform_remeshing(len_factor * m.average_len_valen()[0], its);
    m.consolidate_mesh();
    m.stop_history();
    return rec->history();
}

Eigen::Vector3d position(
    const SurfacePoint& p,
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 3>>& F)
{
    const auto& f = F[p.face];
    return p.bary[0] * V[f[0]] + p.bary[1] * V[f[1]] + p.bary[2] * V[f[2]];
}

/// `n` random points strictly inside every live face.
std::vector<SurfacePoint> sample(const std::vector<uint8_t>& alive, int n, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(0.02, 0.98);
    std::vector<SurfacePoint> pts;
    for (uint64_t f = 0; f < alive.size(); ++f) {
        if (!alive[f]) continue;
        for (int k = 0; k < n; ++k) {
            double a = u(rng), b = u(rng);
            if (a + b > 1) {
                a = 1 - a;
                b = 1 - b;
            }
            pts.push_back({f, Eigen::Vector3d(1 - a - b, a, b)});
        }
    }
    return pts;
}

/// How far a round trip may come back from where it started: rounding, through thousands of local
/// maps whose layouts may hold thin triangles. A map that is not a bijection misses by far more.
constexpr double kRoundTrip = 1e-8;

/// Maps `pts` there and back and returns the largest 3D displacement; every point has to land
/// on a live face both ways.
double round_trip(
    const SurfaceMap& map,
    const std::vector<SurfacePoint>& pts,
    const bool from_input,
    size_t& warnings)
{
    const auto& V = from_input ? map.input_vertices() : map.output_vertices();
    const auto& F = from_input ? map.input_faces() : map.output_faces();
    const auto& other_alive = from_input ? map.output_face_alive() : map.input_face_alive();
    const auto there = from_input ? map.to_output(pts) : map.to_input(pts);
    warnings = map.map_warnings().total();
    for (const auto& p : there) {
        REQUIRE(p.face < other_alive.size());
        REQUIRE(other_alive[p.face]);
    }
    const auto back = from_input ? map.to_input(there) : map.to_output(there);
    warnings += map.map_warnings().total();
    double err = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(back[i].face < F.size());
        err = std::max(err, (position(back[i], V, F) - position(pts[i], V, F)).norm());
    }
    return err;
}

} // namespace

TEST_CASE("surface-map-round-trip-remeshing", "[surface_map]")
{
    // A closed manifold remeshed with the link condition on: every local map is a bijection, so
    // the map is one, and any point comes back to where it was.
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        const auto h = remesh("sphere.obj", threads, 0.6, 2);
        const SurfaceMap map(h);
        CHECK(map.build_warnings().total() == 0);
        CHECK(map.cells_by_kind()[size_t(history::EditKind::collapse_edge)] > 0);
        CHECK(map.cells_by_kind()[size_t(history::EditKind::swap_edge)] > 0);

        size_t warnings = 0;
        const double in_err = round_trip(map, sample(map.input_face_alive(), 3, 1), true, warnings);
        CHECK(warnings == 0);
        CHECK(in_err < kRoundTrip);

        const double out_err =
            round_trip(map, sample(map.output_face_alive(), 3, 2), false, warnings);
        CHECK(warnings == 0);
        CHECK(out_err < kRoundTrip);
    }
}

TEST_CASE("surface-map-round-trip-remeshing-bunny", "[surface_map][.]")
{
    // The bunny has boundary loops. Its boundary vertices are frozen, but the vertices splits
    // insert on boundary edges are not, and they can collapse along the boundary afterwards: the
    // chord case runs. Where such a vertex has a single triangle, the collapse flattens it onto
    // the chord -- the one way a 2D map breaks -- and the map reports it, with the atlas it
    // inverts, and nothing else. Hidden, a few seconds.
    const auto h = remesh("bunny.obj", 8, 0.8, 3);
    const SurfaceMap map(h);
    const auto& w = map.build_warnings();
    INFO(
        "not a disk " << w.not_a_disk << ", flat " << w.flat_triangle << ", inverted " << w.inverted
                      << ", merged " << w.merged_faces);
    CHECK(map.curve_collapses() > 0);
    CHECK(w.not_a_disk == 0);
    CHECK(w.merged_faces == 0);
    CHECK(w.inverted <= w.flat_triangle);
    size_t warnings = 0;
    const double err = round_trip(map, sample(map.output_face_alive(), 1, 3), false, warnings);
    CHECK(warnings == 0);
    CHECK(err < kRoundTrip);
}

TEST_CASE("surface-map-transfer-face-labels", "[surface_map]")
{
    // Label the sphere's faces by the side of x = 0 their centroid is on, and by octant; remesh
    // from the labelled gmsh file while recording; carry the labels onto the output. An output
    // face clearly on one side gets that side's label. Near x = 0 an output face can straddle
    // the boundary, since the remeshing does not keep it as a curve.
    MatrixXd V;
    MatrixXi F;
    wmtk::io::read_triangle_mesh(std::string(WMTK_DATA_DIR) + "/models/sphere.obj", V, F, 0);
    surface_map::SurfaceData in;
    for (int i = 0; i < V.rows(); ++i) in.vertices.push_back(V.row(i).transpose());
    auto& side = in.face_fields["side"];
    auto& octant = in.face_fields["octant"];
    const auto centroid = [](const surface_map::SurfaceData& s, size_t f) {
        return (s.vertices[s.faces[f][0]] + s.vertices[s.faces[f][1]] + s.vertices[s.faces[f][2]]) /
               3;
    };
    for (int f = 0; f < F.rows(); ++f) {
        in.faces.push_back({{uint64_t(F(f, 0)), uint64_t(F(f, 1)), uint64_t(F(f, 2))}});
        const Eigen::Vector3d c = centroid(in, size_t(f));
        side.push_back({c.x() > 0 ? 1. : 0.});
        octant.push_back({double((c.x() > 0) + 2 * (c.y() > 0) + 4 * (c.z() > 0))});
    }
    const auto path =
        (std::filesystem::temp_directory_path() / "wmtk_surface_map_labels.msh").string();
    surface_map::write_surface_msh(path, in);
    const surface_map::SurfaceData reread = surface_map::read_surface_msh(path);
    REQUIRE(reread.faces == in.faces);
    REQUIRE(reread.face_fields == in.face_fields);

    const auto h = remesh(path, 0, 0.6, 2);
    std::filesystem::remove(path);
    const SurfaceMap map(h);
    std::string why;
    INFO(why);
    REQUIRE(surface_map::matches_input(map, reread, why));

    const surface_map::SurfaceData out = surface_map::transfer_fields(map, reread);
    CHECK(map.map_warnings().total() == 0);
    REQUIRE(out.face_fields.at("side").size() == out.faces.size());
    const auto longest_edge = [](const surface_map::SurfaceData& s) {
        double h = 0;
        for (const auto& t : s.faces) {
            for (int k = 0; k < 3; ++k) {
                h = std::max(h, (s.vertices[t[k]] - s.vertices[t[(k + 1) % 3]]).norm());
            }
        }
        return h;
    };
    // The labelled side ends within one input face of x = 0, and an output face reaches at most
    // its longest edge from its centroid.
    const double margin = longest_edge(in) + longest_edge(out);
    size_t far = 0, wrong = 0;
    for (size_t f = 0; f < out.faces.size(); ++f) {
        const Eigen::Vector3d c = centroid(out, f);
        if (std::abs(c.x()) < margin) continue;
        ++far;
        wrong += out.face_fields.at("side")[f][0] != (c.x() > 0 ? 1. : 0.);
    }
    INFO(far << " of " << out.faces.size() << " faces clearly on one side");
    CHECK(far > out.faces.size() / 3);
    CHECK(wrong == 0);
}

namespace {

/// The sphere, labelled per face by the sides of three tilted planes through the centroid. They
/// cut across the sphere's latitude-longitude grid, so the label boundaries zigzag along its
/// edges, with triangles holding two boundary edges at the corners.
surface_map::SurfaceData labelled_sphere()
{
    MatrixXd V;
    MatrixXi F;
    wmtk::io::read_triangle_mesh(std::string(WMTK_DATA_DIR) + "/models/sphere.obj", V, F, 0);
    surface_map::SurfaceData s;
    for (int i = 0; i < V.rows(); ++i) s.vertices.push_back(V.row(i).transpose());
    for (int f = 0; f < F.rows(); ++f) {
        s.faces.push_back({{uint64_t(F(f, 0)), uint64_t(F(f, 1)), uint64_t(F(f, 2))}});
        const Eigen::Vector3d c =
            (s.vertices[s.faces[f][0]] + s.vertices[s.faces[f][1]] + s.vertices[s.faces[f][2]]) / 3;
        const double x = c.x() + 0.37 * c.y() + 0.21 * c.z();
        const double y = c.y() - 0.29 * c.z() + 0.17 * c.x();
        const double z = c.z() + 0.43 * c.x() - 0.13 * c.y();
        s.face_fields["side"].push_back({x > 0 ? 1. : 0.});
        s.face_fields["octant"].push_back({double((x > 0) + 2 * (y > 0) + 4 * (z > 0))});
    }
    return s;
}

/// Edges between two faces with different values of any field.
std::vector<std::array<size_t, 2>> label_boundaries(const surface_map::SurfaceData& s)
{
    std::map<std::array<size_t, 2>, std::vector<size_t>> edge_faces;
    for (size_t f = 0; f < s.faces.size(); ++f) {
        for (int k = 0; k < 3; ++k) {
            size_t a = s.faces[f][k], b = s.faces[f][(k + 1) % 3];
            if (a > b) std::swap(a, b);
            edge_faces[{{a, b}}].push_back(f);
        }
    }
    std::vector<std::array<size_t, 2>> out;
    for (const auto& [e, fs] : edge_faces) {
        if (fs.size() != 2) continue;
        for (const auto& [name, field] : s.face_fields) {
            if (field[fs[0]] != field[fs[1]]) {
                out.push_back(e);
                break;
            }
        }
    }
    return out;
}

} // namespace

TEST_CASE("surface-map-conforming-labels", "[surface_map]")
{
    // With the label boundaries kept as feature curves, every output face lies inside one label
    // region: points anywhere in it map to input faces of one label, the labels hold exactly,
    // and the output's label boundaries are its feature curves.
    const surface_map::SurfaceData in = labelled_sphere();
    std::vector<Eigen::Vector3d> verts(in.vertices.begin(), in.vertices.end());
    std::vector<std::array<size_t, 3>> tris;
    for (const auto& f : in.faces) tris.push_back({{size_t(f[0]), size_t(f[1]), size_t(f[2])}});
    const auto features = label_boundaries(in);
    REQUIRE(!features.empty());

    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        isotropic_remeshing::IsotropicRemeshing m(verts, threads);
        m.create_mesh(verts.size(), tris, {}, true, 0, 0);
        m.set_feature_edges(features);
        const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
        for (const auto& e : m.feature_edges())
            rec->history().feature_edges.push_back({{e[0], e[1]}});
        m.uniform_remeshing(0.6 * m.average_len_valen()[0], 2);
        m.consolidate_mesh();
        m.stop_history();
        const auto& h = rec->history();

        // The history carries the curves exactly as the mesh does.
        history::Replay r(h);
        r.apply_all(h);
        std::set<std::array<uint64_t, 2>> mesh_features;
        for (const auto& e : m.feature_edges()) mesh_features.insert({{e[0], e[1]}});
        CHECK(r.feature_edges() == mesh_features);
        CHECK(mesh_features.size() > features.size() / 4);

        const SurfaceMap map(h);
        INFO(
            "build: not a disk " << map.build_warnings().not_a_disk << ", flat "
                                 << map.build_warnings().flat_triangle << ", inverted "
                                 << map.build_warnings().inverted);
        CHECK(map.build_warnings().total() == 0);
        // Collapses along the curves, the ears' among them, each split a triangle virtually:
        // the map stays a bijection, both ways.
        CHECK(map.curve_collapses() > 0);
        size_t trip_warnings = 0;
        CHECK(
            round_trip(map, sample(map.input_face_alive(), 3, 5), true, trip_warnings) <
            kRoundTrip);
        CHECK(trip_warnings == 0);
        CHECK(
            round_trip(map, sample(map.output_face_alive(), 3, 6), false, trip_warnings) <
            kRoundTrip);
        CHECK(trip_warnings == 0);

        // Four points in every output face, mapped to the input.
        std::vector<SurfacePoint> pts;
        std::vector<uint64_t> out_faces;
        for (uint64_t f = 0; f < map.output_face_alive().size(); ++f) {
            if (!map.output_face_alive()[f]) continue;
            out_faces.push_back(f);
            pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
            pts.push_back({f, Eigen::Vector3d(0.8, 0.1, 0.1)});
            pts.push_back({f, Eigen::Vector3d(0.1, 0.8, 0.1)});
            pts.push_back({f, Eigen::Vector3d(0.1, 0.1, 0.8)});
        }
        const auto src = map.to_input(pts);
        CHECK(map.map_warnings().total() == 0);
        size_t mixed = 0;
        for (size_t i = 0; i < out_faces.size(); ++i) {
            const auto label = [&](size_t k) {
                return in.face_fields.at("octant")[src[4 * i + k].face];
            };
            if (label(1) != label(0) || label(2) != label(0) || label(3) != label(0)) ++mixed;
        }
        CHECK(mixed == 0);

        // The transferred labels change exactly across feature edges.
        const surface_map::SurfaceData out = surface_map::transfer_fields(map, in);
        std::map<std::array<uint64_t, 2>, std::vector<size_t>> edge_faces;
        for (size_t f = 0; f < out.faces.size(); ++f) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = out.faces[f][k], b = out.faces[f][(k + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_faces[{{a, b}}].push_back(f);
            }
        }
        size_t label_edges = 0, label_edges_not_feature = 0;
        for (const auto& [e, fs] : edge_faces) {
            if (fs.size() != 2) continue;
            if (out.face_fields.at("octant")[fs[0]] == out.face_fields.at("octant")[fs[1]])
                continue;
            ++label_edges;
            // out numbers vertices by rank, as the consolidated mesh does.
            if (!mesh_features.count(e)) ++label_edges_not_feature;
        }
        CHECK(label_edges > 0);
        CHECK(label_edges_not_feature == 0);

        // No triangle holds both curve edges of a vertex inside a curve -- an "ear", which
        // smoothing along the curve would flatten. Where curves meet, a corner triangle may, and
        // so may a triangle with curves all round, a region of its own.
        std::map<uint64_t, size_t> curve_degree;
        for (const auto& e : mesh_features) {
            ++curve_degree[e[0]];
            ++curve_degree[e[1]];
        }
        size_t ears = 0;
        for (const auto& t : out.faces) {
            for (int k = 0; k < 3; ++k) {
                const uint64_t b = t[k], a = t[(k + 2) % 3], c = t[(k + 1) % 3];
                const auto key = [](uint64_t x, uint64_t y) {
                    return x < y ? std::array<uint64_t, 2>{{x, y}}
                                 : std::array<uint64_t, 2>{{y, x}};
                };
                if (curve_degree[b] == 2 && mesh_features.count(key(a, b)) &&
                    mesh_features.count(key(b, c)) && !mesh_features.count(key(a, c))) {
                    ++ears;
                }
            }
        }
        CHECK(ears == 0);
    }
}

TEST_CASE("surface-map-small-regions-survive", "[surface_map]")
{
    // Single-triangle label regions -- curves all round, or curves and the open boundary --
    // through a coarsening with the boundary free: the remeshing may not collapse one away (that
    // would merge its curves), so every label is still there, and the map, with boundary
    // vertices collapsing inwards among its edits, has no warning and round-trips.
    MatrixXd V0;
    MatrixXi F0;
    wmtk::io::read_triangle_mesh(std::string(WMTK_DATA_DIR) + "/models/sphere.obj", V0, F0, 0);
    // The sphere less its top cap: an open boundary.
    surface_map::SurfaceData in;
    std::map<int, uint64_t> vid;
    for (int f = 0; f < F0.rows(); ++f) {
        double y = 0;
        for (int k = 0; k < 3; ++k) y += V0(F0(f, k), 1) / 3;
        if (y > 0.5) continue;
        std::array<uint64_t, 3> t;
        for (int k = 0; k < 3; ++k) {
            const auto [it, inserted] = vid.emplace(F0(f, k), in.vertices.size());
            if (inserted) in.vertices.push_back(V0.row(F0(f, k)).transpose());
            t[k] = it->second;
        }
        in.faces.push_back(t);
    }
    std::map<std::array<uint64_t, 2>, int> edge_count;
    for (const auto& t : in.faces) {
        for (int k = 0; k < 3; ++k) {
            ++edge_count[{{std::min(t[k], t[(k + 1) % 3]), std::max(t[k], t[(k + 1) % 3])}}];
        }
    }
    // A label of its own for every seventh triangle and for every third one on the boundary.
    auto& label = in.face_fields["label"];
    size_t singles = 0, on_boundary = 0;
    for (size_t f = 0; f < in.faces.size(); ++f) {
        bool boundary = false;
        for (int k = 0; k < 3; ++k) {
            const auto& t = in.faces[f];
            boundary =
                boundary ||
                edge_count[{{std::min(t[k], t[(k + 1) % 3]), std::max(t[k], t[(k + 1) % 3])}}] == 1;
        }
        const bool single = f % 7 == 0 || (boundary && f % 3 == 0);
        label.push_back({single ? double(f + 1) : 0.});
        singles += single;
        on_boundary += single && boundary;
    }
    REQUIRE(on_boundary > 0);
    std::vector<Eigen::Vector3d> verts(in.vertices.begin(), in.vertices.end());
    std::vector<std::array<size_t, 3>> tris;
    for (const auto& f : in.faces) tris.push_back({{size_t(f[0]), size_t(f[1]), size_t(f[2])}});
    const auto features = label_boundaries(in);

    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        isotropic_remeshing::IsotropicRemeshing m(verts, threads);
        m.create_mesh(verts.size(), tris, {}, false, 0, 0);
        m.set_feature_edges(features);
        const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
        for (const auto& e : m.feature_edges())
            rec->history().feature_edges.push_back({{e[0], e[1]}});
        m.uniform_remeshing(1.5 * m.average_len_valen()[0], 3);
        m.consolidate_mesh();
        m.stop_history();

        const SurfaceMap map(rec->history());
        INFO(
            "build: not a disk " << map.build_warnings().not_a_disk << ", flat "
                                 << map.build_warnings().flat_triangle << ", inverted "
                                 << map.build_warnings().inverted);
        CHECK(map.build_warnings().total() == 0);
        size_t trip_warnings = 0;
        CHECK(
            round_trip(map, sample(map.input_face_alive(), 3, 7), true, trip_warnings) <
            kRoundTrip);
        CHECK(trip_warnings == 0);

        const surface_map::SurfaceData out = surface_map::transfer_fields(map, in);
        std::set<double> labels_in, labels_out;
        for (const auto& l : label) labels_in.insert(l[0]);
        for (const auto& l : out.face_fields.at("label")) labels_out.insert(l[0]);
        CHECK(labels_out == labels_in);
        CHECK(out.faces.size() < in.faces.size()); // it did coarsen
    }
}

TEST_CASE("surface-map-transfer-uv", "[surface_map]")
{
    // The sphere's own texture coordinates, seams kept as curves: carried through the remeshing,
    // they stay continuous away from the seams, no texture triangle turns over away from the
    // poles, and the texture area is what it was.
    Eigen::MatrixXd V, TC, N;
    Eigen::MatrixXi F, FTC, FN;
    REQUIRE(igl::readOBJ(std::string(WMTK_DATA_DIR) + "/models/sphere.obj", V, TC, N, F, FTC, FN));
    surface_map::SurfaceData in;
    for (int i = 0; i < V.rows(); ++i) in.vertices.push_back(V.row(i).transpose());
    auto& uv = in.corner_fields["uv"];
    for (int f = 0; f < F.rows(); ++f) {
        in.faces.push_back({{uint64_t(F(f, 0)), uint64_t(F(f, 1)), uint64_t(F(f, 2))}});
        std::vector<double> c;
        for (int k = 0; k < 3; ++k) c.insert(c.end(), {TC(FTC(f, k), 0), TC(FTC(f, k), 1), 0.});
        uv.push_back(c);
    }
    // Seams: edges whose two faces disagree on the texture coordinates at either end.
    const auto corner = [](const surface_map::SurfaceData& s,
                           const std::vector<std::vector<double>>& field,
                           size_t f,
                           uint64_t v) {
        for (int c = 0; c < 3; ++c) {
            if (s.faces[f][c] == v) return Eigen::Vector2d(field[f][3 * c], field[f][3 * c + 1]);
        }
        return Eigen::Vector2d(NAN, NAN);
    };
    const auto edge_faces = [](const surface_map::SurfaceData& s) {
        std::map<std::array<uint64_t, 2>, std::vector<size_t>> ef;
        for (size_t f = 0; f < s.faces.size(); ++f) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = s.faces[f][k], b = s.faces[f][(k + 1) % 3];
                if (a > b) std::swap(a, b);
                ef[{{a, b}}].push_back(f);
            }
        }
        return ef;
    };
    std::vector<std::array<size_t, 2>> seams;
    for (const auto& [e, fs] : edge_faces(in)) {
        if (fs.size() == 2 && (corner(in, uv, fs[0], e[0]) != corner(in, uv, fs[1], e[0]) ||
                               corner(in, uv, fs[0], e[1]) != corner(in, uv, fs[1], e[1]))) {
            seams.push_back({{size_t(e[0]), size_t(e[1])}});
        }
    }
    REQUIRE(!seams.empty());
    const auto signed_area = [](const std::vector<double>& c) {
        return 0.5 * ((c[3] - c[0]) * (c[7] - c[1]) - (c[4] - c[1]) * (c[6] - c[0]));
    };
    const auto orientation = [](const std::vector<double>& c) {
        return wmtk::utils::predicates::orient2d(
            Eigen::Vector2d(c[0], c[1]),
            Eigen::Vector2d(c[3], c[4]),
            Eigen::Vector2d(c[6], c[7]));
    };
    double in_area = 0;
    int in_sign = 0;
    for (const auto& c : uv) {
        in_area += std::abs(signed_area(c));
        in_sign += int(orientation(c));
    }
    // The orientation of the texture; the input has no triangle against it, or flat.
    const auto texture_orientation = in_sign > 0 ? wmtk::utils::predicates::Orientation::POSITIVE
                                                 : wmtk::utils::predicates::Orientation::NEGATIVE;
    for (const auto& c : uv) REQUIRE(orientation(c) == texture_orientation);
    // The texture is singular at the poles -- a pole has a coordinate per triangle, and its first
    // ring is a straight line -- so a triangle reaching that far may come out flat or turned over
    // however good the map is. The caps out to the first rings are left out.
    std::map<uint64_t, std::set<std::pair<double, double>>> coordinates;
    std::map<uint64_t, std::set<uint64_t>> neighbours;
    for (size_t f = 0; f < in.faces.size(); ++f) {
        for (int k = 0; k < 3; ++k) {
            coordinates[in.faces[f][k]].insert({uv[f][3 * k], uv[f][3 * k + 1]});
            neighbours[in.faces[f][k]].insert(in.faces[f].begin(), in.faces[f].end());
        }
    }
    std::vector<std::pair<Eigen::Vector3d, double>> caps;
    for (const auto& [v, cs] : coordinates) {
        if (cs.size() <= 2) continue; // a seam has two
        double radius = 0;
        for (const uint64_t u : neighbours[v]) {
            radius = std::max(radius, (in.vertices[u] - in.vertices[v]).norm());
        }
        caps.push_back({in.vertices[v], radius});
    }
    REQUIRE(caps.size() == 2);
    const auto in_caps = [&caps](const Eigen::Vector3d& p) {
        for (const auto& [centre, radius] : caps) {
            if ((p - centre).norm() <= radius) return true;
        }
        return false;
    };

    std::vector<Eigen::Vector3d> verts(in.vertices.begin(), in.vertices.end());
    std::vector<std::array<size_t, 3>> tris;
    for (const auto& f : in.faces) tris.push_back({{size_t(f[0]), size_t(f[1]), size_t(f[2])}});
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        isotropic_remeshing::IsotropicRemeshing m(verts, threads);
        m.create_mesh(verts.size(), tris, {}, true, 0, 0);
        m.set_feature_edges(seams);
        const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
        for (const auto& e : m.feature_edges())
            rec->history().feature_edges.push_back({{e[0], e[1]}});
        m.uniform_remeshing(0.6 * m.average_len_valen()[0], 2);
        m.consolidate_mesh();
        m.stop_history();
        const SurfaceMap map(rec->history());
        CHECK(map.build_warnings().total() == 0);

        const surface_map::SurfaceData out = surface_map::transfer_fields(map, in);
        CHECK(map.map_warnings().total() == 0);
        const auto& ouv = out.corner_fields.at("uv");
        REQUIRE(ouv.size() == out.faces.size());

        std::set<std::array<uint64_t, 2>> features;
        for (const auto& e : m.feature_edges()) features.insert({{e[0], e[1]}});
        double jump = 0; // largest disagreement across an edge that is not a seam
        for (const auto& [e, fs] : edge_faces(out)) {
            if (fs.size() != 2 || features.count(e)) continue;
            for (const uint64_t v : e) {
                jump = std::max(
                    jump,
                    (corner(out, ouv, fs[0], v) - corner(out, ouv, fs[1], v)).norm());
            }
        }
        size_t flipped = 0, flipped_in_caps = 0;
        double out_area = 0;
        for (size_t f = 0; f < ouv.size(); ++f) {
            out_area += std::abs(signed_area(ouv[f]));
            if (orientation(ouv[f]) == texture_orientation) continue;
            bool capped = false;
            for (const uint64_t v : out.faces[f]) capped = capped || in_caps(out.vertices[v]);
            ++(capped ? flipped_in_caps : flipped);
        }
        INFO(
            "uv jump " << jump << ", flipped " << flipped << " of " << ouv.size() << " ("
                       << flipped_in_caps << " more at the poles), area " << out_area << " vs "
                       << in_area);
        CHECK(jump < 1e-5);
        // The remesher folds nothing over (IsotropicRemeshing::set_check_inversions), so no local
        // map has to stretch across a fold, and no texture triangle turns over.
        CHECK(map.folded_cells() == 0);
        CHECK(flipped == 0);
        CHECK(std::abs(out_area - in_area) < 1e-2 * in_area);
    }
}

TEST_CASE("surface-map-collapse-removes-a-side", "[surface_map]")
{
    // A sheet A, B, E and a fin F on the edge (0, 1), which is non-manifold, so A+E, B and F
    // are flattened as three sides. Collapsing 1 into 0 removes B and F whole: their points have
    // no image, while A's land on E.
    history::SurfaceHistory h;
    h.vertices = {
        {0, 0, 0},
        {1, 0, 0},
        {0.5, 1, 0},
        {0.5, -1, 0},
        {0, 0, 0}, // unused
        {0.5, 0, 1},
        {1.5, 1, 0}};
    h.vertex_alive = {1, 1, 1, 1, 0, 1, 1};
    h.faces = {{{0, 1, 2}}, {{1, 0, 3}}, {{0, 1, 5}}, {{2, 1, 6}}}; // A, B, F, E
    h.face_alive = {1, 1, 1, 1};
    h.edits.push_back(
        {history::EditKind::collapse_edge, {1, 0}, {}, {{0, Eigen::Vector3d(0.5, 0, 0)}}});

    const SurfaceMap map(h);
    REQUIRE(map.output_face_alive() == std::vector<uint8_t>{0, 0, 0, 1});
    const auto pts = sample(map.input_face_alive(), 10, 3);
    const auto there = map.to_output(pts);
    size_t lost = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (pts[i].face == 1 || pts[i].face == 2) {
            CHECK(there[i].face >= map.output_faces().size());
            ++lost;
        } else {
            CHECK(there[i].face == 3);
        }
    }
    CHECK(map.map_warnings().unlocated == lost);

    // Back from the output, every point lands on the side that survived.
    for (const auto& p : map.to_input(sample(map.output_face_alive(), 10, 4))) {
        CHECK((p.face == 0 || p.face == 3));
    }
}
