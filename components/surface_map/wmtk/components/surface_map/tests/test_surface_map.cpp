#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/components/surface_map/SurfaceMap.hpp>
#include <wmtk/components/surface_map/surface_io.hpp>
#include <wmtk/components/surface_map/transfer.hpp>
#include <wmtk/history/Replay.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>

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
        CHECK(in_err < 1e-9);

        const double out_err =
            round_trip(map, sample(map.output_face_alive(), 3, 2), false, warnings);
        CHECK(warnings == 0);
        CHECK(out_err < 1e-9);
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
    CHECK(map.chord_collapses() > 0);
    CHECK(w.not_a_disk == 0);
    CHECK(w.merged_faces == 0);
    CHECK(w.inverted <= w.flat_triangle);
    size_t warnings = 0;
    const double err = round_trip(map, sample(map.output_face_alive(), 1, 3), false, warnings);
    CHECK(warnings == 0);
    CHECK(err < 1e-9);
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

    const surface_map::SurfaceData out = surface_map::transfer_face_fields(map, reread);
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

/// The sphere, labelled per face by the side of x = 0 and by octant of the centroid.
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
        s.face_fields["side"].push_back({c.x() > 0 ? 1. : 0.});
        s.face_fields["octant"].push_back(
            {double((c.x() > 0) + 2 * (c.y() > 0) + 4 * (c.z() > 0))});
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
        const surface_map::SurfaceData out = surface_map::transfer_face_fields(map, in);
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
    }
}
