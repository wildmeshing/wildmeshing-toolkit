// How to use an application's tracking file: map your own points, or attributes, between its
// input and its output. This one uses isotropic_remeshing, whose output is a surface: a point on
// it is an output triangle and barycentric coordinates. See the tetwild and simwild tests for
// tet outputs, where it is three vertices of the tet mesh instead.

#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>

namespace fs = std::filesystem;
using namespace wmtk::components;
using tracking::InputPoint;
using tracking::kNoId;
using tracking::OutputPoint;
using tracking::Tracking;

namespace {

Eigen::Vector3d
at(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const uint64_t f, const Eigen::Vector3d& b)
{
    return b[0] * V.row(F(f, 0)).transpose() + b[1] * V.row(F(f, 1)).transpose() +
           b[2] * V.row(F(f, 2)).transpose();
}

std::vector<InputPoint> random_points(const size_t n_faces, const size_t n, const unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint64_t> face(0, n_faces - 1);
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<InputPoint> pts;
    for (size_t i = 0; i < n; ++i) {
        double a = u(rng), b = u(rng);
        if (a + b > 1) {
            a = 1 - a;
            b = 1 - b;
        }
        pts.push_back({face(rng), Eigen::Vector3d(1 - a - b, a, b)});
    }
    return pts;
}

} // namespace

TEST_CASE("tracking-example-isotropic", "[tracking]")
{
    const fs::path dir = fs::current_path() / "tracking_example_isotropic";
    fs::create_directories(dir);
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    const fs::path output = dir / "sphere_remeshed.obj";
    const fs::path tracking_file = dir / "sphere.wmtkhist";

    // 1. Run the application with tracking on: history_output names the tracking file.
    isotropic_remeshing::isotropic_remeshing({
        {"application", "isotropic_remeshing"},
        {"input", input.string()},
        {"output", output.string()},
        {"history_output", tracking_file.string()},
        {"length_rel", 0.08},
        {"max_iterations", 3},
    });
    Eigen::MatrixXd V_in, V_out;
    Eigen::MatrixXi F_in, F_out;
    REQUIRE(igl::readOBJ(input.string(), V_in, F_in));
    REQUIRE(igl::readOBJ(output.string(), V_out, F_out));

    // 2. Load it, and map points on the input -- a triangle of the input file and barycentric
    // coordinates -- to the output, and back.
    const Tracking tracking = Tracking::load(tracking_file.string());
    REQUIRE(!tracking.output_by_vertices()); // a surface output names its own triangles
    REQUIRE(tracking.input_faces().size() == size_t(F_in.rows()));
    const std::vector<InputPoint> pts = random_points(F_in.rows(), 500, 7);
    tracking::MapWarnings to_output_warnings, to_input_warnings;
    const std::vector<OutputPoint> there = tracking.to_output(pts, &to_output_warnings);
    const std::vector<InputPoint> back = tracking.to_input(there, &to_input_warnings);

    double mean_edge = 0;
    for (Eigen::Index f = 0; f < F_in.rows(); ++f) {
        mean_edge += (V_in.row(F_in(f, 0)) - V_in.row(F_in(f, 1))).norm();
    }
    mean_edge /= F_in.rows();
    double round_trip = 0, moved = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].face != kNoId);
        REQUIRE(there[i].face < uint64_t(F_out.rows()));
        REQUIRE(back[i].face == pts[i].face);
        round_trip = std::max(round_trip, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        // The image is where the point went on the surface: near it, the remeshing having moved
        // the surface's vertices along it by about an edge at most.
        moved = std::max(
            moved,
            (at(V_out, F_out, there[i].face, there[i].bary) -
             at(V_in, F_in, pts[i].face, pts[i].bary))
                .norm());
    }
    CHECK(round_trip < 1e-8);
    CHECK(moved < 2 * mean_edge);
    CHECK(to_output_warnings.total() == 0);
    CHECK(to_input_warnings.total() == 0);

    // 3. Remap an attribute of your own: per vertex on the input, wanted per vertex on the
    // output. Each output vertex is a point of an output triangle at one corner; map it to the
    // input and interpolate there.
    const auto f = [](const Eigen::Vector3d& p) { return p.x() + 2 * p.y() + 3 * p.z(); };
    std::vector<double> attribute(V_in.rows());
    for (Eigen::Index v = 0; v < V_in.rows(); ++v) attribute[v] = f(V_in.row(v).transpose());
    std::vector<OutputPoint> corners(V_out.rows());
    for (Eigen::Index t = 0; t < F_out.rows(); ++t) {
        for (int k = 0; k < 3; ++k) {
            corners[F_out(t, k)].face = uint64_t(t);
            corners[F_out(t, k)].bary = Eigen::Vector3d::Unit(k);
        }
    }
    const std::vector<InputPoint> sources = tracking.to_input(corners);
    double worst = 0;
    for (Eigen::Index v = 0; v < V_out.rows(); ++v) {
        const InputPoint& s = sources[v];
        REQUIRE(s.face != kNoId);
        double value = 0;
        for (int k = 0; k < 3; ++k) value += s.bary[k] * attribute[F_in(s.face, k)];
        // A linear attribute comes back as itself, up to how far the vertex moved on the surface.
        worst = std::max(worst, std::abs(value - f(V_out.row(v).transpose())));
    }
    CHECK(worst < 4 * 2 * mean_edge); // |grad f| < 4

    // 4. The same from the command line, as the wmtk binary runs it: a file of points, one
    // `face b0 b1 b2` per line, mapped by the tracking application.
    const fs::path points = dir / "points.txt";
    {
        std::ofstream o(points);
        o.precision(17);
        o << "# input triangle, then barycentric coordinates\n";
        for (const InputPoint& p : pts) {
            o << p.face << " " << p.bary[0] << " " << p.bary[1] << " " << p.bary[2] << "\n";
        }
    }
    const fs::path mapped = dir / "points_on_output.txt";
    tracking::tracking({
        {"application", "tracking"},
        {"history", tracking_file.string()},
        {"operation", "map"},
        {"points", points.string()},
        {"output_points", mapped.string()},
        {"direction", "to_output"},
    });
    std::ifstream in(mapped);
    size_t i = 0, same = 0;
    for (std::string line; std::getline(in, line); ++i) {
        std::istringstream ss(line);
        int64_t face;
        Eigen::Vector3d b;
        ss >> face >> b[0] >> b[1] >> b[2];
        REQUIRE(i < there.size());
        same += uint64_t(face) == there[i].face && (b - there[i].bary).norm() < 1e-12;
    }
    CHECK(i == pts.size());
    CHECK(same == pts.size());
}

TEST_CASE("tracking-memory-route", "[tracking]")
{
    // The memory route: an application records its history from the mesh it is given and
    // returns it in its report, as the bytes of the file the file route would write, which it
    // does not write; tracking's info reports on it, the report returned rather than logged
    // only.
    const fs::path dir = fs::current_path() / "tracking_memory_route";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const fs::path tracking_file = dir / "sphere.wmtkhist";
    const wmtk::MeshMap meshes{
        {"sphere",
         wmtk::io::read_mesh((fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj").string())}};
    const wmtk::ComponentResult remeshed = isotropic_remeshing::isotropic_remeshing(
        {{"input", "sphere"},
         {"history_output", tracking_file.string()},
         {"length_rel", 0.08},
         {"max_iterations", 3}},
        meshes);
    CHECK(!fs::exists(tracking_file));
    REQUIRE(remeshed.second.at("history").is_binary());
    const std::vector<uint8_t>& bytes = remeshed.second.at("history").get_binary();
    wmtk::history::write_history(tracking_file, bytes);

    // The history starts at the mesh given, as the meshes hold it, and ends at the mesh returned.
    const Tracking tracking(wmtk::history::SurfaceHistory::from_bytes(bytes));
    CHECK(
        tracking.input_faces().size() ==
        size_t(meshes.at("sphere").cell(wmtk::CellType::Triangle).connectivity.rows()));
    const auto& F_out = remeshed.first.at("mesh").cell(wmtk::CellType::Triangle).connectivity;
    const auto triangles = tracking.output_triangles();
    REQUIRE(triangles.size() == size_t(F_out.rows()));
    for (Eigen::Index f = 0; f < F_out.rows(); ++f) {
        for (int k = 0; k < 3; ++k) CHECK(triangles[f][k] == uint64_t(F_out(f, k)));
    }

    // tracking on the memory route takes the bytes.
    const nlohmann::json history = nlohmann::json::binary(bytes);
    const auto [none, report] =
        tracking::tracking({{"history", history}, {"operation", "info"}}, {});
    CHECK(none.empty());
    CHECK(report.at("edits").get<size_t>() > 0);
    CHECK(report.at("build_warnings").at("total") == 0);
    CHECK(report.at("round_trip_lost") == 0);
    CHECK(report.at("round_trip_max_displacement").get<double>() < 1e-8);

    // transfer: the fields of the mesh given -- uv per corner, a label per face -- onto the
    // output, returned as a mesh; nothing written.
    wmtk::MeshMap labelled = meshes;
    auto& t = labelled.at("sphere").cells.at(wmtk::CellType::Triangle);
    REQUIRE(t.attributes.at("uv").cols() == 6);
    wmtk::DataMatrix<int64_t> label(t.connectivity.rows(), 1);
    for (Eigen::Index f = 0; f < label.rows(); ++f) label(f, 0) = f % 5;
    t.attributes["label"].values = label;
    const auto [carried, transfer_report] = tracking::tracking(
        {{"history", history},
         {"operation", "transfer"},
         {"input", "sphere"},
         {"output_obj", (dir / "must_not_exist.obj").string()}},
        labelled);
    CHECK(!fs::exists(dir / "must_not_exist.obj"));
    REQUIRE(carried.count("mesh"));
    const auto& out = carried.at("mesh").cell(wmtk::CellType::Triangle);
    CHECK(out.connectivity == F_out);
    CHECK(out.attributes.at("uv").cols() == 6);
    CHECK(out.attributes.at("label").values.index() == 1);
    const auto from = out.attributes.at("input_face").integers();
    const auto labels = out.attributes.at("label").integers();
    for (Eigen::Index f = 0; f < from.rows(); ++f) {
        REQUIRE(from(f, 0) >= 0);
        CHECK(labels(f, 0) == from(f, 0) % 5);
    }
    CHECK(transfer_report.at("to_input_warnings").at("total") == 0);

    // What the memory route refuses: a file name for the history, which it never resolves;
    // a transfer without the mesh it starts from; map, whose points are text files.
    CHECK_THROWS_WITH(
        tracking::tracking({{"history", tracking_file.string()}, {"operation", "info"}}, {}),
        Catch::Matchers::ContainsSubstring("not a file name"));
    CHECK_THROWS_WITH(
        tracking::tracking({{"history", history}, {"operation", "transfer"}}, labelled),
        Catch::Matchers::ContainsSubstring("needs input"));
    CHECK_THROWS_WITH(
        tracking::tracking({{"history", history}, {"operation", "map"}}, {}),
        Catch::Matchers::ContainsSubstring("only info and transfer"));
    // And the file route refuses bytes for a file.
    CHECK_THROWS_WITH(
        tracking::tracking({{"application", "tracking"}, {"history", history}}),
        Catch::Matchers::ContainsSubstring("history is a file on the file route"));

    // The history says it was recorded on the memory route, whose input was a mesh, named by its
    // key: a transfer from the file of the history, given no input, does not take a file of
    // that name here for it.
    CHECK(
        nlohmann::json::parse(wmtk::history::SurfaceHistory::from_bytes(bytes).metadata)
            .at("route") == "memory");
    {
        std::ofstream o(fs::current_path() / "sphere");
        o << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    }
    CHECK_THROWS_WITH(
        tracking::tracking(
            {{"application", "tracking"},
             {"history", tracking_file.string()},
             {"operation", "transfer"},
             {"output", (dir / "transferred.msh").string()}}),
        Catch::Matchers::ContainsSubstring("recorded on the memory route"));
    fs::remove(fs::current_path() / "sphere");
    CHECK(!fs::exists(dir / "transferred.msh"));

    // Points and the mapped points are both relative to input_dir.
    {
        std::ofstream o(dir / "points.txt");
        o << "0 0.2 0.3 0.5\n";
    }
    tracking::tracking(
        {{"application", "tracking"},
         {"input_dir", dir.string()},
         {"history", tracking_file.filename().string()},
         {"operation", "map"},
         {"points", "points.txt"},
         {"output_points", "mapped.txt"}});
    CHECK(fs::exists(dir / "mapped.txt"));
    CHECK(!fs::exists(fs::current_path() / "mapped.txt"));
}

TEST_CASE("tracking-info-counts-a-lost-point-once", "[tracking]")
{
    // info maps the output's face centroids (segment midpoints) to the input and back. One on a
    // face a replace_faces added has no preimage: it is lost, and counted once, on the way in --
    // not again on the way back, where it is no point at all.
    using wmtk::history::EditKind;
    const auto info = [](const std::vector<uint8_t>& bytes) {
        return tracking::tracking(
                   {{"history", nlohmann::json::binary(bytes)}, {"operation", "info"}},
                   {})
            .second;
    };
    const auto check = [](const nlohmann::json& report) {
        CHECK(report.at("round_trip_lost") == 1);
        CHECK(report.at("to_input_warnings").at("total") == 1);
        CHECK(report.at("to_output_warnings").at("total") == 0);
        CHECK(report.at("round_trip_max_displacement").get<double>() < 1e-12);
    };
    SECTION("a surface")
    {
        // A square of two triangles; the second is replaced by a face with its corners.
        wmtk::history::SurfaceHistory h;
        h.vertices = {
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(1, 0, 0),
            Eigen::Vector3d(1, 1, 0),
            Eigen::Vector3d(0, 1, 0)};
        h.vertex_alive.assign(4, 1);
        h.faces = {{{0, 1, 2}}, {{0, 2, 3}}};
        h.face_alive.assign(2, 1);
        h.edits.push_back({EditKind::replace_faces, {0, 2, 3}, {1, 2}, {}});
        const nlohmann::json report = info(h.to_bytes());
        CHECK(report.at("centroids") == 2);
        check(report);
    }
    SECTION("curves")
    {
        // A polyline 0-1-2; its second segment is replaced by one with its ends.
        wmtk::history::CurveHistory h;
        h.vertices = {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 0, 0)};
        h.vertex_alive.assign(3, 1);
        h.faces = {{{0, 1}}, {{1, 2}}};
        h.face_alive.assign(2, 1);
        h.features = {{{0}}, {{2}}};
        h.edits.push_back({EditKind::replace_faces, {1, 2}, {1, 2}, {}});
        const nlohmann::json report = info(h.to_bytes());
        CHECK(report.at("midpoints") == 2);
        check(report);
    }
}

TEST_CASE("tracking-info-drift-in-three-units", "[tracking]")
{
    // info says how far the map moves the output's midpoints from their preimages: in mean
    // output lengths, in the input's bounding-box diagonal, and in the envelope's eps when the
    // recording's parameters give it. A fold the operations cut away is squeezed onto what is
    // left of it, so a point there lies as far from its preimage as the fold was long: flagged.
    using wmtk::history::EditKind;
    // A curve from (0,0) out to (10,0) and back to (0,0.001), then on to (1,0.001).
    wmtk::history::CurveHistory h;
    h.vertices = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(10, 0, 0),
        Eigen::Vector3d(0, 0.001, 0),
        Eigen::Vector3d(1, 0.001, 0)};
    h.vertex_alive.assign(4, 1);
    h.faces = {{{0, 1}}, {{1, 2}}, {{2, 3}}};
    h.face_alive.assign(3, 1);
    h.features = {{{0}}, {{3}}};
    h.metadata = R"({"application": "triwild", "eps_rel": 0.001})";
    const auto drift = [](const wmtk::history::CurveHistory& c) {
        return tracking::tracking(
                   {{"history", nlohmann::json::binary(c.to_bytes())}, {"operation", "info"}},
                   {})
            .second.at("drift");
    };
    const double diagonal = std::sqrt(100 + 1e-6);
    {
        // Untouched, every point is its own preimage.
        const nlohmann::json d = drift(h);
        CHECK(d.at("lengths").at("max").get<double>() < 1e-12);
        CHECK(d.at("diagonal").get<double>() == diagonal);
        CHECK(std::abs(d.at("envelope_eps").get<double>() - 1e-3 * diagonal) < 1e-15);
        CHECK_FALSE(d.at("large").get<bool>());
    }
    // The tip collapses onto the end of the way back: the fold, 20 long, becomes 0.001.
    h.edits.push_back({EditKind::collapse_edge, {1, 2}, {}, {}});
    const nlohmann::json d = drift(h);
    // The midpoint of what is left, (0,0.0005), maps half way along the fold: to its tip.
    const double max = Eigen::Vector2d(10, 0.0005).norm(), length = (0.001 + 1) / 2;
    const auto near = [](const double x, const double y) { return std::abs(x / y - 1) < 1e-6; };
    CHECK(near(d.at("mean_length").get<double>(), length));
    CHECK(near(d.at("lengths").at("max").get<double>(), max / length));
    CHECK(near(d.at("diagonals").at("max").get<double>(), max / diagonal));
    CHECK(near(d.at("eps").at("max").get<double>(), max / (1e-3 * diagonal)));
    CHECK(d.at("large").get<bool>());
}
