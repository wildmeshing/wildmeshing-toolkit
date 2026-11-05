#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <wmtk/MeshData.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#ifdef WMTK_TEST_qslim
#include <wmtk/components/qslim/qslim.hpp>
#endif
#ifdef WMTK_TEST_shortest_edge_collapse
#include <wmtk/components/shortest_edge_collapse/shortest_edge_collapse.hpp>
#endif
#ifdef WMTK_TEST_isotropic_remeshing
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#endif
#ifdef WMTK_TEST_simwild
#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/components/simwild/simwild.hpp>
#endif
#ifdef WMTK_TEST_triwild
#include <wmtk/components/triwild/triwild.hpp>
#endif
#ifdef WMTK_TEST_tetwild
#include <wmtk/components/tetwild/tetwild.hpp>
#endif
#ifdef WMTK_TEST_topological_offset
#include <wmtk/components/topological_offset/topological_offset.hpp>
#endif
#ifdef WMTK_TEST_manifold_extraction
#include <wmtk/components/manifold_extraction/manifold_extraction.hpp>
#endif
#ifdef WMTK_TEST_prismatic_mesh
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>
#endif
#ifdef WMTK_TEST_c1_simplification
#include <wmtk/components/c1_simplification/c1_simplification.hpp>
#endif

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif
using namespace wmtk;
using namespace wmtk::components;
namespace {
struct WorkingDirectory
{
    std::filesystem::path old = std::filesystem::current_path();
    std::filesystem::path path =
        old / ("wmtk_memory_io_" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    WorkingDirectory()
    {
        std::filesystem::create_directory(path);
        std::filesystem::current_path(path);
    }
    ~WorkingDirectory()
    {
        std::filesystem::current_path(old);
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};
MeshData triangle()
{
    MatrixXd V(3, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    MatrixXi F(1, 3);
    F << 0, 1, 2;
    return io::geometry_data(V, F, CellType::Triangle);
}
MeshData tet()
{
    MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    return io::geometry_data(V, T, CellType::Tetrahedron);
}
// The unit cube as its six tets around the main diagonal: a tet mesh that fills its box.
[[maybe_unused]] MeshData cube_tets()
{
    MatrixXd V(8, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1;
    MatrixXi T(6, 4);
    T << 0, 1, 3, 7, 0, 1, 7, 5, 0, 2, 7, 3, 0, 2, 6, 7, 0, 4, 5, 7, 0, 4, 7, 6;
    return io::geometry_data(V, T, CellType::Tetrahedron);
}
// The unit square as two triangles: a triangle mesh that fills its box.
[[maybe_unused]] MeshData square_triangles()
{
    MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0;
    MatrixXi F(2, 3);
    F << 0, 1, 3, 0, 3, 2;
    return io::geometry_data(V, F, CellType::Triangle);
}
void add_tags(MeshData& data, CellType type)
{
    auto& c = data.cells.at(type);
    c.attributes["tags"] = {DataMatrix<uint8_t>::Ones(c.connectivity.rows(), 1).eval(), {"tag_0"}};
}
void check(const ComponentResult& result)
{
    REQUIRE(result.first.count("mesh"));
    REQUIRE(result.second.is_object());
    for (const auto& [name, data] : result.first) {
        INFO(name);
        REQUIRE_NOTHROW(data.validate());
    }
    REQUIRE(std::filesystem::is_empty(std::filesystem::current_path()));
}
// The rows of `type` in `mesh` carrying the label `name`, as their corners' positions in row
// order.
[[maybe_unused]] std::vector<std::vector<Vector3d>>
labelled_rows(const MeshData& mesh, CellType type, const std::string& name)
{
    std::vector<std::vector<Vector3d>> rows;
    const auto it = mesh.cells.find(type);
    if (it == mesh.cells.end() || !it->second.attributes.count("tags")) return rows;
    const auto& tags = it->second.attributes.at("tags");
    const auto column = std::find(tags.labels.begin(), tags.labels.end(), name);
    if (column == tags.labels.end()) return rows;
    const auto member = tags.integers();
    const auto& c = it->second.connectivity;
    for (Eigen::Index r = 0; r < c.rows(); ++r) {
        if (member(r, column - tags.labels.begin()) == 0) continue;
        rows.emplace_back();
        for (Eigen::Index k = 0; k < c.cols(); ++k)
            rows.back().emplace_back(mesh.V(c(r, k), 0), mesh.V(c(r, k), 1), mesh.V(c(r, k), 2));
    }
    return rows;
}
// `mesh` with its tags only: the fields the .msh route drops (read_image_msh) dropped.
[[maybe_unused]] MeshData tags_only(MeshData mesh)
{
    mesh.vertex_attributes.clear();
    for (auto& entry : mesh.cells) {
        auto& attributes = entry.second.attributes;
        for (auto it = attributes.begin(); it != attributes.end();)
            it = it->first == "tags" ? std::next(it) : attributes.erase(it);
    }
    return mesh;
}
// Rows as labelled_rows() gives them, a triangle's corners turned to start at the smallest (its
// orientation kept), sorted: equal for the same oriented cells in any order.
[[maybe_unused]] std::vector<std::vector<std::array<double, 3>>> canonical(
    const std::vector<std::vector<Vector3d>>& rows)
{
    std::vector<std::vector<std::array<double, 3>>> out;
    for (const auto& row : rows) {
        std::vector<std::array<double, 3>> r;
        for (const Vector3d& p : row) r.push_back({{p[0], p[1], p[2]}});
        if (r.size() == 3) std::rotate(r.begin(), std::min_element(r.begin(), r.end()), r.end());
        out.push_back(r);
    }
    std::sort(out.begin(), out.end());
    return out;
}
// A unit tet's boundary, `shift` along x, its triangles facing out.
[[maybe_unused]] MeshData tet_surface(double shift)
{
    auto data = tet();
    data.V.col(0).array() += shift;
    data.cells.clear();
    data.cells[CellType::Triangle].connectivity.resize(4, 3);
    data.cells[CellType::Triangle].connectivity << 1, 2, 3, 0, 3, 2, 0, 1, 3, 0, 2, 1;
    return data;
}
// `mesh`'s triangles (f) or edges (l) as an OBJ file.
[[maybe_unused]] void write_obj(const std::string& path, const MeshData& mesh)
{
    std::ofstream f(path);
    f.precision(17);
    for (Eigen::Index v = 0; v < mesh.V.rows(); ++v)
        f << "v " << mesh.V(v, 0) << " " << mesh.V(v, 1) << " " << mesh.V(v, 2) << "\n";
    for (const auto& entry : mesh.cells) {
        const char* kind = entry.first == CellType::Triangle ? "f" : "l";
        const auto& c = entry.second.connectivity;
        for (Eigen::Index r = 0; r < c.rows(); ++r) {
            f << kind;
            for (Eigen::Index k = 0; k < c.cols(); ++k) f << " " << c(r, k) + 1;
            f << "\n";
        }
    }
}
// `name` in the working directory, as an absolute path: the file route reads inputs from there.
[[maybe_unused]] std::string here(const std::string& name)
{
    return (std::filesystem::current_path() / name).string();
}
} // namespace
TEST_CASE("Tag labels are unique and empty results are tagged", "[component_io]")
{
    struct Tagged
    {
        size_t m_tags_count = 3;
        std::map<int64_t, std::string> m_tag_id_to_name{{0, "tag_1"}};
    } mesh;
    MeshMap outputs{{"mesh", tet()}};
    const std::vector<int> cells = {0};
    const auto tags = [](int) { return std::set<int64_t>{0, 1}; };
    io::collect_tags(mesh, outputs, CellType::Tetrahedron, cells, tags);
    const auto& field = outputs.at("mesh").cell(CellType::Tetrahedron).attributes.at("tags");
    REQUIRE(field.labels == std::vector<std::string>{"tag_1", "tag_1_", "tag_2"});
    REQUIRE(field.integers() == (DataMatrix<int64_t>(1, 3) << 1, 1, 0).finished());
    MeshMap empty{{"mesh", MeshData()}};
    io::collect_tags(mesh, empty, CellType::Tetrahedron, std::vector<int>{}, tags);
    REQUIRE_NOTHROW(empty.at("mesh").validate());
}
TEST_CASE("Surface components operate on named meshes without files", "[component_io]")
{
    WorkingDirectory cwd;
    const auto data = triangle();
    MeshMap inputs{{"surface", data}};
    nlohmann::json options = {
        {"input", "surface"},
        {"num_threads", 1},
        {"target_rel", 1.0},
        {"eps_rel", 0.0},
        {"DEBUG_output", true},
        {"output", "must_not_exist.obj"},
        {"log_file", "must_not_exist.log"}};
#ifdef WMTK_TEST_qslim
    SECTION("qslim")
    {
        check(qslim::qslim(options, inputs));
    }
#endif
#ifdef WMTK_TEST_shortest_edge_collapse
    SECTION("shortest edge collapse")
    {
        check(shortest_edge_collapse::shortest_edge_collapse(options, inputs));
    }
#endif
#ifdef WMTK_TEST_isotropic_remeshing
    SECTION("isotropic remeshing")
    {
        options["max_iterations"] = 0;
        check(isotropic_remeshing::isotropic_remeshing(options, inputs));
    }
#endif
    REQUIRE(inputs.at("surface").V == data.V);
    REQUIRE(
        inputs.at("surface").cell(CellType::Triangle).connectivity ==
        data.cell(CellType::Triangle).connectivity);
}
#ifdef WMTK_TEST_simwild
TEST_CASE("SimWild accepts tagged arrays in both dimensions", "[component_io]")
{
    // A mesh that fills its box, as simwild takes a tagged mesh (require_box_boundary()), every
    // cell tagged, and its boundary as the envelope, oriented out of it.
    WorkingDirectory cwd;
    MeshData data;
    SECTION("2D")
    {
        data = square_triangles();
        add_tags(data, CellType::Triangle);
    }
    SECTION("3D")
    {
        data = cube_tets();
        add_tags(data, CellType::Tetrahedron);
    }
    MeshData envelope;
    envelope.V = data.V;
    if (data.cells.count(CellType::Tetrahedron)) {
        envelope.cells[CellType::Triangle].connectivity.resize(12, 3);
        envelope.cells[CellType::Triangle].connectivity << 0, 3, 1, 0, 2, 3, 1, 3, 7, 1, 7, 5, 0, 1,
            5, 0, 5, 4, 2, 7, 3, 2, 6, 7, 0, 6, 2, 0, 4, 6, 4, 5, 7, 4, 7, 6;
    } else {
        envelope.cells[CellType::Edge].connectivity.resize(4, 2);
        envelope.cells[CellType::Edge].connectivity << 0, 1, 1, 3, 3, 2, 2, 0;
    }
    nlohmann::json options = {
        {"input", "tagged"},
        {"envelope", "boundary"},
        {"max_iterations", 0},
        {"num_threads", 1},
        {"DEBUG_output", true},
        {"write_vtu", true}};
    auto result = simwild::simwild(options, {{"tagged", data}, {"boundary", envelope}});
    check(result);
    REQUIRE(result.first.at("mesh").cells.begin()->second.attributes.count("tags"));
}
TEST_CASE(
    "SimWild remeshes a tagged mesh with nothing tracked and no envelope",
    "[component_io][tagged]")
{
    // One region filling the box: no interface, so no tracked face (2D: edge), and no envelope
    // given, so the one built from the tracked faces is empty. Until 2026-11-05 2D crashed
    // building it (Envelope::init on no edges).
    WorkingDirectory cwd;
    const bool is_3d = GENERATE(false, true);
    INFO((is_3d ? "3D" : "2D"));
    MeshData data = is_3d ? cube_tets() : square_triangles();
    add_tags(data, is_3d ? CellType::Tetrahedron : CellType::Triangle);
    check(
        simwild::simwild(
            {{"input", "tagged"}, {"max_iterations", 0}, {"num_threads", 1}},
            {{"tagged", data}}));
}
TEST_CASE("SimWild refuses a tagged mesh that does not fill its box", "[component_io][tagged]")
{
    // One tagged tet (2D: triangle) with its boundary as the envelope, as "SimWild accepts tagged
    // arrays in both dimensions" took it until 2026-11-05: its slanted face (edge) is a boundary
    // face in no plane of the box, which nothing would hold (require_box_boundary()).
    WorkingDirectory cwd;
    const bool is_3d = GENERATE(false, true);
    INFO((is_3d ? "3D" : "2D"));
    MeshData data = is_3d ? tet() : triangle();
    add_tags(data, is_3d ? CellType::Tetrahedron : CellType::Triangle);
    MeshData envelope;
    envelope.V = data.V;
    if (is_3d) {
        envelope.cells[CellType::Triangle].connectivity.resize(4, 3);
        envelope.cells[CellType::Triangle].connectivity << 1, 2, 3, 0, 3, 2, 0, 1, 3, 0, 2, 1;
    } else {
        envelope.cells[CellType::Edge].connectivity.resize(3, 2);
        envelope.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 0;
    }
    REQUIRE_THROWS_WITH(
        simwild::simwild(
            {{"input", "tagged"},
             {"envelope", "boundary"},
             {"max_iterations", 0},
             {"num_threads", 1}},
            {{"tagged", data}, {"boundary", envelope}}),
        Catch::Matchers::ContainsSubstring(
            is_3d ? "1 untracked boundary faces in no plane of its bounding box"
                  : "1 untracked boundary edges on no line of its bounding box"));
}
TEST_CASE("Tagged meshes read a triangle block as tagged faces", "[component_io]")
{
    // A tagged tet plus one of its faces tagged `sheet`. Only rows labelled EnvelopeSurface,
    // simwild's envelope group, are an envelope.
    auto data = tet();
    add_tags(data, CellType::Tetrahedron);
    auto& faces = data.cells[CellType::Triangle];
    faces.connectivity.resize(1, 3);
    faces.connectivity << 1, 2, 3;
    faces.attributes["tags"] = {DataMatrix<uint8_t>::Ones(1, 1).eval(), {"sheet"}};
    const auto input = simwild::read_mesh_data(data);
    REQUIRE(input.V_input.rows() == 4);
    REQUIRE(input.F_tagged.rows() == 1);
    REQUIRE(input.face_tag_names == std::vector<std::string>{"sheet"});
    REQUIRE(input.V_envelope.rows() == 0);
    const auto separate = triangle();
    const auto both = simwild::read_mesh_data(data, &separate);
    REQUIRE(both.F_tagged.rows() == 1);
    REQUIRE(both.V_envelope.rows() == 3);
    SECTION("rows labelled EnvelopeSurface, on vertices of their own, are the envelope")
    {
        data.V.conservativeResize(6, 3);
        data.V.bottomRows(2) << 5, 5, 5, 6, 5, 5;
        faces.connectivity.conservativeResize(2, 3);
        faces.connectivity.row(1) << 4, 5, 0;
        DataMatrix<uint8_t> member(2, 2);
        member << 1, 0, 0, 1;
        faces.attributes["tags"] = {member, {"sheet", "EnvelopeSurface"}};
        const auto split = simwild::read_mesh_data(data);
        REQUIRE(split.V_input.rows() == 4);
        REQUIRE(split.F_tagged.rows() == 1);
        REQUIRE(split.face_tag_names == std::vector<std::string>{"sheet"});
        REQUIRE(split.V_envelope.rows() == 3);
        REQUIRE(split.F_envelope.rows() == 1);
        REQUIRE(split.envelope_name == "EnvelopeSurface");
        REQUIRE_THROWS(simwild::read_mesh_data(data, &separate));
    }
    SECTION("a face label that is also a tet label")
    {
        faces.attributes["tags"].labels = {"tag_0"};
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
    SECTION("a face on a vertex no tet has")
    {
        data.V.conservativeResize(5, 3);
        data.V.row(4) << 5, 5, 5;
        faces.connectivity << 1, 2, 4;
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
    SECTION("a triangle block without tags")
    {
        faces.attributes.clear();
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
    SECTION("an edge block in a tet mesh without tags")
    {
        data.cells[CellType::Edge].connectivity.resize(1, 2);
        data.cells[CellType::Edge].connectivity << 0, 1;
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
}
TEST_CASE("Tagged meshes read an edge block as tagged curves", "[component_io]")
{
    // A tagged triangle and two of its edges, tagged `curve`, the first also `tag_0`, the label of
    // the triangle: in 2D a curve and a region may share a name, as triwild's do.
    auto data = triangle();
    add_tags(data, CellType::Triangle);
    auto& edges = data.cells[CellType::Edge];
    edges.connectivity.resize(2, 2);
    edges.connectivity << 1, 0, 1, 2;
    DataMatrix<uint8_t> member(2, 2);
    member << 1, 1, 1, 0;
    edges.attributes["tags"] = {member, {"curve", "tag_0"}};
    const auto input = simwild::read_mesh_data(data);
    REQUIRE(input.F_tagged.rows() == 2);
    REQUIRE(input.F_tagged.cols() == 2);
    REQUIRE(input.face_tag_names == std::vector<std::string>{"curve", "tag_0"});
    REQUIRE(input.tag_names == std::vector<std::string>{"tag_0"});
    // Each row as given: its vertex order is its orientation.
    CHECK(input.F_tagged(0, 0) == 1);
    CHECK(input.F_tagged(0, 1) == 0);
    CHECK(input.F_tags.coeff(0, 0) == 1);
    CHECK(input.F_tags.coeff(0, 1) == 1);
    CHECK(input.F_tags.coeff(1, 0) == 1);
    CHECK(input.F_tags.coeff(1, 1) == 0);
    // Tagged curves are no envelope: it is built from the tags, or given.
    REQUIRE(input.V_envelope.rows() == 0);
    MeshData separate;
    separate.V = data.V;
    separate.cells[CellType::Edge].connectivity.resize(3, 2);
    separate.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 0;
    const auto both = simwild::read_mesh_data(data, &separate);
    REQUIRE(both.F_tagged.rows() == 2);
    REQUIRE(both.F_envelope.rows() == 3);
    SECTION("rows labelled EnvelopeSurface, on vertices of their own, are the envelope")
    {
        edges.connectivity.conservativeResize(3, 2);
        edges.connectivity.row(2) << 2, 0;
        DataMatrix<uint8_t> three(3, 3);
        three << 1, 1, 0, 1, 0, 0, 0, 0, 1;
        edges.attributes["tags"] = {three, {"curve", "tag_0", "EnvelopeSurface"}};
        const auto split = simwild::read_mesh_data(data);
        REQUIRE(split.F_tagged.rows() == 2);
        REQUIRE(split.face_tag_names == std::vector<std::string>{"curve", "tag_0"});
        REQUIRE(split.V_envelope.rows() == 2);
        REQUIRE(split.F_envelope.rows() == 1);
        REQUIRE(split.envelope_name == "EnvelopeSurface");
        REQUIRE_THROWS(simwild::read_mesh_data(data, &separate));
    }
    SECTION("an edge block without tags is the envelope, as a lower group always was")
    {
        edges.attributes.clear();
        const auto envelope = simwild::read_mesh_data(data);
        REQUIRE(envelope.F_tagged.rows() == 0);
        REQUIRE(envelope.V_envelope.rows() == 3);
        REQUIRE(envelope.F_envelope.rows() == 2);
        REQUIRE_THROWS(simwild::read_mesh_data(data, &separate));
    }
    SECTION("an edge without a tag")
    {
        member(1, 0) = 0;
        edges.attributes["tags"] = {member, {"curve", "tag_0"}};
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
    SECTION("an edge on a vertex no triangle has")
    {
        data.V.conservativeResize(4, 3);
        data.V.row(3) << 5, 5, 0;
        edges.connectivity.row(1) << 1, 3;
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
}
TEST_CASE("Tagged tet meshes read an edge block as feature curves", "[component_io]")
{
    // A tagged tet and two of its edges, tagged as two feature inputs: (0, 1) by both, (1, 3) by
    // the second. Each is an input edge per tag, placed on itself.
    auto data = tet();
    add_tags(data, CellType::Tetrahedron);
    auto& edges = data.cells[CellType::Edge];
    edges.connectivity.resize(2, 2);
    edges.connectivity << 0, 1, 1, 3;
    DataMatrix<uint8_t> member(2, 2);
    member << 1, 1, 0, 1;
    edges.attributes["tags"] = {member, {"feature_0", "feature_1"}};
    const auto input = simwild::read_mesh_data(data);
    REQUIRE(input.feature_edge_names == std::vector<std::string>{"feature_0", "feature_1"});
    REQUIRE(input.features.edges.size() == 3);
    REQUIRE(input.features.edge_input == std::vector<int>{0, 1, 1});
    REQUIRE(input.features.points.empty());
    REQUIRE(input.features_placed.edge_tiling.size() == 3);
    const std::vector<std::array<size_t, 2>> placed = {{{0, 1}}, {{0, 1}}, {{1, 3}}};
    for (size_t e = 0; e < 3; ++e) {
        REQUIRE(input.features_placed.edge_tiling[e].size() == 1);
        CHECK(input.features_placed.edge_tiling[e][0] == placed[e]);
        for (int k = 0; k < 2; ++k) {
            const Vector3d read = input.features.edge_vertices[input.features.edges[e][k]];
            const Vector3d at = input.V_input.row(Eigen::Index(placed[e][k])).transpose();
            CHECK((read - at).squaredNorm() == 0);
        }
    }
    // The faces and the envelope are untouched.
    REQUIRE(input.F_tagged.rows() == 0);
    REQUIRE(input.V_envelope.rows() == 0);
    SECTION("EnvelopeSurface labels triangles in a tet mesh")
    {
        edges.attributes["tags"].labels = {"feature_0", "EnvelopeSurface"};
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
    SECTION("an edge on a vertex no tet has")
    {
        data.V.conservativeResize(5, 3);
        data.V.row(4) << 5, 5, 5;
        edges.connectivity.row(1) << 1, 4;
        REQUIRE_THROWS(simwild::read_mesh_data(data));
    }
}
TEST_CASE("Tagged mesh files carry an earlier offset's correspondence", "[component_io][io]")
{
    // corr_input_vid names the vid of each offset vertex's input vertex: on both routes the same
    // V_input rows. The .msh route used to drop it.
    WorkingDirectory cwd;
    auto data = tet();
    add_tags(data, CellType::Tetrahedron);
    data.vertex_attributes["vid"].values = (DataMatrix<int64_t>(4, 1) << 10, 20, 30, 40).finished();
    data.vertex_attributes["corr_input_vid"].values =
        (DataMatrix<int64_t>(4, 1) << -1, -1, -1, 10).finished();
    io::write_mesh("offset.msh", data);
    const auto memory = simwild::read_mesh_data(data);
    const auto file = simwild::read_image_msh("offset.msh");
    REQUIRE(memory.corr_input_vid == std::vector<int64_t>{-1, -1, -1, 0});
    REQUIRE(file.corr_input_vid == memory.corr_input_vid);
}
TEST_CASE("SimWild keeps a tagged mesh's curves on both routes", "[component_io][io]")
{
    // The first cell tagged, so that the face (2D: edge) between it and the next is an interface,
    // in a mesh that fills its box, as simwild takes a tagged mesh: in 2D the unit square of two
    // triangles and the two below it, the edge (1, 0) of the first tagged `curve` -- inside the
    // mesh, as simwild takes a tracked edge; in 3D the unit cube's six tets (cube_tets()), the
    // main diagonal (7, 0), an edge of the first and inside the mesh, tagged `feature_0`. Until
    // 2026-11-05 the 3D mesh was two tets, which do not fill their box. Through simwild, the curve
    // comes back on its edge block, under its label, held on its segment, the 2D curve in its
    // direction.
    WorkingDirectory cwd;
    const bool is_3d = GENERATE(false, true);
    INFO((is_3d ? "3D" : "2D"));
    MeshData data;
    if (is_3d) {
        data = cube_tets();
    } else {
        MatrixXd V(6, 3);
        V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, -1, 0, 1, -1, 0;
        MatrixXi F(4, 3);
        F << 0, 1, 2, 1, 3, 2, 0, 4, 1, 1, 4, 5;
        data = io::geometry_data(V, F, CellType::Triangle);
    }
    auto& cells = data.cells.at(is_3d ? CellType::Tetrahedron : CellType::Triangle);
    const Eigen::Index n_cells = cells.connectivity.rows();
    DataMatrix<uint8_t> first = DataMatrix<uint8_t>::Zero(n_cells, 1);
    first(0, 0) = 1;
    cells.attributes["tags"] = {first, {"tag_0"}};
    const std::string label = is_3d ? "feature_0" : "curve";
    auto& edges = data.cells[CellType::Edge];
    edges.connectivity.resize(1, 2);
    if (is_3d) {
        edges.connectivity << 7, 0;
    } else {
        edges.connectivity << 1, 0;
    }
    edges.attributes["tags"] = {DataMatrix<uint8_t>::Ones(1, 1).eval(), {label}};
    const nlohmann::json options = {{"max_iterations", 0}, {"simplify", false}, {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "tagged";
    const auto result = simwild::simwild(memory_options, {{"tagged", data}});
    check(result);
    io::write_mesh("tagged.msh", data);
    auto file_options = options;
    file_options["application"] = "simwild";
    file_options["input"] = {here("tagged.msh")};
    file_options["output"] = "out";
    file_options["write_vtu"] = false;
    simwild::simwild(file_options);
    const MeshData file = io::read_mesh("out.msh");
    for (const MeshData* mesh : {&result.first.at("mesh"), &file}) {
        const auto rows = labelled_rows(*mesh, CellType::Edge, label);
        REQUIRE(!rows.empty());
        for (const auto& row : rows) {
            // On its segment: the x axis in 2D, the main diagonal in 3D.
            for (const Vector3d& p : row) {
                CHECK(
                    (is_3d ? std::abs(p.x() - p.y()) + std::abs(p.y() - p.z())
                           : std::abs(p.y()) + std::abs(p.z())) < 1e-12);
            }
            // In 3D a curve has no direction, and the row is in ascending vertex order.
            if (!is_3d) CHECK(row[1].x() < row[0].x());
        }
    }
}
TEST_CASE("SimWild checks a forced dimension against memory inputs", "[component_io]")
{
    WorkingDirectory cwd;
    auto surface = tet();
    surface.cells.clear();
    surface.cells[CellType::Triangle].connectivity.resize(4, 3);
    surface.cells[CellType::Triangle].connectivity << 1, 2, 3, 0, 3, 2, 0, 1, 3, 0, 2, 1;
    MeshData curves = triangle();
    curves.cells.clear();
    curves.cells[CellType::Edge].connectivity.resize(3, 2);
    curves.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 0;
    auto tagged = tet();
    add_tags(tagged, CellType::Tetrahedron);
    const MeshMap inputs{{"surface", surface}, {"curves", curves}, {"tagged", tagged}};
    auto run = [&](const std::string& input, const std::string& dimension) {
        simwild::simwild(
            {{"input", input}, {"dimension", dimension}, {"max_iterations", 0}, {"num_threads", 1}},
            inputs);
    };
    // A tagged mesh with another input: refused, rather than the other dropped.
    REQUIRE_THROWS_WITH(
        simwild::simwild(
            {{"input", {"tagged", "surface"}}, {"max_iterations", 0}, {"num_threads", 1}},
            inputs),
        Catch::Matchers::ContainsSubstring("only input"));
    REQUIRE_THROWS_WITH(run("surface", "2"), Catch::Matchers::ContainsSubstring("dimension"));
    REQUIRE_THROWS_WITH(run("curves", "3"), Catch::Matchers::ContainsSubstring("dimension"));
    REQUIRE_THROWS_WITH(run("tagged", "2"), Catch::Matchers::ContainsSubstring("dimension"));
    REQUIRE_THROWS_WITH(
        simwild::simwild(
            {{"input", "surface"}, {"envelope", "surface"}, {"max_iterations", 0}},
            inputs),
        Catch::Matchers::ContainsSubstring("envelope"));
}
#endif
#ifdef WMTK_TEST_triwild
TEST_CASE("TriWild embeds named curves in memory", "[component_io]")
{
    WorkingDirectory cwd;
    MeshData data;
    data.V.resize(4, 3);
    data.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0;
    data.cells[CellType::Edge].connectivity.resize(4, 2);
    data.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 3, 3, 0;
    check(
        triwild::triwild(
            {{"input", {"curves"}},
             {"simplify", false},
             {"max_iterations", 0},
             {"num_threads", 1},
             {"DEBUG_output", true},
             {"write_vtu", true}},
            {{"curves", data}}));
}
TEST_CASE("TriWild writes each input's curve tags on both routes", "[component_io][io]")
{
    // Two unit squares 3 apart, counter-clockwise, named `a` and `b`. The output's edge block
    // holds each tracked edge under the inputs it lies on, oriented as its input, and under the
    // name its region's triangles carry. Until 2026-11 triwild wrote no edge tags.
    WorkingDirectory cwd;
    const auto square = [](double shift) {
        MeshData data;
        data.V.resize(4, 3);
        data.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0;
        data.V.col(0).array() += shift;
        data.cells[CellType::Edge].connectivity.resize(4, 2);
        data.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 3, 3, 0;
        return data;
    };
    const nlohmann::json options = {
        {"input_names", {"a", "b"}},
        {"simplify", false},
        {"max_iterations", 0},
        {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = {"a", "b"};
    const auto result = triwild::triwild(memory_options, {{"a", square(0)}, {"b", square(3)}});
    check(result);
    write_obj("a.obj", square(0));
    write_obj("b.obj", square(3));
    auto file_options = options;
    file_options["application"] = "triwild";
    file_options["input"] = {here("a.obj"), here("b.obj")};
    file_options["output"] = "out";
    file_options["write_vtu"] = false;
    triwild::triwild(file_options);
    const MeshData file = io::read_mesh("out.msh");
    for (const MeshData* mesh : {&result.first.at("mesh"), &file}) {
        const auto& block = mesh->cell(CellType::Edge);
        REQUIRE(block.attributes.at("tags").labels == std::vector<std::string>{"a", "b"});
        size_t n = 0;
        for (const std::string name : {"a", "b"}) {
            INFO(name);
            // No region tags: regions are simwild's (until 2026-11 each input's was one).
            CHECK(labelled_rows(*mesh, CellType::Triangle, name).empty());
            const Vector3d centre(name == "a" ? 0.5 : 3.5, 0.5, 0);
            const auto rows = labelled_rows(*mesh, CellType::Edge, name);
            CHECK(rows.size() >= 4);
            n += rows.size();
            for (const auto& e : rows) {
                // On its own input, and turning counter-clockwise around it.
                const Vector3d d = e[1] - e[0], r = centre - e[0];
                CHECK(std::abs((e[0] + e[1]).x() / 2 - centre.x()) < 1);
                CHECK(d.x() * r.y() - d.y() * r.x() > 0);
            }
        }
        // Each edge on one input: the two are 3 apart.
        CHECK(n == size_t(block.connectivity.rows()));
    }
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(result.first.at("mesh"), CellType::Edge, name)) ==
            canonical(labelled_rows(file, CellType::Edge, name)));
    }
#ifdef WMTK_TEST_simwild
    // Read back as simwild reads a tagged mesh: tracked edges with their tags, not the envelope.
    // On the memory route once triwild's fields, which it does not take, are dropped.
    const auto from_memory = simwild::read_mesh_data(tags_only(result.first.at("mesh")));
    const auto from_file = simwild::read_image_msh("out.msh");
    for (const auto* input : {&from_memory, &from_file}) {
        CHECK(input->face_tag_names == std::vector<std::string>{"a", "b"});
        CHECK(
            input->F_tagged.rows() ==
            result.first.at("mesh").cell(CellType::Edge).connectivity.rows());
        CHECK(input->V_envelope.rows() == 0);
        CHECK(input->tag_names.empty());
    }
#endif
}
#endif
#ifdef WMTK_TEST_tetwild
TEST_CASE("TetWild embeds a named surface in memory", "[component_io]" + release_only)
{
    WorkingDirectory cwd;
    auto data = tet();
    data.cells.clear();
    data.cells[CellType::Triangle].connectivity.resize(4, 3);
    data.cells[CellType::Triangle].connectivity << 1, 2, 3, 0, 3, 2, 0, 1, 3, 0, 2, 1;
    check(
        tetwild::tetwild(
            {{"input", {"surface"}},
             {"simplify", false},
             {"max_iterations", 0},
             {"num_threads", 1},
             {"DEBUG_output", true},
             {"write_vtu", true}},
            {{"surface", data}}));
}
TEST_CASE(
    "TetWild writes each input's face tags and its feature curves on both routes",
    "[component_io][io]" + release_only)
{
    // Two unit tets 3 apart, named `a` and `b`, and a segment inside `a` as a feature file. The
    // output's triangle block holds each tracked face under the inputs it lies on, facing as its
    // input (out), and its edge block the tet edges tiling the segment, as feature_0. With and
    // without per-input envelopes: the inputs are the tags either way. Until 2026-11 tetwild wrote
    // no tags, and without per-input envelopes every tracked face was input 0's.
    WorkingDirectory cwd;
    const bool per_input_envelopes = GENERATE(true, false);
    INFO("per_input_envelopes " << per_input_envelopes);
    MeshData curve;
    curve.V.resize(2, 3);
    curve.V << 0.1, 0.1, 0.1, 0.3, 0.2, 0.1;
    curve.cells[CellType::Edge].connectivity.resize(1, 2);
    curve.cells[CellType::Edge].connectivity << 0, 1;
    const nlohmann::json options = {
        {"input_names", {"a", "b"}},
        {"simplify", false},
        {"max_iterations", 0},
        {"num_threads", 1},
        {"per_input_envelopes", per_input_envelopes}};
    auto memory_options = options;
    memory_options["input"] = {"a", "b"};
    memory_options["input_edges"] = {"curve"};
    const auto result = tetwild::tetwild(
        memory_options,
        {{"a", tet_surface(0)}, {"b", tet_surface(3)}, {"curve", curve}});
    check(result);
    write_obj("a.obj", tet_surface(0));
    write_obj("b.obj", tet_surface(3));
    write_obj("curve.obj", curve);
    auto file_options = options;
    file_options["application"] = "tetwild";
    file_options["input"] = {here("a.obj"), here("b.obj")};
    file_options["input_edges"] = {here("curve.obj")};
    file_options["output"] = "out";
    file_options["write_vtu"] = false;
    tetwild::tetwild(file_options);
    const MeshData file = io::read_mesh("out_final.msh");
    const double eps = result.second.at("eps");
    const Vector3d p = curve.V.row(0).transpose(), q = curve.V.row(1).transpose();
    const auto off_segment = [&](const Vector3d& x) {
        const Vector3d d = q - p;
        const double t = std::clamp((x - p).dot(d) / d.squaredNorm(), 0.0, 1.0);
        return (p + t * d - x).norm();
    };
    for (const MeshData* mesh : {&result.first.at("mesh"), &file}) {
        const auto& faces = mesh->cell(CellType::Triangle);
        REQUIRE(faces.attributes.at("tags").labels == std::vector<std::string>{"a", "b"});
        size_t n = 0;
        for (const std::string name : {"a", "b"}) {
            INFO(name);
            const Vector3d centre(name == "a" ? 0.25 : 3.25, 0.25, 0.25);
            const auto rows = labelled_rows(*mesh, CellType::Triangle, name);
            CHECK(rows.size() >= 4);
            n += rows.size();
            for (const auto& f : rows) {
                // On its own input, and facing out of it.
                const Vector3d c = (f[0] + f[1] + f[2]) / 3;
                CHECK(std::abs(c.x() - centre.x()) < 1);
                CHECK((f[1] - f[0]).cross(f[2] - f[0]).dot(c - centre) > 0);
            }
        }
        // Each face on one input: the two are 3 apart.
        CHECK(n == size_t(faces.connectivity.rows()));
        // The segment's tet edges: in its tube, its ends among their vertices, within the
        // anchors' radius.
        REQUIRE(
            mesh->cell(CellType::Edge).attributes.at("tags").labels ==
            std::vector<std::string>{"feature_0"});
        const auto edges = labelled_rows(*mesh, CellType::Edge, "feature_0");
        REQUIRE(!edges.empty());
        for (const Vector3d& end : {p, q}) {
            double nearest = std::numeric_limits<double>::infinity();
            for (const auto& e : edges) {
                for (const Vector3d& x : e) {
                    CHECK(off_segment(x) <= eps);
                    nearest = std::min(nearest, (x - end).norm());
                }
            }
            CHECK(nearest <= eps);
        }
    }
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(result.first.at("mesh"), CellType::Triangle, name)) ==
            canonical(labelled_rows(file, CellType::Triangle, name)));
    }
    CHECK(
        canonical(labelled_rows(result.first.at("mesh"), CellType::Edge, "feature_0")) ==
        canonical(labelled_rows(file, CellType::Edge, "feature_0")));
#ifdef WMTK_TEST_simwild
    // Read back as simwild and the offset read a tagged mesh: the face tags and the feature
    // curves. On the memory route once tetwild's fields, which it does not take, are dropped.
    const auto from_memory = simwild::read_mesh_data(tags_only(result.first.at("mesh")));
    const auto from_file = simwild::read_image_msh("out_final.msh");
    const auto& mesh = result.first.at("mesh");
    for (const auto* input : {&from_memory, &from_file}) {
        CHECK(input->face_tag_names == std::vector<std::string>{"a", "b"});
        CHECK(input->F_tagged.rows() == mesh.cell(CellType::Triangle).connectivity.rows());
        CHECK(input->feature_edge_names == std::vector<std::string>{"feature_0"});
        CHECK(
            input->features.edges.size() == size_t(mesh.cell(CellType::Edge).connectivity.rows()));
        CHECK(input->tag_names.empty());
    }
#endif
}
#endif
#ifdef WMTK_TEST_topological_offset
TEST_CASE("Offset accepts named tagged meshes", "[component_io]")
{
    // A mesh that fills its box: the offset refuses one that does not (require_box_boundary()),
    // as a single tet does, its fourth face in no plane of the box.
    WorkingDirectory cwd;
    auto data = cube_tets();
    add_tags(data, CellType::Tetrahedron);
    check(
        topological_offset::topological_offset(
            {{"input", "tagged"},
             {"offset_selection", "tag_0 & !tag_0"},
             {"optimize_offset", false},
             {"DEBUG_output", true}},
            {{"tagged", data}}));
}
#endif
#ifdef WMTK_TEST_manifold_extraction
TEST_CASE("Manifold extraction accepts named tagged meshes", "[component_io]")
{
    WorkingDirectory cwd;
    auto data = triangle();
    add_tags(data, CellType::Triangle);
    MeshData envelope = triangle();
    envelope.cells.clear();
    envelope.cells[CellType::Edge].connectivity.resize(3, 2);
    envelope.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 0;
    nlohmann::json options = {
        {"input", "tagged"},
        {"tag_selection", "tag_0"},
        {"DEBUG_output", true}};
    SECTION("without an envelope") {}
    SECTION("with an envelope")
    {
        options["envelope"] = "boundary";
    }
    const auto result = manifold_extraction::manifold_extraction(
        options,
        {{"tagged", data}, {"boundary", envelope}});
    check(result);
    REQUIRE(result.first.count("surface"));
    REQUIRE(result.first.at("surface").cells.count(CellType::Edge));
    // A 2D offset exports no input complex; its `_surf` export is the envelope curve.
    REQUIRE(!result.first.count("input_complex"));
    REQUIRE(result.first.count("envelope") == options.count("envelope"));
}
#endif
#ifdef WMTK_TEST_prismatic_mesh
TEST_CASE("Prismatic processing consumes construction fields in memory", "[component_io]")
{
    WorkingDirectory cwd;
    auto data = tet();
    data.vertex_attributes["labels"].values = (DataMatrix<int64_t>(4, 1) << 1, 0, 0, 2).finished();
    data.vertex_attributes["vid"].values = (DataMatrix<int64_t>(4, 1) << 10, 20, 30, 40).finished();
    data.vertex_attributes["corr_input_vid"].values =
        (DataMatrix<int64_t>(4, 1) << -1, -1, -1, 10).finished();
    SECTION("full-width source IDs")
    {
        data.vertex_attributes["vid"].values = (DataMatrix<int64_t>(4, 1) << 9007199254740993LL,
                                                9007199254740995LL,
                                                9007199254740997LL,
                                                9007199254740999LL)
                                                   .finished();
        data.vertex_attributes["corr_input_vid"].values =
            (DataMatrix<int64_t>(4, 1) << -1, -1, -1, 9007199254740993LL).finished();
    }
    SECTION("ordinary source IDs") {}
    auto& c = data.cells.at(CellType::Tetrahedron);
    c.attributes["tag_0"].values = DataMatrix<uint8_t>::Zero(1, 1).eval();
    c.attributes["offset_tag"].values = DataMatrix<int64_t>::Ones(1, 1).eval();
    const auto result = prismatic_mesh::prismatic_mesh(
        {{"input", "construction"}, {"iterations", 0}},
        {{"construction", data}});
    check(result);
    REQUIRE(
        result.first.at("mesh").vertex_attributes.at("vid").integers() ==
        data.vertex_attributes.at("vid").integers());
    REQUIRE(
        result.first.at("mesh").vertex_attributes.at("corr_input_vid").integers() ==
        data.vertex_attributes.at("corr_input_vid").integers());
}
#endif
#ifdef WMTK_TEST_c1_simplification
TEST_CASE("C1 consumes named surface and UV arrays", "[component_io]")
{
    WorkingDirectory cwd;
    auto surface = triangle(), uv = triangle();
    surface.vertex_attributes["is_cone"].values = DataMatrix<uint8_t>::Ones(3, 1).eval();
    surface.cells.at(CellType::Triangle).attributes["dofs"].values =
        DataMatrix<double>::Zero(1, 36).eval();
    check(
        c1_simplification::c1_simplification(
            {{"surface_mesh", "surface"}, {"uv_mesh", "uv"}},
            {{"surface", surface}, {"uv", uv}}));
}
#endif
#ifdef WMTK_TEST_qslim
TEST_CASE("Memory calls do not use a previously configured file logger", "[component_io]")
{
    WorkingDirectory cwd;
    const auto logfile =
        cwd.old /
        ("wmtk_logger_test_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".log");
    set_file_logger(logfile.string());
    logger().info("before memory call");
    logger().flush();
    const auto size = std::filesystem::file_size(logfile);
    check(
        qslim::qslim(
            {{"input", "surface"}, {"target_abs", 3}, {"num_threads", 1}},
            {{"surface", triangle()}}));
    logger().flush();
    REQUIRE(std::filesystem::file_size(logfile) == size);
    logger().info("after memory call");
    logger().flush();
    REQUIRE(std::filesystem::file_size(logfile) > size);
    set_logger(nullptr);
    std::filesystem::remove(logfile);
}
TEST_CASE("File and memory QSlim interfaces agree", "[component_io][io]")
{
    WorkingDirectory cwd;
    auto data = triangle();
    {
        std::ofstream f("input.obj");
        f << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    }
    const nlohmann::json memory_options = {
        {"input", "source"},
        {"target_abs", 3},
        {"num_threads", 1}};
    const auto result = qslim::qslim(memory_options, {{"source", data}});
    auto file_options = memory_options;
    file_options["application"] = "qslim";
    file_options["input"] = "input.obj";
    file_options["output"] = "output.obj";
    file_options["report"] = "report.json";
    qslim::qslim(file_options);
    const auto reread = io::read_mesh("output.obj");
    REQUIRE(result.first.at("mesh").V == reread.V);
    REQUIRE(
        result.first.at("mesh").cell(CellType::Triangle).connectivity ==
        reread.cell(CellType::Triangle).connectivity);
    auto report = nlohmann::json::parse(std::ifstream("report.json"));
    auto expected = result.second;
    report.erase("time_sec");
    expected.erase("time_sec");
    REQUIRE(report == expected);
}
TEST_CASE("Geometry inputs accept the fields read_mesh attaches", "[component_io][io]")
{
    WorkingDirectory cwd;
    {
        std::ofstream f("input.obj");
        f << "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nvt 0 0\nvn 0 0 1\n"
             "f 2/1/1 3/1/1 4/1/1\nf 1/1/1 4/1/1 3/1/1\nf 1/1/1 2/1/1 4/1/1\nf 1/1/1 3/1/1 2/1/1\n";
    }
    auto data = io::read_mesh("input.obj");
    std::filesystem::remove("input.obj");
    REQUIRE(data.cell(CellType::Triangle).attributes.count("uv"));
    REQUIRE(data.cell(CellType::Triangle).attributes.count("normal"));
    const auto result = qslim::qslim(
        {{"input", "source"}, {"target_abs", 4}, {"num_threads", 1}},
        {{"source", data}});
    check(result);
    // Any other field too: an annotation, dropped (io::ComponentIO::data_fields), the result as
    // without it. Until 2026-11 it was refused.
    data.vertex_attributes["unrelated"].values = DataMatrix<double>::Zero(4, 1).eval();
    data.cells.at(CellType::Triangle).attributes["unrelated"].values =
        DataMatrix<double>::Zero(4, 1).eval();
    const auto annotated = qslim::qslim(
        {{"input", "source"}, {"target_abs", 4}, {"num_threads", 1}},
        {{"source", data}});
    check(annotated);
    const auto& a = annotated.first.at("mesh");
    const auto& b = result.first.at("mesh");
    const auto& Fa = a.cell(CellType::Triangle).connectivity;
    const auto& Fb = b.cell(CellType::Triangle).connectivity;
    REQUIRE(a.V.rows() == b.V.rows());
    REQUIRE(a.V == b.V);
    REQUIRE(Fa.rows() == Fb.rows());
    REQUIRE(Fa == Fb);
}
TEST_CASE("Memory inputs with degenerate cells are cleaned like files", "[component_io][io]")
{
    WorkingDirectory cwd;
    {
        std::ofstream f("input.obj");
        f << "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
             "f 2 3 4\nf 1 4 3\nf 1 2 4\nf 1 3 2\nf 2 2 3\n";
    }
    const nlohmann::json memory_options = {
        {"input", "source"},
        {"target_abs", 4},
        {"num_threads", 1}};
    const auto result = qslim::qslim(memory_options, {{"source", io::read_mesh("input.obj")}});
    auto file_options = memory_options;
    file_options["application"] = "qslim";
    file_options["input"] = "input.obj";
    file_options["output"] = "output.obj";
    qslim::qslim(file_options);
    const auto reread = io::read_mesh("output.obj");
    REQUIRE(result.first.at("mesh").V == reread.V);
    REQUIRE(
        result.first.at("mesh").cell(CellType::Triangle).connectivity ==
        reread.cell(CellType::Triangle).connectivity);
}
#endif
#if defined(WMTK_TEST_simwild) && defined(WMTK_TEST_topological_offset) && \
    defined(WMTK_TEST_prismatic_mesh)
TEST_CASE(
    "Tagged meshing offset and prismatic processing compose in memory",
    "[component_io][memory_pipeline]" + release_only)
{
    WorkingDirectory cwd;
    auto surface = tet();
    surface.cells.clear();
    surface.cells[CellType::Triangle].connectivity.resize(4, 3);
    surface.cells[CellType::Triangle].connectivity << 1, 2, 3, 0, 3, 2, 0, 1, 3, 0, 2, 1;
    auto meshed = simwild::simwild(
        {{"input", {"surface"}},
         {"input_names", {"tag_0"}},
         {"max_iterations", 0},
         {"simplify", false},
         {"num_threads", 1},
         {"length_rel", 0.5},
         {"coarsen_pass", false}},
        {{"surface", surface}});
    check(meshed);
    auto offset = topological_offset::topological_offset(
        {{"input", "mesh"},
         {"envelope", "envelope"},
         {"offset_selection", "tag_0"},
         {"offset_output_tags", {"offset"}},
         {"optimize_offset", false},
         {"save_offset_correspondence", true},
         {"check_manifoldness", false},
         {"num_threads", 1}},
        meshed.first);
    check(offset);
    REQUIRE(offset.first.at("mesh").vertex_attributes.count("corr_input_vid"));
    auto prism =
        prismatic_mesh::prismatic_mesh({{"input", "mesh"}, {"iterations", 0}}, offset.first);
    check(prism);
    // The input-and-shell export returns what its files hold: mesh, offset faces, report.
    auto shell = prismatic_mesh::prismatic_mesh(
        {{"input", "mesh"},
         {"iterations", 0},
         {"export_input_and_shell", true},
         {"prism_dominant", true}},
        offset.first);
    check(shell);
    REQUIRE(shell.first.count("input_and_shell"));
    REQUIRE(shell.first.count("input_and_shell_offset") == shell.first.count("offset"));
    REQUIRE(shell.second.at("input_and_shell_hybrid").at("export_scope") == "input_and_shell");
    REQUIRE(!shell.second.at("hybrid").contains("export_scope"));
}
#endif

// The two routes must agree on real inputs: a memory call on io::read_mesh(file) and a file call
// on the same file give the same report (timings aside). The memory call runs first because
// check() requires that it writes nothing.
namespace {
[[maybe_unused]] std::string data_file(const std::string& name)
{
    return (std::filesystem::path(WMTK_DATA_DIR) / "models" / name).string();
}
[[maybe_unused]] nlohmann::json without_timings(nlohmann::json report)
{
    for (const auto* key : {"time", "time_sec", "insertion_and_preprocessing"}) report.erase(key);
    return report;
}
// The top-dimensional cells as sorted sets of corner positions: equal for the same geometry,
// whatever the vertex and cell order of the two outputs.
[[maybe_unused]] std::vector<std::vector<std::array<double, 3>>> cell_geometry(const MeshData& m)
{
    const auto type =
        m.cells.count(CellType::Tetrahedron) ? CellType::Tetrahedron : CellType::Triangle;
    const auto& c = m.cell(type).connectivity;
    std::vector<std::vector<std::array<double, 3>>> cells(c.rows());
    for (Eigen::Index r = 0; r < c.rows(); ++r) {
        for (Eigen::Index j = 0; j < c.cols(); ++j)
            cells[r].push_back({{m.V(c(r, j), 0), m.V(c(r, j), 1), m.V(c(r, j), 2)}});
        std::sort(cells[r].begin(), cells[r].end());
    }
    std::sort(cells.begin(), cells.end());
    return cells;
}
template <typename Component>
[[maybe_unused]] void compare_routes(
    Component component,
    const std::string& application,
    nlohmann::json options,
    const nlohmann::json& file_input,
    const nlohmann::json& memory_input,
    const MeshMap& meshes,
    const std::string& file_output = "out.msh")
{
    options["input"] = memory_input;
    const ComponentResult memory = component(options, meshes);
    check(memory);
    options["application"] = application;
    options["input"] = file_input;
    options["output"] = "out";
    options["report"] = "report.json";
    component(options);
    const auto file_report = nlohmann::json::parse(std::ifstream("report.json"));
    REQUIRE(without_timings(file_report) == without_timings(memory.second));
    REQUIRE(cell_geometry(io::read_mesh(file_output)) == cell_geometry(memory.first.at("mesh")));
}
} // namespace
#ifdef WMTK_TEST_tetwild
TEST_CASE(
    "TetWild gives the same result from memory and from a file",
    "[component_io][equivalence]" + release_only)
{
    WorkingDirectory cwd;
    const auto path = data_file("sphere.obj");
    compare_routes(
        [](auto&&... args) { return tetwild::tetwild(args...); },
        "tetwild",
        {{"max_iterations", 2}, {"num_threads", 1}},
        {path},
        {"surface"},
        {{"surface", io::read_mesh(path)}},
        "out_final.msh");
}
#endif
#ifdef WMTK_TEST_triwild
TEST_CASE(
    "TriWild gives the same result from memory and from a file",
    "[component_io][equivalence]" + release_only)
{
    WorkingDirectory cwd;
    const auto path = data_file("triwild_polylines.obj");
    compare_routes(
        [](auto&&... args) { return triwild::triwild(args...); },
        "triwild",
        {{"max_iterations", 2}, {"num_threads", 1}, {"stop_energy", 20}},
        {path},
        {"curves"},
        {{"curves", io::read_mesh(path)}});
}
#endif
#ifdef WMTK_TEST_simwild
TEST_CASE(
    "SimWild gives the same result from memory and from a file",
    "[component_io][equivalence]" + release_only)
{
    WorkingDirectory cwd;
    // Tagged .msh inputs: physical-group order, the embedded EnvelopeSurface and the ambient group
    // all have to come through read_mesh as the .msh route reads them.
    // bunny2d.msh holds its regions as label images (tag_<k> cell fields), which both routes
    // turn into regions.
    const auto file =
        GENERATE(as<std::string>{}, "topo_annots_groups.msh", "double_sphere.msh", "bunny2d.msh");
    INFO(file);
    const auto path = data_file(file);
    compare_routes(
        [](auto&&... args) { return simwild::simwild(args...); },
        "simwild",
        {{"max_iterations", 2}, {"num_threads", 1}},
        {path},
        {"tagged"},
        {{"tagged", io::read_mesh(path)}});
}
#endif
#ifdef WMTK_TEST_manifold_extraction
TEST_CASE("Manifold extraction leaves a manifold selection as it is", "[component_io]")
{
    // tag_0 alone is a sphere, whose boundary is manifold: nothing to offset. The 3D run used to
    // crash on the empty input complex.
    const MeshMap meshes{{"tagged", io::read_mesh(data_file("double_sphere_hole.msh"))}};
    const auto [outputs, report] = manifold_extraction::manifold_extraction(
        {{"input", "tagged"}, {"tag_selection", "tag_0"}},
        meshes);
    CHECK(report.at("after #t") == report.at("before #t"));
    CHECK(
        outputs.at("mesh").cell(CellType::Tetrahedron).connectivity.rows() ==
        meshes.at("tagged").cell(CellType::Tetrahedron).connectivity.rows());
}
#endif
#ifdef WMTK_TEST_manifold_extraction
TEST_CASE(
    "Manifold extraction gives the same result from memory and from a file",
    "[component_io][equivalence]" + release_only)
{
    WorkingDirectory cwd;
    const auto path = data_file("double_sphere_hole.msh");
    compare_routes(
        [](auto&&... args) { return manifold_extraction::manifold_extraction(args...); },
        "manifold_extraction",
        {{"tag_selection", "tag_0 | tag_1"}, {"fill_tags", nlohmann::json::array()}},
        path,
        "tagged",
        {{"tagged", io::read_mesh(path)}});
}
#endif
#ifdef WMTK_TEST_topological_offset
TEST_CASE(
    "Topological offset gives the same result from memory and from a file",
    "[component_io][equivalence]" + release_only)
{
    WorkingDirectory cwd;
    const auto path = data_file("double_sphere_hole.msh");
    compare_routes(
        [](auto&&... args) { return topological_offset::topological_offset(args...); },
        "topological_offset",
        {{"offset_selection", "tag_0"}, {"optimize_offset", false}, {"num_threads", 1}},
        path,
        "tagged",
        {{"tagged", io::read_mesh(path)}});
}
#endif
#if defined(WMTK_TEST_simwild) && (defined(WMTK_TEST_tetwild) || defined(WMTK_TEST_triwild))
namespace {
template <typename A, typename B>
bool equal_arrays(const A& x, const B& y)
{
    return x.rows() == y.rows() && x.cols() == y.cols() && x == y;
}
// Whether two meshes hold the same arrays, exactly: positions, connectivity, attributes, labels.
bool same_data(const MeshData& a, const MeshData& b)
{
    if (!equal_arrays(a.V, b.V) || a.cells.size() != b.cells.size() ||
        a.vertex_attributes.size() != b.vertex_attributes.size())
        return false;
    for (const auto& [type, ca] : a.cells) {
        const auto cb = b.cells.find(type);
        if (cb == b.cells.end() || !equal_arrays(ca.connectivity, cb->second.connectivity) ||
            ca.attributes.size() != cb->second.attributes.size())
            return false;
        for (const auto& [name, x] : ca.attributes) {
            const auto y = cb->second.attributes.find(name);
            if (y == cb->second.attributes.end() || x.values.index() != y->second.values.index() ||
                x.labels != y->second.labels || !equal_arrays(x.doubles(), y->second.doubles()))
                return false;
        }
    }
    return true;
}
} // namespace
#endif
#if defined(WMTK_TEST_tetwild) && defined(WMTK_TEST_simwild)
namespace {
// An icosahedron subdivided `levels` times onto the sphere of radius `r` about `c`, facing out.
MeshData icosphere(const int levels, const double r, const Vector3d& c)
{
    const double t = (1 + std::sqrt(5.0)) / 2;
    std::vector<Vector3d> V = {
        {-1, t, 0},
        {1, t, 0},
        {-1, -t, 0},
        {1, -t, 0},
        {0, -1, t},
        {0, 1, t},
        {0, -1, -t},
        {0, 1, -t},
        {t, 0, -1},
        {t, 0, 1},
        {-t, 0, -1},
        {-t, 0, 1}};
    for (Vector3d& v : V) v.normalize();
    std::vector<std::array<int64_t, 3>> F = {
        {{0, 11, 5}}, {{0, 5, 1}},  {{0, 1, 7}},   {{0, 7, 10}}, {{0, 10, 11}},
        {{1, 5, 9}},  {{5, 11, 4}}, {{11, 10, 2}}, {{10, 7, 6}}, {{7, 1, 8}},
        {{3, 9, 4}},  {{3, 4, 2}},  {{3, 2, 6}},   {{3, 6, 8}},  {{3, 8, 9}},
        {{4, 9, 5}},  {{2, 4, 11}}, {{6, 2, 10}},  {{8, 6, 7}},  {{9, 8, 1}}};
    for (int level = 0; level < levels; ++level) {
        std::map<std::pair<int64_t, int64_t>, int64_t> middle;
        const auto mid = [&](const int64_t p, const int64_t q) {
            const auto [it, added] =
                middle.emplace(std::make_pair(std::min(p, q), std::max(p, q)), int64_t(V.size()));
            if (added) V.push_back((V[p] + V[q]).normalized());
            return it->second;
        };
        std::vector<std::array<int64_t, 3>> G;
        for (const auto& f : F) {
            const int64_t x = mid(f[0], f[1]), y = mid(f[1], f[2]), z = mid(f[2], f[0]);
            G.push_back({{f[0], x, z}});
            G.push_back({{f[1], y, x}});
            G.push_back({{f[2], z, y}});
            G.push_back({{x, y, z}});
        }
        F = std::move(G);
    }
    MeshData data;
    data.V.resize(V.size(), 3);
    for (size_t i = 0; i < V.size(); ++i) data.V.row(i) = (c + r * V[i]).transpose();
    auto& C = data.cells[CellType::Triangle].connectivity;
    C.resize(F.size(), 3);
    for (size_t i = 0; i < F.size(); ++i)
        for (int j = 0; j < 3; ++j) C(i, j) = F[i][j];
    return data;
}
} // namespace
TEST_CASE(
    "TetWild hands on its envelope and SimWild holds a tagged mesh to it on both routes",
    "[component_io][io][envelope]" + release_only)
{
    // Two spheres `a` and `b`, and a segment inside `a` as a feature file. tetwild simplifies
    // them within half its eps, so the faces it hands on leave the spheres by up to that much:
    // a run at a tenth of its eps cannot hold them. simwild remeshes tetwild's mesh held to the
    // envelope tetwild returns, the spheres as given, at tetwild's eps, and must return a mesh
    // within that eps of them -- of the original input, not of the mesh it received.
    WorkingDirectory cwd;
    const MeshData a = icosphere(3, 1, Vector3d::Zero());
    const MeshData b = icosphere(3, 0.5, Vector3d(2.5, 0, 0));
    MeshData curve;
    curve.V.resize(2, 3);
    curve.V << -0.5, 0.1, 0.1, 0.5, 0.2, 0.1;
    curve.cells[CellType::Edge].connectivity.resize(1, 2);
    curve.cells[CellType::Edge].connectivity << 0, 1;
    const nlohmann::json tetwild_options = {
        {"input_names", {"a", "b"}},
        {"eps_rel", 1e-2},
        {"eps_simplify_rel", 5e-3},
        {"max_iterations", 2},
        {"num_threads", 1}};
    auto memory_options = tetwild_options;
    memory_options["input"] = {"a", "b"};
    memory_options["input_edges"] = {"curve"};
    const auto meshed = tetwild::tetwild(memory_options, {{"a", a}, {"b", b}, {"curve", curve}});
    check(meshed);
    write_obj("a.obj", a);
    write_obj("b.obj", b);
    write_obj("curve.obj", curve);
    auto file_options = tetwild_options;
    file_options["application"] = "tetwild";
    file_options["input"] = {here("a.obj"), here("b.obj")};
    file_options["input_edges"] = {here("curve.obj")};
    file_options["output"] = "t";
    file_options["write_vtu"] = false;
    tetwild::tetwild(file_options);
    const double eps = meshed.second.at("eps");
    const MeshData envelope = io::read_mesh("t_envelope.msh");

    // The envelope is the input, labelled as the mesh's tags: each sphere's triangles under its
    // name, the segment as feature_0. The same on both routes.
    REQUIRE(meshed.first.count("envelope"));
    const MeshData& memory_envelope = meshed.first.at("envelope");
    for (const MeshData* e : {&memory_envelope, &envelope}) {
        REQUIRE(
            e->cell(CellType::Triangle).attributes.at("tags").labels ==
            std::vector<std::string>{"a", "b"});
        REQUIRE(
            e->cell(CellType::Edge).attributes.at("tags").labels ==
            std::vector<std::string>{"feature_0"});
        CHECK(
            labelled_rows(*e, CellType::Triangle, "a").size() ==
            size_t(a.cell(CellType::Triangle).connectivity.rows()));
        CHECK(
            labelled_rows(*e, CellType::Triangle, "b").size() ==
            size_t(b.cell(CellType::Triangle).connectivity.rows()));
        CHECK(labelled_rows(*e, CellType::Edge, "feature_0").size() == 1);
    }
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(memory_envelope, CellType::Triangle, name)) ==
            canonical(labelled_rows(envelope, CellType::Triangle, name)));
    }
    CHECK(
        canonical(labelled_rows(memory_envelope, CellType::Edge, "feature_0")) ==
        canonical(labelled_rows(envelope, CellType::Edge, "feature_0")));

    // simwild on tetwild's mesh, held to the envelope at tetwild's eps, on both routes: on the
    // file route the envelope is a path relative to input_dir.
    const MeshData tagged = tags_only(io::read_mesh("t_final.msh"));
    const nlohmann::json simwild_options = {
        {"eps", eps},
        {"max_iterations", 2},
        {"num_threads", 1}};
    auto simwild_memory = simwild_options;
    simwild_memory["input"] = "tagged";
    simwild_memory["envelope"] = "envelope";
    const MeshMap inputs{{"tagged", tagged}, {"envelope", envelope}};
    const auto remeshed = simwild::simwild(simwild_memory, inputs);
    for (const auto& [name, data] : remeshed.first) REQUIRE_NOTHROW(data.validate());
    auto simwild_file = simwild_options;
    simwild_file["application"] = "simwild";
    simwild_file["input"] = {here("t_final.msh")};
    simwild_file["input_dir"] = std::filesystem::current_path().string();
    simwild_file["envelope"] = "t_envelope.msh";
    simwild_file["output"] = "s";
    simwild_file["write_vtu"] = false;
    simwild::simwild(simwild_file);
    const MeshData file = io::read_mesh("s.msh");

    // The envelope comes back as given, for the next component, on both routes.
    REQUIRE(remeshed.first.count("envelope"));
    CHECK(same_data(remeshed.first.at("envelope"), envelope));
    CHECK(same_data(io::read_mesh("s_envelope.msh"), envelope));
    // The same mesh on both routes.
    const MeshData& memory = remeshed.first.at("mesh");
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(memory, CellType::Triangle, name)) ==
            canonical(labelled_rows(file, CellType::Triangle, name)));
    }
    // A feature curve has no direction: each edge's ends in order of position.
    const auto undirected = [](std::vector<std::vector<Vector3d>> rows) {
        for (auto& e : rows) {
            if (std::lexicographical_compare(
                    e[1].data(),
                    e[1].data() + 3,
                    e[0].data(),
                    e[0].data() + 3))
                std::swap(e[0], e[1]);
        }
        return canonical(rows);
    };
    CHECK(
        undirected(labelled_rows(memory, CellType::Edge, "feature_0")) ==
        undirected(labelled_rows(file, CellType::Edge, "feature_0")));

    // Every face within eps of its own sphere as given to tetwild, in its exact envelope and as
    // measured at its corners, edge midpoints and centroid; every feature edge within eps of the
    // segment (feature_envelope_ratio 1).
    const Vector3d p = curve.V.row(0).transpose(), q = curve.V.row(1).transpose();
    const auto off_segment = [&](const Vector3d& x) {
        const Vector3d d = q - p;
        const double s = std::clamp((x - p).dot(d) / d.squaredNorm(), 0.0, 1.0);
        return (p + s * d - x).norm();
    };
    for (const MeshData* mesh : {&memory, &file}) {
        const std::array<std::pair<std::string, const MeshData*>, 2> spheres = {
            {{"a", &a}, {"b", &b}}};
        for (const auto& [name, sphere] : spheres) {
            std::vector<Vector3d> V;
            std::vector<Vector3i> F;
            for (Eigen::Index v = 0; v < sphere->V.rows(); ++v)
                V.push_back(sphere->V.row(v).transpose());
            const auto& C = sphere->cell(CellType::Triangle).connectivity;
            for (Eigen::Index f = 0; f < C.rows(); ++f)
                F.emplace_back(int(C(f, 0)), int(C(f, 1)), int(C(f, 2)));
            Envelope exact(true);
            exact.init(V, F, eps);
            const auto rows = labelled_rows(*mesh, CellType::Triangle, name);
            REQUIRE(!rows.empty());
            size_t outside = 0;
            double farthest = 0;
            for (const auto& f : rows) {
                outside += exact.is_outside(std::array<Vector3d, 3>{{f[0], f[1], f[2]}});
                const std::array<Vector3d, 7> samples = {
                    {f[0],
                     f[1],
                     f[2],
                     Vector3d((f[0] + f[1]) / 2),
                     Vector3d((f[1] + f[2]) / 2),
                     Vector3d((f[2] + f[0]) / 2),
                     Vector3d((f[0] + f[1] + f[2]) / 3)}};
                for (const Vector3d& x : samples)
                    farthest = std::max(farthest, std::sqrt(exact.squared_distance(x)));
            }
            INFO(name << ": " << rows.size() << " faces, farthest " << farthest / eps << " eps");
            CHECK(outside == 0);
            CHECK(farthest <= eps * (1 + 1e-9));
        }
        const auto edges = labelled_rows(*mesh, CellType::Edge, "feature_0");
        REQUIRE(!edges.empty());
        for (const auto& e : edges)
            for (const Vector3d& x : e) CHECK(off_segment(x) <= eps * (1 + 1e-9));
    }

    // At a tenth of tetwild's eps the faces it simplified lie outside: refused, naming the label.
    auto narrow = simwild_memory;
    narrow["eps"] = eps / 10;
    REQUIRE_THROWS_WITH(
        simwild::simwild(narrow, inputs),
        Catch::Matchers::ContainsSubstring("held faces outside") &&
            Catch::Matchers::ContainsSubstring("a ("));
    // A label the mesh holds that the envelope lacks, of a face or of a feature curve: refused,
    // naming it.
    MeshData renamed = envelope;
    renamed.cells.at(CellType::Triangle).attributes.at("tags").labels = {"a", "c"};
    REQUIRE_THROWS_WITH(
        simwild::simwild(simwild_memory, {{"tagged", tagged}, {"envelope", renamed}}),
        Catch::Matchers::ContainsSubstring("labelled b"));
    renamed = envelope;
    renamed.cells.at(CellType::Edge).attributes.at("tags").labels = {"feature_1"};
    REQUIRE_THROWS_WITH(
        simwild::simwild(simwild_memory, {{"tagged", tagged}, {"envelope", renamed}}),
        Catch::Matchers::ContainsSubstring("labelled feature_0"));
}
#endif
#if defined(WMTK_TEST_triwild) && defined(WMTK_TEST_simwild)
namespace {
// The closed polyline of `n` segments around the circle of radius `r` about (cx, 0),
// counter-clockwise.
MeshData circle(const int n, const double r, const double cx)
{
    MeshData data;
    data.V = DataMatrix<double>::Zero(n, 3);
    auto& E = data.cells[CellType::Edge].connectivity;
    E.resize(n, 2);
    for (int i = 0; i < n; ++i) {
        const double angle = 2 * std::acos(-1.0) * i / n;
        data.V(i, 0) = cx + r * std::cos(angle);
        data.V(i, 1) = r * std::sin(angle);
        E(i, 0) = i;
        E(i, 1) = (i + 1) % n;
    }
    return data;
}
} // namespace
TEST_CASE(
    "TriWild hands on its envelope and SimWild holds a tagged mesh to it on both routes",
    "[component_io][io][envelope]" + release_only)
{
    // The 2D twin of the case above: two circles `a` and `b`, simplified by triwild within half
    // its eps, remeshed by simwild held to triwild's envelope at triwild's eps.
    WorkingDirectory cwd;
    const MeshData a = circle(64, 1, 0), b = circle(64, 0.5, 2.5);
    const nlohmann::json triwild_options = {
        {"input_names", {"a", "b"}},
        {"eps_rel", 1e-2},
        {"eps_simplify_rel", 5e-3},
        {"max_iterations", 2},
        {"num_threads", 1}};
    auto memory_options = triwild_options;
    memory_options["input"] = {"a", "b"};
    const auto meshed = triwild::triwild(memory_options, {{"a", a}, {"b", b}});
    check(meshed);
    write_obj("a.obj", a);
    write_obj("b.obj", b);
    auto file_options = triwild_options;
    file_options["application"] = "triwild";
    file_options["input"] = {here("a.obj"), here("b.obj")};
    file_options["output"] = "t";
    file_options["write_vtu"] = false;
    triwild::triwild(file_options);
    const double eps = meshed.second.at("eps");
    const MeshData envelope = io::read_mesh("t_envelope.msh");

    // The envelope is the input curves, each segment under its curve's name, on both routes.
    REQUIRE(meshed.first.count("envelope"));
    for (const MeshData* e : {&meshed.first.at("envelope"), &envelope}) {
        REQUIRE(
            e->cell(CellType::Edge).attributes.at("tags").labels ==
            std::vector<std::string>{"a", "b"});
        CHECK(labelled_rows(*e, CellType::Edge, "a").size() == 64);
        CHECK(labelled_rows(*e, CellType::Edge, "b").size() == 64);
    }
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(meshed.first.at("envelope"), CellType::Edge, name)) ==
            canonical(labelled_rows(envelope, CellType::Edge, name)));
    }

    const MeshData tagged = tags_only(io::read_mesh("t.msh"));
    const nlohmann::json simwild_options = {
        {"eps", eps},
        {"max_iterations", 2},
        {"num_threads", 1}};
    auto simwild_memory = simwild_options;
    simwild_memory["input"] = "tagged";
    simwild_memory["envelope"] = "envelope";
    const MeshMap inputs{{"tagged", tagged}, {"envelope", envelope}};
    const auto remeshed = simwild::simwild(simwild_memory, inputs);
    for (const auto& [name, data] : remeshed.first) REQUIRE_NOTHROW(data.validate());
    auto simwild_file = simwild_options;
    simwild_file["application"] = "simwild";
    simwild_file["input"] = {here("t.msh")};
    simwild_file["input_dir"] = std::filesystem::current_path().string();
    simwild_file["envelope"] = "t_envelope.msh";
    simwild_file["output"] = "s";
    simwild_file["write_vtu"] = false;
    simwild::simwild(simwild_file);
    const MeshData file = io::read_mesh("s.msh");

    REQUIRE(remeshed.first.count("envelope"));
    CHECK(same_data(remeshed.first.at("envelope"), envelope));
    CHECK(same_data(io::read_mesh("s_envelope.msh"), envelope));
    const MeshData& memory = remeshed.first.at("mesh");
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(memory, CellType::Edge, name)) ==
            canonical(labelled_rows(file, CellType::Edge, name)));
    }
    // Every tracked edge within eps of its own circle as given to triwild.
    for (const MeshData* mesh : {&memory, &file}) {
        const std::array<std::pair<std::string, const MeshData*>, 2> circles = {
            {{"a", &a}, {"b", &b}}};
        for (const auto& [name, input] : circles) {
            std::vector<Vector2d> V;
            std::vector<Vector2i> E;
            for (Eigen::Index v = 0; v < input->V.rows(); ++v)
                V.emplace_back(input->V(v, 0), input->V(v, 1));
            const auto& C = input->cell(CellType::Edge).connectivity;
            for (Eigen::Index e = 0; e < C.rows(); ++e) E.emplace_back(int(C(e, 0)), int(C(e, 1)));
            Envelope exact(true);
            exact.init(V, E, eps);
            const auto rows = labelled_rows(*mesh, CellType::Edge, name);
            REQUIRE(!rows.empty());
            size_t outside = 0;
            double farthest = 0;
            for (const auto& e : rows) {
                const Vector2d x = e[0].head<2>(), y = e[1].head<2>();
                outside += exact.is_outside(std::array<Vector2d, 2>{{x, y}});
                for (const Vector2d& s : {x, y, Vector2d((x + y) / 2)})
                    farthest = std::max(farthest, std::sqrt(exact.squared_distance(s)));
            }
            INFO(name << ": " << rows.size() << " edges, farthest " << farthest / eps << " eps");
            CHECK(outside == 0);
            CHECK(farthest <= eps * (1 + 1e-9));
        }
    }

    auto narrow = simwild_memory;
    narrow["eps"] = eps / 10;
    REQUIRE_THROWS_WITH(
        simwild::simwild(narrow, inputs),
        Catch::Matchers::ContainsSubstring("held edges outside") &&
            Catch::Matchers::ContainsSubstring("a ("));
    MeshData renamed = envelope;
    renamed.cells.at(CellType::Edge).attributes.at("tags").labels = {"a", "c"};
    REQUIRE_THROWS_WITH(
        simwild::simwild(simwild_memory, {{"tagged", tagged}, {"envelope", renamed}}),
        Catch::Matchers::ContainsSubstring("labelled b"));
}
#endif
#ifdef WMTK_TEST_tetwild
namespace {
// A segment inside the unit tet, as a feature file.
MeshData inner_segment()
{
    MeshData curve;
    curve.V.resize(2, 3);
    curve.V << 0.1, 0.1, 0.1, 0.3, 0.2, 0.1;
    curve.cells[CellType::Edge].connectivity.resize(1, 2);
    curve.cells[CellType::Edge].connectivity << 0, 1;
    return curve;
}
// tetwild on two unit tets 3 apart, `a` and `b`, and inner_segment() as a feature file, without
// an optimization: a tet mesh of the box around them with tagged faces and a feature curve.
ComponentResult two_tets_meshed(const nlohmann::json& extra = nlohmann::json::object())
{
    nlohmann::json options = {
        {"input", {"a", "b"}},
        {"input_names", {"a", "b"}},
        {"input_edges", {"curve"}},
        {"simplify", false},
        {"max_iterations", 0},
        {"num_threads", 1}};
    options.update(extra);
    return tetwild::tetwild(
        options,
        {{"a", tet_surface(0)}, {"b", tet_surface(3)}, {"curve", inner_segment()}});
}
// The volume of `mesh`'s tets, and the largest x of their centroids.
std::pair<double, double> volume_and_max_x(const MeshData& mesh)
{
    const auto& T = mesh.cell(CellType::Tetrahedron).connectivity;
    double volume = 0, max_x = -std::numeric_limits<double>::infinity();
    for (Eigen::Index t = 0; t < T.rows(); ++t) {
        std::array<Vector3d, 4> p;
        for (int k = 0; k < 4; ++k) p[k] = mesh.V.row(T(t, k)).transpose();
        volume += std::abs((p[1] - p[0]).cross(p[2] - p[0]).dot(p[3] - p[0])) / 6;
        max_x = std::max(max_x, (p[0] + p[1] + p[2] + p[3]).x() / 4);
    }
    return {volume, max_x};
}
// Each face of `mesh` tagged `name` on its unit tet (`a` at x 0, `b` at x 3) and facing out of it.
void check_on_own_input_facing_out(const MeshData& mesh, const std::string& name)
{
    INFO(name);
    const Vector3d centre(name == "a" ? 0.25 : 3.25, 0.25, 0.25);
    const auto rows = labelled_rows(mesh, CellType::Triangle, name);
    CHECK(rows.size() >= 4);
    for (const auto& f : rows) {
        const Vector3d c = (f[0] + f[1] + f[2]) / 3;
        CHECK(std::abs(c.x() - centre.x()) < 1);
        CHECK((f[1] - f[0]).cross(f[2] - f[0]).dot(c - centre) > 0);
    }
}
} // namespace
TEST_CASE(
    "TetWild continues from a tagged tet mesh, its tets kept, on both routes",
    "[component_io][io][tagged]" + release_only)
{
    // tetwild's own tagged mesh with its fields dropped: a tet mesh with face tags `a` and `b`
    // and a feature curve feature_0. The tagged route reads it exactly -- every tagged face and
    // edge on its row, with its labels and its orientation: the reference it hands on, built from
    // what it received, is those rows -- and gives one result on both routes, with the labels
    // kept, no tet tags, and each face on its own input and facing out of it. Not the mesh as it
    // came at max_iterations 0: mesh_improvement() runs its bare collapse and swap passes before
    // the first iteration and after the last on both routes (optimization_bare_coarsen_passes(),
    // as after an insertion), whatever the count.
    WorkingDirectory cwd;
    const MeshData tagged = tags_only(two_tets_meshed().first.at("mesh"));
    const nlohmann::json options = {{"max_iterations", 0}, {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    const auto result = tetwild::tetwild(memory_options, {{"m", tagged}});
    check(result);
    auto optimizing = memory_options;
    optimizing["max_iterations"] = 3;
    const auto optimized = tetwild::tetwild(optimizing, {{"m", tagged}});
    check(optimized);

    io::write_mesh("m.msh", tagged);
    auto file_options = options;
    file_options["application"] = "tetwild";
    file_options["input"] = {here("m.msh")};
    file_options["output"] = "t";
    file_options["write_vtu"] = false;
    tetwild::tetwild(file_options);
    const MeshData file = io::read_mesh("t_final.msh");

    const MeshData& memory = result.first.at("mesh");
    for (const MeshData* mesh : {&memory, &file}) {
        REQUIRE(
            mesh->cell(CellType::Triangle).attributes.at("tags").labels ==
            std::vector<std::string>{"a", "b"});
        REQUIRE(
            mesh->cell(CellType::Edge).attributes.at("tags").labels ==
            std::vector<std::string>{"feature_0"});
        CHECK(!mesh->cell(CellType::Tetrahedron).attributes.count("tags"));
        for (const std::string name : {"a", "b"}) check_on_own_input_facing_out(*mesh, name);
    }
    // One result on both routes.
    CHECK(
        memory.cell(CellType::Tetrahedron).connectivity.rows() ==
        file.cell(CellType::Tetrahedron).connectivity.rows());
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(memory, CellType::Triangle, name)) ==
            canonical(labelled_rows(file, CellType::Triangle, name)));
    }
    CHECK(
        canonical(labelled_rows(memory, CellType::Edge, "feature_0")) ==
        canonical(labelled_rows(file, CellType::Edge, "feature_0")));
    // The reference handed on, without an `envelope` given: the tagged faces and the curve as
    // received, exactly the input's rows, labelled as the mesh labels them, on both routes.
    const MeshData file_envelope = io::read_mesh("t_envelope.msh");
    for (const MeshData* envelope : {&result.first.at("envelope"), &file_envelope}) {
        for (const std::string name : {"a", "b"}) {
            CHECK(
                canonical(labelled_rows(*envelope, CellType::Triangle, name)) ==
                canonical(labelled_rows(tagged, CellType::Triangle, name)));
        }
        CHECK(
            canonical(labelled_rows(*envelope, CellType::Edge, "feature_0")) ==
            canonical(labelled_rows(tagged, CellType::Edge, "feature_0")));
    }

    const MeshData& mesh = optimized.first.at("mesh");
    REQUIRE(
        mesh.cell(CellType::Triangle).attributes.at("tags").labels ==
        std::vector<std::string>{"a", "b"});
    for (const std::string name : {"a", "b"}) check_on_own_input_facing_out(mesh, name);
    CHECK(!labelled_rows(mesh, CellType::Edge, "feature_0").empty());
}
TEST_CASE(
    "TetWild takes its own output back in as it is, on both routes",
    "[component_io][io][tagged]" + release_only)
{
    // tetwild -> tetwild: the memory route's mesh with every field tetwild writes, and the file
    // route's _final.msh.
    WorkingDirectory cwd;
    const auto first = two_tets_meshed({{"max_iterations", 2}});
    const nlohmann::json options = {{"max_iterations", 2}, {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    const auto again = tetwild::tetwild(memory_options, {{"m", first.first.at("mesh")}});
    check(again);

    write_obj("a.obj", tet_surface(0));
    write_obj("b.obj", tet_surface(3));
    write_obj("curve.obj", inner_segment());
    tetwild::tetwild(
        {{"application", "tetwild"},
         {"input", {here("a.obj"), here("b.obj")}},
         {"input_names", {"a", "b"}},
         {"input_edges", {here("curve.obj")}},
         {"simplify", false},
         {"max_iterations", 2},
         {"num_threads", 1},
         {"output", "t1"},
         {"write_vtu", false}});
    auto file_options = options;
    file_options["application"] = "tetwild";
    file_options["input"] = {here("t1_final.msh")};
    file_options["output"] = "t2";
    file_options["write_vtu"] = false;
    tetwild::tetwild(file_options);
    const MeshData file = io::read_mesh("t2_final.msh");

    for (const MeshData* mesh : {&again.first.at("mesh"), &file}) {
        REQUIRE(
            mesh->cell(CellType::Triangle).attributes.at("tags").labels ==
            std::vector<std::string>{"a", "b"});
        REQUIRE(
            mesh->cell(CellType::Edge).attributes.at("tags").labels ==
            std::vector<std::string>{"feature_0"});
        for (const std::string name : {"a", "b"}) check_on_own_input_facing_out(*mesh, name);
    }
}
TEST_CASE(
    "TetWild refuses a tagged tet mesh with volume tags, naming them",
    "[component_io][tagged]" + release_only)
{
    WorkingDirectory cwd;
    auto volume = tet();
    add_tags(volume, CellType::Tetrahedron);
    REQUIRE_THROWS_WITH(
        tetwild::tetwild({{"input", "m"}, {"num_threads", 1}}, {{"m", volume}}),
        Catch::Matchers::ContainsSubstring("tag_0") &&
            Catch::Matchers::ContainsSubstring("simwild"));
    // ambient, and a label no tet carries, are no volumes.
    MeshData tagged = tags_only(two_tets_meshed().first.at("mesh"));
    auto& tets = tagged.cells.at(CellType::Tetrahedron);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(tets.connectivity.rows(), 2);
    member.col(0).setOnes();
    tets.attributes["tags"] = {member, {"ambient", "unused"}};
    check(
        tetwild::tetwild(
            {{"input", "m"}, {"max_iterations", 0}, {"num_threads", 1}},
            {{"m", tagged}}));
}
TEST_CASE(
    "TetWild refuses what its tagged route has no input for",
    "[component_io][tagged]" + release_only)
{
    WorkingDirectory cwd;
    const MeshData tagged = tags_only(two_tets_meshed().first.at("mesh"));
    const MeshMap meshes{{"m", tagged}, {"s", tet_surface(5)}, {"curve", inner_segment()}};
    const auto run = [&](nlohmann::json options) {
        if (!options.contains("input")) options["input"] = "m";
        options["num_threads"] = 1;
        options["max_iterations"] = 0;
        return tetwild::tetwild(options, meshes);
    };
    using Catch::Matchers::ContainsSubstring;
    REQUIRE_THROWS_WITH(run({{"input", {"m", "s"}}}), ContainsSubstring("only input"));
    REQUIRE_THROWS_WITH(run({{"input", {"s", "m"}}}), ContainsSubstring("only input"));
    REQUIRE_THROWS_WITH(run({{"filter", "input"}}), ContainsSubstring("filter \"input\""));
    REQUIRE_THROWS_WITH(run({{"input_edges", {"curve"}}}), ContainsSubstring("input_edges"));
    REQUIRE_THROWS_WITH(run({{"input_names", {"x"}}}), ContainsSubstring("input_names"));
    REQUIRE_THROWS_WITH(
        run({{"filter", "flood"}, {"filter_inputs", {"c"}}}),
        ContainsSubstring("c is not an input"));
    REQUIRE_THROWS_WITH(run({{"filter_inputs", {"a"}}}), ContainsSubstring("nothing is filtered"));
    // A surface input takes no envelope.
    REQUIRE_THROWS_WITH(
        run({{"input", "s"}, {"envelope", "m"}}),
        ContainsSubstring("tagged mesh input only"));
}
TEST_CASE(
    "TetWild filter_inputs: only the inputs named bound the domain kept",
    "[component_io][tagged]" + release_only)
{
    // Two unit tets 3 apart, `a` and `b`: every filter keeps both by default, and with
    // filter_inputs a only a, its volume 1/6 exactly as inserted, b's faces left in the part
    // removed. On the tagged route too, by a's face-tag label, except "input", refused there.
    WorkingDirectory cwd;
    const std::string filter = GENERATE("input", "tracked", "flood");
    INFO("filter " << filter);
    const auto both = volume_and_max_x(two_tets_meshed({{"filter", filter}}).first.at("mesh"));
    CHECK(std::abs(both.first - 2. / 6) < 1e-9);
    CHECK(both.second > 2);
    const auto only_a = volume_and_max_x(
        two_tets_meshed({{"filter", filter}, {"filter_inputs", {"a"}}}).first.at("mesh"));
    CHECK(std::abs(only_a.first - 1. / 6) < 1e-9);
    CHECK(only_a.second < 1);
    if (filter == "input") return;
    const MeshData tagged = two_tets_meshed().first.at("mesh");
    const auto tagged_a = volume_and_max_x(
        tetwild::tetwild(
            {{"input", "m"},
             {"filter", filter},
             {"filter_inputs", {"a"}},
             {"max_iterations", 0},
             {"num_threads", 1}},
            {{"m", tagged}})
            .first.at("mesh"));
    CHECK(std::abs(tagged_a.first - 1. / 6) < 1e-9);
    CHECK(tagged_a.second < 1);
}
#endif
#if defined(WMTK_TEST_tetwild) && defined(WMTK_TEST_topological_offset)
TEST_CASE(
    "TetWild refuses an offset's output, whose band is a volume",
    "[component_io][tagged]" + release_only)
{
    // tetwild -> topological_offset -> tetwild: the offset tags the band around `a` as tets,
    // `off`, a volume tetwild does not hold. Without feature curves, which the offset does not
    // take yet.
    WorkingDirectory cwd;
    const auto offset = topological_offset::topological_offset(
        {{"input", "m"},
         {"offset_selection", "a"},
         {"target_distance", 0.05},
         {"offset_output_tags", {"off"}},
         {"optimize_offset", false},
         {"num_threads", 1}},
        {{"m",
          tags_only(
              two_tets_meshed({{"input_edges", nlohmann::json::array()}}).first.at("mesh"))}});
    REQUIRE_THROWS_WITH(
        tetwild::tetwild(
            {{"input", "m"}, {"num_threads", 1}},
            {{"m", tags_only(offset.first.at("mesh"))}}),
        Catch::Matchers::ContainsSubstring("off") && Catch::Matchers::ContainsSubstring("simwild"));
}
#endif
#if defined(WMTK_TEST_tetwild) && defined(WMTK_TEST_topological_offset)
namespace {
// The offset of `a`'s surface in two_tets_meshed()'s mesh, without feature curves (the offset
// takes none), its band tagged `off`, held to the envelope `e`, optimized as users run it.
const nlohmann::json offset_of_a = {
    {"input", "m"},
    {"envelope", "e"},
    {"offset_selection", "a"},
    {"target_distance", 0.05},
    {"offset_output_tags", {"off"}},
    {"throw_on_nonconvergence", true},
    {"num_threads", 1}};
} // namespace
TEST_CASE(
    "The offset refuses a reference it does not lie in, or one without a label of its mesh",
    "[component_io][envelope][tagged]" + release_only)
{
    // tetwild's envelope moved by 10 of its eps, which the offset holds to: every face of a and b
    // is outside, named by label; and with b renamed c, b is missing from it. The checks
    // simwild's and tetwild's tagged routes run, by the same code (TetOptimizerMesh).
    WorkingDirectory cwd;
    using Catch::Matchers::ContainsSubstring;
    const auto meshed = two_tets_meshed({{"input_edges", nlohmann::json::array()}});
    const MeshData mesh_in = tags_only(meshed.first.at("mesh"));
    const MeshData& envelope = meshed.first.at("envelope");
    const double eps = meshed.second.at("eps");
    auto options = offset_of_a;
    options["envelope_size"] = eps * (1 + 1e-9);
    MeshData moved = envelope;
    moved.V.col(0).array() += 10 * eps;
    REQUIRE_THROWS_WITH(
        topological_offset::topological_offset(options, {{"m", mesh_in}, {"e", moved}}),
        ContainsSubstring("held faces outside the envelopes of their labels") &&
            ContainsSubstring("a (") && ContainsSubstring("b ("));
    MeshData renamed = envelope;
    renamed.cells.at(CellType::Triangle).attributes.at("tags").labels = {"a", "c"};
    REQUIRE_THROWS_WITH(
        topological_offset::topological_offset(options, {{"m", mesh_in}, {"e", renamed}}),
        ContainsSubstring("labelled b"));
}
TEST_CASE(
    "The offset refuses a filtered tetwild output, its boundary its tracked faces",
    "[component_io][tagged]" + release_only)
{
    // The two tets filtered to their insides: the mesh's boundary is the faces of a and b. The
    // offset holds its domain's wall by the box rule, so it takes a mesh that fills its box
    // (require_box_boundary()), as tetwild's tagged route does.
    WorkingDirectory cwd;
    const auto filtered =
        two_tets_meshed({{"input_edges", nlohmann::json::array()}, {"filter", "tracked"}});
    auto options = offset_of_a;
    options.erase("envelope");
    REQUIRE_THROWS_WITH(
        topological_offset::topological_offset(
            options,
            {{"m", tags_only(filtered.first.at("mesh"))}}),
        Catch::Matchers::Matches(
            "[1-9][0-9]* tracked faces of the tagged mesh lie on its boundary.*") &&
            Catch::Matchers::ContainsSubstring("filter \"none\""));
}
#endif
#if defined(WMTK_TEST_tetwild) && defined(WMTK_TEST_topological_offset) && \
    defined(WMTK_TEST_simwild)
namespace {
// `rows` (labelled_rows()) with each row's corners sorted, the rows sorted: equal for the same
// triangles in any order and orientation.
std::vector<std::vector<std::array<double, 3>>> unoriented(
    const std::vector<std::vector<Vector3d>>& rows)
{
    auto out = canonical(rows);
    for (auto& r : out) std::sort(r.begin(), r.end());
    std::sort(out.begin(), out.end());
    return out;
}
// The faces between a tet of `mesh` tagged `name` and a tet that is not, as labelled_rows() gives
// rows: the boundary of a region inside the mesh.
std::vector<std::vector<Vector3d>> region_boundary(const MeshData& mesh, const std::string& name)
{
    const auto& cells = mesh.cell(CellType::Tetrahedron);
    const auto& tags = cells.attributes.at("tags");
    const auto column = std::find(tags.labels.begin(), tags.labels.end(), name);
    REQUIRE(column != tags.labels.end());
    const auto member = tags.integers();
    std::map<std::array<int64_t, 3>, std::vector<bool>> sides;
    const auto& T = cells.connectivity;
    for (Eigen::Index t = 0; t < T.rows(); ++t) {
        for (int skip = 0; skip < 4; ++skip) {
            std::array<int64_t, 3> f;
            for (int c = 0, k = 0; c < 4; ++c)
                if (c != skip) f[size_t(k++)] = T(t, c);
            std::sort(f.begin(), f.end());
            sides[f].push_back(member(t, column - tags.labels.begin()) != 0);
        }
    }
    std::vector<std::vector<Vector3d>> rows;
    for (const auto& [f, in] : sides) {
        if (in.size() != 2 || in[0] == in[1]) continue;
        rows.emplace_back();
        for (const int64_t v : f)
            rows.back().emplace_back(mesh.V(v, 0), mesh.V(v, 1), mesh.V(v, 2));
    }
    return rows;
}
} // namespace
TEST_CASE(
    "The offset holds tetwild's envelope and hands it on with its offset surface, which simwild "
    "holds",
    "[component_io][io][envelope][tagged]" + release_only)
{
    // tetwild -> topological_offset -> simwild, each given the envelope the one before returned.
    // The offset grows the band `off` around a's surface held to tetwild's reference at tetwild's
    // eps, and returns that reference with the offset surface added under `off` -- exactly the
    // faces between the band and the rest, the offset surface being its own reference -- on both
    // routes. simwild then holds the offset's mesh to it: a and b to tetwild's input, the offset
    // surface to itself.
    WorkingDirectory cwd;
    const auto meshed = two_tets_meshed({{"input_edges", nlohmann::json::array()}});
    const MeshData mesh_in = tags_only(meshed.first.at("mesh"));
    const MeshData& envelope = meshed.first.at("envelope");
    const double eps = meshed.second.at("eps");
    auto options = offset_of_a;
    options["envelope_size"] = eps * (1 + 1e-9);
    const auto offset =
        topological_offset::topological_offset(options, {{"m", mesh_in}, {"e", envelope}});
    check(offset);
    REQUIRE(offset.first.count("envelope"));
    const MeshData& handed = offset.first.at("envelope");
    CHECK(
        handed.cell(CellType::Triangle).attributes.at("tags").labels ==
        std::vector<std::string>{"a", "b", "off"});
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(handed, CellType::Triangle, name)) ==
            canonical(labelled_rows(envelope, CellType::Triangle, name)));
    }
    const MeshData& mesh = offset.first.at("mesh");
    const auto front = region_boundary(mesh, "off");
    REQUIRE(!front.empty());
    CHECK(unoriented(labelled_rows(handed, CellType::Triangle, "off")) == unoriented(front));

    // The file route writes it as <output>_envelope.msh.
    io::write_mesh("m.msh", mesh_in);
    io::write_mesh("e.msh", envelope);
    auto file_options = options;
    file_options["application"] = "topological_offset";
    file_options["input"] = here("m.msh");
    file_options["input_dir"] = std::filesystem::current_path().string();
    file_options["envelope"] = "e.msh";
    file_options["output"] = "o";
    topological_offset::topological_offset(file_options);
    const MeshData file_envelope = io::read_mesh("o_envelope.msh");
    for (const std::string name : {"a", "b", "off"}) {
        CHECK(
            unoriented(labelled_rows(file_envelope, CellType::Triangle, name)) ==
            unoriented(labelled_rows(handed, CellType::Triangle, name)));
    }

    const auto remeshed = simwild::simwild(
        {{"input", "m"},
         {"envelope", "e"},
         {"eps", eps},
         {"max_iterations", 2},
         {"num_threads", 1}},
        {{"m", tags_only(mesh)}, {"e", handed}});
    for (const auto& [name, data] : remeshed.first) REQUIRE_NOTHROW(data.validate());
    CHECK(same_data(remeshed.first.at("envelope"), handed));
}
TEST_CASE(
    "The offset of a region hands on its band's whole boundary, which simwild holds",
    "[component_io][envelope][tagged]" + release_only)
{
    // simwild -> topological_offset -> simwild. simwild meshes a and b as regions, so the band
    // `off` grown around a is bounded by the offset surface and by a's surface, and simwild labels
    // a face between a and off with both. The offset hands on the band's whole boundary under
    // `off`, as every other label's envelope covers its region's boundary; with the offset surface
    // alone, simwild found the faces between a and off outside off's envelope (650, 2026-11-05).
    WorkingDirectory cwd;
    const auto meshed = simwild::simwild(
        {{"input", {"a", "b"}},
         {"input_names", {"a", "b"}},
         {"max_iterations", 0},
         {"num_threads", 1}},
        {{"a", tet_surface(0)}, {"b", tet_surface(3)}});
    check(meshed);
    const double eps = meshed.second.at("eps");
    auto options = offset_of_a;
    options["envelope_size"] = eps * (1 + 1e-9);
    const auto offset = topological_offset::topological_offset(
        options,
        {{"m", tags_only(meshed.first.at("mesh"))}, {"e", meshed.first.at("envelope")}});
    check(offset);
    const MeshData& mesh = offset.first.at("mesh");
    const MeshData& handed = offset.first.at("envelope");
    CHECK(
        unoriented(labelled_rows(handed, CellType::Triangle, "off")) ==
        unoriented(region_boundary(mesh, "off")));
    const auto remeshed = simwild::simwild(
        {{"input", "m"},
         {"envelope", "e"},
         {"eps", eps},
         {"max_iterations", 2},
         {"num_threads", 1}},
        {{"m", tags_only(mesh)}, {"e", handed}});
    for (const auto& [name, data] : remeshed.first) REQUIRE_NOTHROW(data.validate());
}
#endif
#if defined(WMTK_TEST_tetwild) && defined(WMTK_TEST_simwild)
TEST_CASE(
    "TetWild holds a tagged mesh to the envelope it is given, on both routes",
    "[component_io][io][envelope][tagged]" + release_only)
{
    // Two spheres `a` and `b`, and a segment inside `a` as a feature file, meshed by tetwild,
    // which simplifies them within half its eps. tetwild's tagged route continues from its mesh
    // held to the envelope it returned, the spheres as given, at the same eps: each face within
    // that eps of its sphere -- of the original input, not of the mesh received -- the envelope
    // handed on unchanged, both routes alike. The faces received lie outside a tenth of that eps,
    // and an envelope without a label of the mesh is refused, naming it: the reference checks
    // are the ones simwild runs, by the same code (TetOptimizerMesh).
    WorkingDirectory cwd;
    const MeshData a = icosphere(3, 1, Vector3d::Zero());
    const MeshData b = icosphere(3, 0.5, Vector3d(2.5, 0, 0));
    MeshData curve;
    curve.V.resize(2, 3);
    curve.V << -0.5, 0.1, 0.1, 0.5, 0.2, 0.1;
    curve.cells[CellType::Edge].connectivity.resize(1, 2);
    curve.cells[CellType::Edge].connectivity << 0, 1;
    const nlohmann::json first_options = {
        {"input_names", {"a", "b"}},
        {"eps_rel", 1e-2},
        {"eps_simplify_rel", 5e-3},
        {"max_iterations", 2},
        {"num_threads", 1}};
    auto memory_first = first_options;
    memory_first["input"] = {"a", "b"};
    memory_first["input_edges"] = {"curve"};
    const auto meshed = tetwild::tetwild(memory_first, {{"a", a}, {"b", b}, {"curve", curve}});
    write_obj("a.obj", a);
    write_obj("b.obj", b);
    write_obj("curve.obj", curve);
    auto file_first = first_options;
    file_first["application"] = "tetwild";
    file_first["input"] = {here("a.obj"), here("b.obj")};
    file_first["input_edges"] = {here("curve.obj")};
    file_first["output"] = "t";
    file_first["write_vtu"] = false;
    tetwild::tetwild(file_first);
    const double eps = meshed.second.at("eps");

    // The tagged route's eps_rel is of the mesh's own box, the first run's padded one.
    const MeshData& mesh_in = meshed.first.at("mesh");
    const double diag = (mesh_in.V.colwise().maxCoeff() - mesh_in.V.colwise().minCoeff()).norm();
    const MeshData& envelope = meshed.first.at("envelope");
    const nlohmann::json options = {
        {"eps_rel", eps * (1 + 1e-9) / diag},
        {"max_iterations", 2},
        {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    memory_options["envelope"] = "e";
    const MeshMap inputs{{"m", mesh_in}, {"e", envelope}};
    const auto result = tetwild::tetwild(memory_options, inputs);
    for (const auto& [name, data] : result.first) REQUIRE_NOTHROW(data.validate());
    auto file_options = options;
    file_options["application"] = "tetwild";
    file_options["input"] = {here("t_final.msh")};
    file_options["input_dir"] = std::filesystem::current_path().string();
    file_options["envelope"] = "t_envelope.msh";
    file_options["output"] = "u";
    file_options["write_vtu"] = false;
    tetwild::tetwild(file_options);
    const MeshData file = io::read_mesh("u_final.msh");

    REQUIRE(result.first.count("envelope"));
    CHECK(same_data(result.first.at("envelope"), envelope));
    CHECK(same_data(io::read_mesh("u_envelope.msh"), io::read_mesh("t_envelope.msh")));
    for (const MeshData* mesh : {&result.first.at("mesh"), &file}) {
        const std::array<std::pair<std::string, const MeshData*>, 2> spheres = {
            {{"a", &a}, {"b", &b}}};
        for (const auto& [name, sphere] : spheres) {
            std::vector<Vector3d> V;
            std::vector<Vector3i> F;
            for (Eigen::Index v = 0; v < sphere->V.rows(); ++v)
                V.push_back(sphere->V.row(v).transpose());
            const auto& C = sphere->cell(CellType::Triangle).connectivity;
            for (Eigen::Index f = 0; f < C.rows(); ++f)
                F.emplace_back(int(C(f, 0)), int(C(f, 1)), int(C(f, 2)));
            Envelope exact(true);
            exact.init(V, F, eps * (1 + 1e-8));
            const auto rows = labelled_rows(*mesh, CellType::Triangle, name);
            REQUIRE(!rows.empty());
            size_t outside = 0;
            for (const auto& f : rows)
                outside += exact.is_outside(std::array<Vector3d, 3>{{f[0], f[1], f[2]}});
            INFO(name << ": " << rows.size() << " faces");
            CHECK(outside == 0);
        }
        CHECK(!labelled_rows(*mesh, CellType::Edge, "feature_0").empty());
    }

    auto narrow = memory_options;
    narrow["eps_rel"] = eps / diag / 10;
    REQUIRE_THROWS_WITH(
        tetwild::tetwild(narrow, inputs),
        Catch::Matchers::ContainsSubstring("held faces outside") &&
            Catch::Matchers::ContainsSubstring("a ("));
    MeshData renamed = envelope;
    renamed.cells.at(CellType::Triangle).attributes.at("tags").labels = {"a", "c"};
    REQUIRE_THROWS_WITH(
        tetwild::tetwild(memory_options, {{"m", mesh_in}, {"e", renamed}}),
        Catch::Matchers::ContainsSubstring("labelled b"));
    renamed = envelope;
    renamed.cells.at(CellType::Edge).attributes.at("tags").labels = {"feature_1"};
    REQUIRE_THROWS_WITH(
        tetwild::tetwild(memory_options, {{"m", mesh_in}, {"e", renamed}}),
        Catch::Matchers::ContainsSubstring("labelled feature_0"));
}
#endif
#ifdef WMTK_TEST_triwild
namespace {
// A counter-clockwise square of side `side` with its lower left corner at (x, 0), as a curve.
MeshData square_curve(const double x, const double side = 1)
{
    MeshData data;
    data.V.resize(4, 3);
    data.V << 0, 0, 0, side, 0, 0, side, side, 0, 0, side, 0;
    data.V.col(0).array() += x;
    data.cells[CellType::Edge].connectivity.resize(4, 2);
    data.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 3, 3, 0;
    return data;
}
// triwild on two unit squares 3 apart, `a` and `b`, without an optimization and without the
// inputs' winding numbers, which no filter here needs: a triangle mesh of the box around them with
// tracked curves and, as every triwild output, no regions.
ComponentResult two_squares_meshed(const nlohmann::json& extra = nlohmann::json::object())
{
    nlohmann::json options = {
        {"input", {"a", "b"}},
        {"input_names", {"a", "b"}},
        {"simplify", false},
        {"skip_winding_number", true},
        {"max_iterations", 0},
        {"num_threads", 1}};
    options.update(extra);
    return triwild::triwild(options, {{"a", square_curve(0)}, {"b", square_curve(3)}});
}
// The area of `mesh`'s triangles, and the largest x of their centroids.
std::pair<double, double> area_and_max_x(const MeshData& mesh)
{
    const auto& F = mesh.cell(CellType::Triangle).connectivity;
    double area = 0, max_x = -std::numeric_limits<double>::infinity();
    for (Eigen::Index t = 0; t < F.rows(); ++t) {
        std::array<Vector3d, 3> p;
        for (int k = 0; k < 3; ++k) p[k] = mesh.V.row(F(t, k)).transpose();
        area += std::abs((p[1] - p[0]).cross(p[2] - p[0]).z()) / 2;
        max_x = std::max(max_x, (p[0] + p[1] + p[2]).x() / 3);
    }
    return {area, max_x};
}
} // namespace
TEST_CASE(
    "TriWild continues from a tagged triangle mesh, its triangles kept, on both routes",
    "[component_io][io][tagged]")
{
    // triwild's own mesh with its fields dropped: triangles with edge tags `a` and `b`. The
    // tagged route reads it exactly -- the reference it hands on, built from what it received, is
    // the input's edge rows, with their labels and orientation -- and gives one result on both
    // routes, the labels kept, no regions, each edge on its own square. Not the mesh as it came
    // at max_iterations 0: mesh_improvement() runs its bare collapse and swap passes whatever the
    // count, as tetwild's does.
    WorkingDirectory cwd;
    const MeshData tagged = io::tags_only(two_squares_meshed().first.at("mesh"));
    const nlohmann::json options = {{"max_iterations", 0}, {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    const auto result = triwild::triwild(memory_options, {{"m", tagged}});
    check(result);
    auto optimizing = memory_options;
    optimizing["max_iterations"] = 3;
    const auto optimized = triwild::triwild(optimizing, {{"m", tagged}});
    check(optimized);

    io::write_mesh("m.msh", tagged);
    auto file_options = options;
    file_options["application"] = "triwild";
    file_options["input"] = {here("m.msh")};
    file_options["output"] = "t";
    file_options["write_vtu"] = false;
    triwild::triwild(file_options);
    const MeshData file = io::read_mesh("t.msh");

    const MeshData& memory = result.first.at("mesh");
    for (const MeshData* mesh : {&memory, &file}) {
        REQUIRE(
            mesh->cell(CellType::Edge).attributes.at("tags").labels ==
            std::vector<std::string>{"a", "b"});
        for (const std::string name : {"a", "b"}) {
            CHECK(labelled_rows(*mesh, CellType::Triangle, name).empty());
        }
    }
    CHECK(
        memory.cell(CellType::Triangle).connectivity.rows() ==
        file.cell(CellType::Triangle).connectivity.rows());
    for (const std::string name : {"a", "b"}) {
        CHECK(
            canonical(labelled_rows(memory, CellType::Edge, name)) ==
            canonical(labelled_rows(file, CellType::Edge, name)));
    }
    const MeshData file_envelope = io::read_mesh("t_envelope.msh");
    for (const MeshData* envelope : {&result.first.at("envelope"), &file_envelope}) {
        for (const std::string name : {"a", "b"}) {
            CHECK(
                canonical(labelled_rows(*envelope, CellType::Edge, name)) ==
                canonical(labelled_rows(tagged, CellType::Edge, name)));
        }
    }
    const MeshData& mesh = optimized.first.at("mesh");
    for (const std::string name : {"a", "b"}) {
        const auto rows = labelled_rows(mesh, CellType::Edge, name);
        CHECK(rows.size() >= 4);
        for (const auto& e : rows)
            for (const Vector3d& x : e) CHECK(std::abs(x.x() - (name == "a" ? 0.5 : 3.5)) < 1);
    }
}
TEST_CASE(
    "TriWild refuses a tagged triangle mesh with regions, and takes its own output back in",
    "[component_io][tagged]")
{
    // A region tag of the mesh: refused, pointing to simwild. triwild writes none -- its inputs'
    // winding numbers are fields, wn_<name> (wn_<i> in the .msh), which the tagged route ignores
    // -- so its output goes back in as it is, fields and all, on both routes.
    WorkingDirectory cwd;
    auto region = triangle();
    add_tags(region, CellType::Triangle);
    REQUIRE_THROWS_WITH(
        triwild::triwild({{"input", "m"}, {"num_threads", 1}}, {{"m", region}}),
        Catch::Matchers::ContainsSubstring("tag_0") &&
            Catch::Matchers::ContainsSubstring("simwild"));
    const auto with_winding = two_squares_meshed({{"skip_winding_number", false}});
    REQUIRE(with_winding.first.at("mesh").cell(CellType::Triangle).attributes.count("wn_b"));
    MeshData regions = with_winding.first.at("mesh");
    auto& triangles = regions.cells.at(CellType::Triangle);
    triangles.attributes["tags"] = {
        DataMatrix<uint8_t>::Ones(triangles.connectivity.rows(), 1).eval(),
        {"r"}};
    REQUIRE_THROWS_WITH(
        triwild::triwild({{"input", "m"}, {"num_threads", 1}}, {{"m", regions}}),
        Catch::Matchers::ContainsSubstring("r") && Catch::Matchers::ContainsSubstring("simwild"));
    const auto again = triwild::triwild(
        {{"input", "m"}, {"max_iterations", 2}, {"num_threads", 1}},
        {{"m", with_winding.first.at("mesh")}});
    check(again);
    write_obj("a.obj", square_curve(0));
    write_obj("b.obj", square_curve(3));
    triwild::triwild(
        {{"application", "triwild"},
         {"input", {here("a.obj"), here("b.obj")}},
         {"input_names", {"a", "b"}},
         {"simplify", false},
         {"skip_winding_number", false},
         {"max_iterations", 0},
         {"num_threads", 1},
         {"output", "t1"},
         {"write_vtu", false}});
    REQUIRE(io::read_mesh("t1.msh").cell(CellType::Triangle).attributes.count("wn_1"));
    triwild::triwild(
        {{"application", "triwild"},
         {"input", {here("t1.msh")}},
         {"max_iterations", 2},
         {"num_threads", 1},
         {"output", "t2"},
         {"write_vtu", false}});
    for (const MeshData* mesh : {&again.first.at("mesh")}) {
        REQUIRE(
            mesh->cell(CellType::Edge).attributes.at("tags").labels ==
            std::vector<std::string>{"a", "b"});
    }
    CHECK(
        io::read_mesh("t2.msh").cell(CellType::Edge).attributes.at("tags").labels ==
        std::vector<std::string>{"a", "b"});
}
TEST_CASE(
    "TriWild writes each input's winding number as a triangle field, as tetwild does",
    "[component_io][io][tagged]")
{
    // Two unit squares 3 apart, a and b, filter none. Each triangle carries each input's winding
    // number at its centroid, 1 inside that square and 0 outside -- fields, not tags, so no
    // regions: wn_<name> on the memory route, wn_<i> in the .msh, as tetwild's on its tets. The
    // winding number of a closed polygon off its boundary is exactly 0 or 1, so the tolerance only
    // absorbs rounding. skip_winding_number leaves them out under filter none and is ignored
    // under any other; the tagged route computes none.
    WorkingDirectory cwd;
    const auto check_fields = [](const MeshData& mesh, const std::array<std::string, 2>& names) {
        const auto& triangles = mesh.cell(CellType::Triangle);
        for (int k = 0; k < 2; ++k) {
            INFO(names[k]);
            REQUIRE(triangles.attributes.count(names[k]));
            const DataMatrix<double> w = triangles.attributes.at(names[k]).doubles();
            REQUIRE(w.rows() == triangles.connectivity.rows());
            const double x0 = k == 0 ? 0 : 3;
            size_t n_inside = 0;
            for (Eigen::Index t = 0; t < w.rows(); ++t) {
                Vector3d c = Vector3d::Zero();
                for (int j = 0; j < 3; ++j) {
                    c += mesh.V.row(triangles.connectivity(t, j)).transpose();
                }
                c /= 3;
                const bool inside = c.x() > x0 && c.x() < x0 + 1 && c.y() > 0 && c.y() < 1;
                CHECK(std::abs(w(t, 0) - (inside ? 1 : 0)) < 1e-6);
                n_inside += inside;
            }
            CHECK(n_inside > 0);
        }
    };
    const auto has_wn = [](const MeshData& mesh) {
        for (const auto& entry : mesh.cell(CellType::Triangle).attributes)
            if (entry.first.rfind("wn_", 0) == 0) return true;
        return false;
    };

    const auto memory = two_squares_meshed({{"skip_winding_number", false}});
    check_fields(memory.first.at("mesh"), {{"wn_a", "wn_b"}});
    for (const std::string name : {"a", "b"}) {
        CHECK(labelled_rows(memory.first.at("mesh"), CellType::Triangle, name).empty());
    }

    write_obj("a.obj", square_curve(0));
    write_obj("b.obj", square_curve(3));
    const nlohmann::json file_options = {
        {"application", "triwild"},
        {"input", {here("a.obj"), here("b.obj")}},
        {"simplify", false},
        {"max_iterations", 0},
        {"num_threads", 1},
        {"write_vtu", false}};
    auto named = file_options;
    named["input_names"] = {"a", "b"};
    named["output"] = "named";
    triwild::triwild(named);
    const MeshData named_file = io::read_mesh("named.msh");
    check_fields(named_file, {{"wn_0", "wn_1"}});
    CHECK(!named_file.cell(CellType::Triangle).attributes.count("wn_a"));
    auto unnamed = file_options;
    unnamed["output"] = "unnamed";
    triwild::triwild(unnamed);
    check_fields(io::read_mesh("unnamed.msh"), {{"wn_0", "wn_1"}});

    CHECK(!has_wn(two_squares_meshed({{"skip_winding_number", true}}).first.at("mesh")));
    check_fields(
        two_squares_meshed({{"skip_winding_number", true}, {"filter", "input"}}).first.at("mesh"),
        {{"wn_a", "wn_b"}});

    const auto again = triwild::triwild(
        {{"input", "m"}, {"max_iterations", 0}, {"num_threads", 1}},
        {{"m", memory.first.at("mesh")}});
    CHECK(!has_wn(again.first.at("mesh")));
}
TEST_CASE("TriWild refuses what its tagged route has no input for", "[component_io][tagged]")
{
    WorkingDirectory cwd;
    const MeshData tagged = io::tags_only(two_squares_meshed().first.at("mesh"));
    const MeshMap meshes{{"m", tagged}, {"s", square_curve(5)}};
    const auto run = [&](nlohmann::json options) {
        if (!options.contains("input")) options["input"] = "m";
        options["num_threads"] = 1;
        options["max_iterations"] = 0;
        return triwild::triwild(options, meshes);
    };
    using Catch::Matchers::ContainsSubstring;
    REQUIRE_THROWS_WITH(run({{"input", {"m", "s"}}}), ContainsSubstring("only input"));
    REQUIRE_THROWS_WITH(run({{"filter", "input"}}), ContainsSubstring("filter \"input\""));
    REQUIRE_THROWS_WITH(run({{"input_points", {"s"}}}), ContainsSubstring("input_points"));
    REQUIRE_THROWS_WITH(
        run({{"filter", "flood"}, {"filter_inputs", {"c"}}}),
        ContainsSubstring("c is not an input"));
    REQUIRE_THROWS_WITH(
        run({{"input", "s"}, {"envelope", "m"}}),
        ContainsSubstring("tagged mesh input only"));
}
TEST_CASE(
    "TriWild filter_inputs: only the inputs named bound the domain kept",
    "[component_io][tagged]")
{
    // Two unit squares 3 apart, `a` and `b`: every filter keeps both by default, and with
    // filter_inputs a only a, its area 1. On the tagged route too, by a's edge-tag label.
    WorkingDirectory cwd;
    const std::string filter = GENERATE("input", "tracked", "flood");
    INFO("filter " << filter);
    const nlohmann::json winding = {{"skip_winding_number", false}, {"filter", filter}};
    const auto both = area_and_max_x(two_squares_meshed(winding).first.at("mesh"));
    CHECK(std::abs(both.first - 2) < 1e-9);
    CHECK(both.second > 2);
    auto only = winding;
    only["filter_inputs"] = {"a"};
    const auto only_a = area_and_max_x(two_squares_meshed(only).first.at("mesh"));
    CHECK(std::abs(only_a.first - 1) < 1e-9);
    CHECK(only_a.second < 1);
    if (filter == "input") return;
    const auto tagged_a = area_and_max_x(
        triwild::triwild(
            {{"input", "m"},
             {"filter", filter},
             {"filter_inputs", {"a"}},
             {"max_iterations", 0},
             {"num_threads", 1}},
            {{"m", two_squares_meshed().first.at("mesh")}})
            .first.at("mesh"));
    CHECK(std::abs(tagged_a.first - 1) < 1e-9);
    CHECK(tagged_a.second < 1);
}
TEST_CASE(
    "TriWild holds a tagged mesh to the envelope it is given, on both routes",
    "[component_io][io][envelope][tagged]")
{
    // Two squares a and b, simplified by triwild within half its eps (a square has nothing to
    // simplify, so a circle-like polygon stands for each): the tagged route continues held to the
    // envelope triwild returned, at its eps, and hands it on unchanged; at a tenth of that eps the
    // edges received lie outside, and an envelope without a label of the mesh is refused.
    WorkingDirectory cwd;
    const auto polygon = [](const double cx, const double r) {
        const int n = 64;
        MeshData data;
        data.V.resize(n, 3);
        data.cells[CellType::Edge].connectivity.resize(n, 2);
        for (int i = 0; i < n; ++i) {
            const double t = 2 * std::acos(-1.0) * i / n;
            data.V.row(i) << cx + r * std::cos(t), r * std::sin(t), 0;
            data.cells[CellType::Edge].connectivity.row(i) << i, (i + 1) % n;
        }
        return data;
    };
    const MeshMap curves{{"a", polygon(0, 1)}, {"b", polygon(3, 0.5)}};
    const nlohmann::json first = {
        {"input", {"a", "b"}},
        {"input_names", {"a", "b"}},
        {"eps_rel", 1e-2},
        {"eps_simplify_rel", 5e-3},
        {"skip_winding_number", true},
        {"max_iterations", 2},
        {"num_threads", 1}};
    const auto meshed = triwild::triwild(first, curves);
    const double eps = meshed.second.at("eps");
    const MeshData& mesh_in = meshed.first.at("mesh");
    const MeshData& envelope = meshed.first.at("envelope");
    const double diag = (mesh_in.V.colwise().maxCoeff() - mesh_in.V.colwise().minCoeff()).norm();
    const nlohmann::json options = {
        {"input", "m"},
        {"envelope", "e"},
        {"eps_rel", eps * (1 + 1e-9) / diag},
        {"max_iterations", 2},
        {"num_threads", 1}};
    const MeshMap inputs{{"m", mesh_in}, {"e", envelope}};
    const auto result = triwild::triwild(options, inputs);
    for (const auto& [name, data] : result.first) REQUIRE_NOTHROW(data.validate());
    REQUIRE(result.first.count("envelope"));
    {
        // The envelope handed on unchanged, on the file route too.
        const MeshData& out = result.first.at("envelope");
        CHECK(out.V == envelope.V);
        CHECK(out.cell(CellType::Edge).connectivity == envelope.cell(CellType::Edge).connectivity);
    }
    io::write_mesh("m.msh", io::tags_only(mesh_in));
    io::write_mesh("e.msh", envelope);
    triwild::triwild(
        {{"application", "triwild"},
         {"input", {here("m.msh")}},
         {"input_dir", std::filesystem::current_path().string()},
         {"envelope", "e.msh"},
         {"eps_rel", eps * (1 + 1e-9) / diag},
         {"max_iterations", 2},
         {"num_threads", 1},
         {"output", "u"},
         {"write_vtu", false}});
    CHECK(
        canonical(labelled_rows(io::read_mesh("u_envelope.msh"), CellType::Edge, "a")) ==
        canonical(labelled_rows(envelope, CellType::Edge, "a")));
    for (const std::string name : {"a", "b"}) {
        const double r = name == "a" ? 1 : 0.5, cx = name == "a" ? 0 : 3;
        for (const auto& e : labelled_rows(result.first.at("mesh"), CellType::Edge, name)) {
            // Within eps of the polygon, which is within its sagitta of the circle.
            for (const Vector3d& x : e) {
                CHECK(std::abs((x - Vector3d(cx, 0, 0)).norm() - r) <= eps * (1 + 1e-9) + 0.002);
            }
        }
    }
    auto narrow = options;
    narrow["eps_rel"] = eps / diag / 10;
    REQUIRE_THROWS_WITH(
        triwild::triwild(narrow, inputs),
        Catch::Matchers::ContainsSubstring("held edges outside"));
    MeshData renamed = envelope;
    renamed.cells.at(CellType::Edge).attributes.at("tags").labels = {"a", "c"};
    REQUIRE_THROWS_WITH(
        triwild::triwild(options, {{"m", mesh_in}, {"e", renamed}}),
        Catch::Matchers::ContainsSubstring("labelled b"));
}
#endif
#ifdef WMTK_TEST_tetwild
namespace {
// The boundary of the box [lo, hi], its triangles facing out.
MeshData box_surface(const Vector3d& lo, const Vector3d& hi)
{
    MeshData data;
    data.V.resize(8, 3);
    for (int i = 0; i < 8; ++i) {
        const bool x = i == 1 || i == 2 || i == 5 || i == 6;
        const bool y = i == 2 || i == 3 || i == 6 || i == 7;
        data.V.row(i) << (x ? hi : lo).x(), (y ? hi : lo).y(), (i >= 4 ? hi : lo).z();
    }
    auto& F = data.cells[CellType::Triangle].connectivity;
    F.resize(12, 3);
    F << 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5;
    return data;
}
} // namespace
TEST_CASE(
    "TetWild filter_inputs: tracked refuses the faces a named input shares, flood keeps it",
    "[component_io][tagged]" + release_only)
{
    // Two unit boxes touching face to face, filter_inputs a: on the faces between them the
    // orientation counts both boxes (one count per face), so "tracked" cannot take a's winding
    // number and throws, naming flood; "flood" keeps a alone, its volume 1. On both input routes.
    WorkingDirectory cwd;
    using Catch::Matchers::ContainsSubstring;
    const MeshMap boxes{
        {"a", box_surface(Vector3d(0, 0, 0), Vector3d(1, 1, 1))},
        {"b", box_surface(Vector3d(1, 0, 0), Vector3d(2, 1, 1))}};
    nlohmann::json options = {
        {"input", {"a", "b"}},
        {"input_names", {"a", "b"}},
        {"simplify", false},
        {"max_iterations", 0},
        {"num_threads", 1},
        {"filter_inputs", {"a"}}};
    options["filter"] = "tracked";
    REQUIRE_THROWS_WITH(
        tetwild::tetwild(options, boxes),
        ContainsSubstring("per face") && ContainsSubstring("flood"));
    options["filter"] = "flood";
    const auto a = volume_and_max_x(tetwild::tetwild(options, boxes).first.at("mesh"));
    CHECK(std::abs(a.first - 1) < 1e-9);
    CHECK(a.second < 1);

    auto unfiltered = options;
    unfiltered.erase("filter_inputs");
    unfiltered["filter"] = "none";
    const MeshMap tagged{{"m", tetwild::tetwild(unfiltered, boxes).first.at("mesh")}};
    nlohmann::json again = {
        {"input", "m"},
        {"filter", "tracked"},
        {"filter_inputs", {"a"}},
        {"max_iterations", 0},
        {"num_threads", 1}};
    REQUIRE_THROWS_WITH(tetwild::tetwild(again, tagged), ContainsSubstring("per face"));
    again["filter"] = "flood";
    const auto tagged_a = volume_and_max_x(tetwild::tetwild(again, tagged).first.at("mesh"));
    CHECK(std::abs(tagged_a.first - 1) < 1e-9);
    CHECK(tagged_a.second < 1);
}
#endif
#ifdef WMTK_TEST_triwild
TEST_CASE(
    "TriWild filter_inputs: tracked refuses the edges a named input shares, flood keeps it",
    "[component_io][tagged]")
{
    // Two unit squares touching edge to edge, filter_inputs a: the 2D twin of tetwild's.
    WorkingDirectory cwd;
    using Catch::Matchers::ContainsSubstring;
    const MeshMap squares{{"a", square_curve(0)}, {"b", square_curve(1)}};
    nlohmann::json options = {
        {"input", {"a", "b"}},
        {"input_names", {"a", "b"}},
        {"simplify", false},
        {"max_iterations", 0},
        {"num_threads", 1},
        {"filter_inputs", {"a"}}};
    options["filter"] = "tracked";
    REQUIRE_THROWS_WITH(
        triwild::triwild(options, squares),
        ContainsSubstring("per edge") && ContainsSubstring("flood"));
    options["filter"] = "flood";
    const auto a = area_and_max_x(triwild::triwild(options, squares).first.at("mesh"));
    CHECK(std::abs(a.first - 1) < 1e-9);
    CHECK(a.second < 1);
}
TEST_CASE("TriWild refuses a filtered mesh, its tracked boundary", "[component_io][tagged]")
{
    // Two unit squares filtered to their insides: the mesh's boundary is its tracked edges.
    // Refused: the tagged route takes a mesh that fills its box, its tracked edges inside.
    WorkingDirectory cwd;
    const auto filtered =
        two_squares_meshed({{"skip_winding_number", false}, {"filter", "tracked"}});
    REQUIRE_THROWS_WITH(
        triwild::triwild(
            {{"input", "m"}, {"max_iterations", 3}, {"num_threads", 1}},
            {{"m", filtered.first.at("mesh")}}),
        Catch::Matchers::ContainsSubstring("tracked edges of the tagged mesh lie on its boundary"));
}
#endif

// Data and annotations (io::ComponentIO::data_fields): a reader reads the tags and the data
// fields it consumes and drops every other field, on both routes, so that a result goes into the
// next component as it is. Until 2026-11 each reader refused every field it did not list, and a
// new field broke a chain until every list had it: triwild's wn_<name> broke triwild -> simwild.
namespace {
// `mesh` with a field no component writes or reads, `note`, on its vertices and its `top` cells.
[[maybe_unused]] MeshData annotated(MeshData mesh, const CellType top)
{
    mesh.vertex_attributes["note"].values = DataMatrix<double>::Zero(mesh.V.rows(), 1).eval();
    auto& cells = mesh.cells.at(top);
    cells.attributes["note"].values = DataMatrix<double>::Ones(cells.connectivity.rows(), 1).eval();
    return mesh;
}
// Whether `a` and `b` hold the same geometry and tags, exactly: the positions, every block's
// connectivity, and its tags with their labels.
[[maybe_unused]] bool same_geometry_and_tags(const MeshData& a, const MeshData& b)
{
    const auto same = [](const auto& x, const auto& y) {
        return x.rows() == y.rows() && x.cols() == y.cols() && x == y;
    };
    if (!same(a.V, b.V) || a.cells.size() != b.cells.size()) return false;
    for (const auto& [type, ca] : a.cells) {
        const auto cb = b.cells.find(type);
        if (cb == b.cells.end() || !same(ca.connectivity, cb->second.connectivity)) return false;
        const auto ta = ca.attributes.find("tags");
        const auto tb = cb->second.attributes.find("tags");
        if ((ta == ca.attributes.end()) != (tb == cb->second.attributes.end())) return false;
        if (ta != ca.attributes.end() && (ta->second.labels != tb->second.labels ||
                                          !same(ta->second.integers(), tb->second.integers())))
            return false;
    }
    return true;
}
// Whether a field `note` is anywhere in `meshes`.
[[maybe_unused]] bool carries_note(const MeshMap& meshes)
{
    for (const auto& [key, mesh] : meshes) {
        if (mesh.vertex_attributes.count("note")) return true;
        for (const auto& [type, cells] : mesh.cells)
            if (cells.attributes.count("note")) return true;
    }
    return false;
}
[[maybe_unused]] std::set<std::string> field_names(const MeshAttributes& attributes)
{
    std::set<std::string> names;
    for (const auto& [name, field] : attributes) names.insert(name);
    return names;
}
} // namespace
#ifdef WMTK_TEST_tetwild
TEST_CASE(
    "TetWild's tagged route drops a field it does not read, on both routes",
    "[component_io][io][tagged]" + release_only)
{
    WorkingDirectory cwd;
    const MeshData tagged = tags_only(two_tets_meshed().first.at("mesh"));
    const MeshData noted = annotated(tagged, CellType::Tetrahedron);
    const nlohmann::json options = {{"max_iterations", 0}, {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    const auto bare = tetwild::tetwild(memory_options, {{"m", tagged}});
    const auto with_note = tetwild::tetwild(memory_options, {{"m", noted}});
    check(with_note);
    CHECK(!carries_note(with_note.first));
    CHECK(same_geometry_and_tags(with_note.first.at("mesh"), bare.first.at("mesh")));

    io::write_mesh("bare.msh", tagged);
    io::write_mesh("noted.msh", noted);
    for (const std::string name : {"bare", "noted"}) {
        auto file_options = options;
        file_options["application"] = "tetwild";
        file_options["input"] = {here(name + ".msh")};
        file_options["output"] = name + "_out";
        file_options["write_vtu"] = false;
        tetwild::tetwild(file_options);
    }
    CHECK(same_geometry_and_tags(
        io::read_mesh("noted_out_final.msh"),
        io::read_mesh("bare_out_final.msh")));
}
#endif
#ifdef WMTK_TEST_triwild
TEST_CASE(
    "TriWild's tagged route drops a field it does not read, on both routes",
    "[component_io][io][tagged]")
{
    WorkingDirectory cwd;
    const MeshData tagged = io::tags_only(two_squares_meshed().first.at("mesh"));
    const MeshData noted = annotated(tagged, CellType::Triangle);
    const nlohmann::json options = {{"max_iterations", 0}, {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    const auto bare = triwild::triwild(memory_options, {{"m", tagged}});
    const auto with_note = triwild::triwild(memory_options, {{"m", noted}});
    check(with_note);
    CHECK(!carries_note(with_note.first));
    CHECK(same_geometry_and_tags(with_note.first.at("mesh"), bare.first.at("mesh")));

    io::write_mesh("bare.msh", tagged);
    io::write_mesh("noted.msh", noted);
    for (const std::string name : {"bare", "noted"}) {
        auto file_options = options;
        file_options["application"] = "triwild";
        file_options["input"] = {here(name + ".msh")};
        file_options["output"] = name + "_out";
        file_options["write_vtu"] = false;
        triwild::triwild(file_options);
    }
    CHECK(same_geometry_and_tags(io::read_mesh("noted_out.msh"), io::read_mesh("bare_out.msh")));
}
#endif
#ifdef WMTK_TEST_simwild
TEST_CASE("SimWild drops a field it does not read, on both routes", "[component_io][io][tagged]")
{
    // The meshes of "SimWild keeps a tagged mesh's curves on both routes": in 2D four triangles,
    // in 3D the unit cube's six tets, the first cell tagged, an edge tagged.
    WorkingDirectory cwd;
    const bool is_3d = GENERATE(false, true);
    INFO((is_3d ? "3D" : "2D"));
    MeshData data;
    if (is_3d) {
        data = cube_tets();
    } else {
        MatrixXd V(6, 3);
        V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, -1, 0, 1, -1, 0;
        MatrixXi F(4, 3);
        F << 0, 1, 2, 1, 3, 2, 0, 4, 1, 1, 4, 5;
        data = io::geometry_data(V, F, CellType::Triangle);
    }
    const CellType top = is_3d ? CellType::Tetrahedron : CellType::Triangle;
    auto& cells = data.cells.at(top);
    DataMatrix<uint8_t> first = DataMatrix<uint8_t>::Zero(cells.connectivity.rows(), 1);
    first(0, 0) = 1;
    cells.attributes["tags"] = {first, {"tag_0"}};
    auto& edges = data.cells[CellType::Edge];
    edges.connectivity.resize(1, 2);
    if (is_3d) {
        edges.connectivity << 7, 0;
    } else {
        edges.connectivity << 1, 0;
    }
    edges.attributes["tags"] = {
        DataMatrix<uint8_t>::Ones(1, 1).eval(),
        {is_3d ? "feature_0" : "curve"}};
    const MeshData noted = annotated(data, top);
    const nlohmann::json options = {{"max_iterations", 0}, {"simplify", false}, {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    const auto bare = simwild::simwild(memory_options, {{"m", data}});
    const auto with_note = simwild::simwild(memory_options, {{"m", noted}});
    check(with_note);
    CHECK(!carries_note(with_note.first));
    CHECK(same_geometry_and_tags(with_note.first.at("mesh"), bare.first.at("mesh")));

    // The file route reads a .msh as the memory route reads the mesh (read_image_msh()).
    io::write_mesh("bare.msh", data);
    io::write_mesh("noted.msh", noted);
    for (const std::string name : {"bare", "noted"}) {
        auto file_options = options;
        file_options["application"] = "simwild";
        file_options["input"] = {here(name + ".msh")};
        file_options["output"] = name + "_out";
        file_options["write_vtu"] = false;
        simwild::simwild(file_options);
    }
    CHECK(same_geometry_and_tags(io::read_mesh("noted_out.msh"), io::read_mesh("bare_out.msh")));
}
#endif
#if defined(WMTK_TEST_triwild) && defined(WMTK_TEST_simwild)
TEST_CASE(
    "TriWild's mesh goes into SimWild as it is, its annotations dropped",
    "[component_io][tagged]")
{
    // triwild -> simwild on the memory route, triwild's mesh with every field it writes (wn_a,
    // wn_b, quality, ...), which simwild refused until 2026-11 (wn_a). The result is simwild's on
    // the mesh with its tags only, exactly, with triwild's curves.
    WorkingDirectory cwd;
    const auto meshed = two_squares_meshed({{"skip_winding_number", false}});
    const MeshData& mesh = meshed.first.at("mesh");
    REQUIRE(mesh.cell(CellType::Triangle).attributes.count("wn_a"));
    const nlohmann::json options = {{"input", "m"}, {"max_iterations", 0}, {"num_threads", 1}};
    const auto as_is = simwild::simwild(options, {{"m", mesh}});
    check(as_is);
    const auto bare = simwild::simwild(options, {{"m", io::tags_only(mesh)}});
    CHECK(same_geometry_and_tags(as_is.first.at("mesh"), bare.first.at("mesh")));
    for (const std::string name : {"a", "b"})
        CHECK(!labelled_rows(as_is.first.at("mesh"), CellType::Edge, name).empty());
}
#endif
#if defined(WMTK_TEST_tetwild) && defined(WMTK_TEST_topological_offset)
TEST_CASE(
    "The offset's result carries tags, data fields and two annotations, its debug frames all",
    "[component_io][io][tagged]" + release_only)
{
    // The result -- the memory route's meshes, the .msh, the save_vtu file -- carries the tags,
    // the data fields components read (vid, labels, offset_tag) and amips and
    // distance_over_target; a DEBUG_output frame every diagnostic field too. The input as in
    // "TetWild refuses an offset's output, whose band is a volume".
    WorkingDirectory cwd;
    const MeshData tagged =
        tags_only(two_tets_meshed({{"input_edges", nlohmann::json::array()}}).first.at("mesh"));
    const nlohmann::json options = {
        {"offset_selection", "a"},
        {"target_distance", 0.05},
        {"offset_output_tags", {"off"}},
        {"optimize_offset", false},
        {"num_threads", 1}};
    auto memory_options = options;
    memory_options["input"] = "m";
    const auto result = topological_offset::topological_offset(memory_options, {{"m", tagged}});
    check(result);
    const std::set<std::string> vertex = {"vid", "labels", "distance_over_target"};
    const std::set<std::string> tet = {"tags", "amips", "offset_tag"};
    const MeshData& mesh = result.first.at("mesh");
    CHECK(field_names(mesh.vertex_attributes) == vertex);
    CHECK(field_names(mesh.cell(CellType::Tetrahedron).attributes) == tet);
    for (const auto& [key, data] : result.first) {
        INFO(key);
        for (const auto& name : field_names(data.vertex_attributes)) CHECK(vertex.count(name));
        for (const auto& [type, cells] : data.cells)
            for (const auto& name : field_names(cells.attributes)) CHECK(tet.count(name));
    }

    io::write_mesh("m.msh", tagged);
    auto file_options = options;
    file_options["application"] = "topological_offset";
    file_options["input"] = here("m.msh");
    file_options["output"] = "o";
    file_options["save_vtu"] = true;
    file_options["DEBUG_output"] = true;
    topological_offset::topological_offset(file_options);
    const MeshData file = io::read_mesh("o.msh");
    CHECK(field_names(file.vertex_attributes) == vertex);
    CHECK(field_names(file.cell(CellType::Tetrahedron).attributes) == tet);
    // The .vtu files by their DataArray names: the result's, and the first debug frame's.
    const auto arrays = [](const std::string& path) {
        REQUIRE(std::filesystem::exists(path));
        std::ifstream in(path);
        const std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        return [s](const std::string& name) {
            return s.find("Name=\"" + name + "\"") != std::string::npos;
        };
    };
    const auto vtu = arrays("o.vtu");
    const auto frame = arrays("o_00000.vtu");
    for (const std::string name :
         {"amips", "distance_over_target", "vid", "labels", "offset_tag"}) {
        INFO(name);
        CHECK(vtu(name));
        CHECK(frame(name));
    }
    for (const std::string name :
         {"tet_energy", "amips_term", "band_term", "plastic", "sizing_scalar", "input_distance"}) {
        INFO(name);
        CHECK(!vtu(name));
        CHECK(frame(name));
    }
    // The offset faces' file: its vertex fields (input_distance) and face fields (front_err_ratio)
    // the frame's only.
    const auto off = arrays("o_off.vtu");
    CHECK(off("distance_over_target"));
    CHECK(!off("input_distance"));
    CHECK(!off("front_err_ratio"));
    CHECK(arrays("o_00000_off.vtu")("input_distance"));
}
#endif
