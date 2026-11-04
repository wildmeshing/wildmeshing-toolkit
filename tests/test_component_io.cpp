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
#include <limits>
#include <wmtk/MeshData.hpp>
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
    WorkingDirectory cwd;
    MeshData data;
    SECTION("2D")
    {
        data = triangle();
        add_tags(data, CellType::Triangle);
    }
    SECTION("3D")
    {
        data = tet();
        add_tags(data, CellType::Tetrahedron);
    }
    MeshData envelope;
    envelope.V = data.V;
    if (data.cells.count(CellType::Tetrahedron)) {
        envelope.cells[CellType::Triangle].connectivity.resize(4, 3);
        envelope.cells[CellType::Triangle].connectivity << 1, 2, 3, 0, 3, 2, 0, 1, 3, 0, 2, 1;
    } else {
        envelope.cells[CellType::Edge].connectivity.resize(3, 2);
        envelope.cells[CellType::Edge].connectivity << 0, 1, 1, 2, 2, 0;
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
    // Two cells, the first tagged, so that the face (2D: edge) between them is an interface: in
    // 2D two triangles, the edge (1, 0) of the first tagged `curve`; in 3D two tets, the edge
    // (1, 0) of the first tagged `feature_0`. Through simwild, the curve comes back on its edge
    // block, under its label, held on its segment, the 2D curve in its direction.
    WorkingDirectory cwd;
    const bool is_3d = GENERATE(false, true);
    INFO((is_3d ? "3D" : "2D"));
    MeshData data;
    if (is_3d) {
        MatrixXd V(5, 3);
        V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1;
        MatrixXi T(2, 4);
        T << 0, 1, 2, 3, 0, 2, 1, 4;
        data = io::geometry_data(V, T, CellType::Tetrahedron);
    } else {
        MatrixXd V(4, 3);
        V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0;
        MatrixXi F(2, 3);
        F << 0, 1, 2, 1, 3, 2;
        data = io::geometry_data(V, F, CellType::Triangle);
    }
    auto& cells = data.cells.at(is_3d ? CellType::Tetrahedron : CellType::Triangle);
    cells.attributes["tags"] = {(DataMatrix<uint8_t>(2, 1) << 1, 0).finished(), {"tag_0"}};
    const std::string label = is_3d ? "feature_0" : "curve";
    auto& edges = data.cells[CellType::Edge];
    edges.connectivity.resize(1, 2);
    edges.connectivity << 1, 0;
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
            for (const Vector3d& p : row) CHECK(std::abs(p.y()) + std::abs(p.z()) < 1e-12);
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
            CHECK(!labelled_rows(*mesh, CellType::Triangle, name).empty());
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
        CHECK(std::count(input->tag_names.begin(), input->tag_names.end(), "a") == 1);
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
    WorkingDirectory cwd;
    auto data = tet();
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
    data.vertex_attributes["unrelated"].values = DataMatrix<double>::Zero(4, 1).eval();
    REQUIRE_THROWS(qslim::qslim({{"input", "source"}, {"num_threads", 1}}, {{"source", data}}));
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
