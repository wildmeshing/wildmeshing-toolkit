#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
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
    SECTION("an edge block in a tet mesh")
    {
        data.cells[CellType::Edge].connectivity.resize(1, 2);
        data.cells[CellType::Edge].connectivity << 0, 1;
        REQUIRE_THROWS(simwild::read_mesh_data(data));
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
             {"skip_simplify", true},
             {"max_iterations", 0},
             {"num_threads", 1},
             {"DEBUG_output", true},
             {"write_vtu", true}},
            {{"curves", data}}));
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
             {"skip_simplify", true},
             {"max_iterations", 0},
             {"num_threads", 1},
             {"DEBUG_output", true},
             {"write_vtu", true}},
            {{"surface", data}}));
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
         {"skip_simplify", true},
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
    const auto file = GENERATE(as<std::string>{}, "topo_annots_groups.msh", "double_sphere.msh");
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
