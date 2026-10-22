#include <catch2/catch_test_macros.hpp>
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
    return simwild::geometry_data(V, F, CellType::Triangle);
}
MeshData tet()
{
    MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    return simwild::geometry_data(V, T, CellType::Tetrahedron);
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
TEST_CASE("Tagged meshes use an embedded envelope block", "[component_io]")
{
    // A tagged tet plus an envelope triangle on two extra vertices, as read_mesh returns a
    // simwild .msh written with write_envelope.
    auto data = tet();
    add_tags(data, CellType::Tetrahedron);
    data.V.conservativeResize(6, 3);
    data.V.bottomRows(2) << 5, 5, 5, 6, 5, 5;
    auto& envelope = data.cells[CellType::Triangle];
    envelope.connectivity.resize(1, 3);
    envelope.connectivity << 4, 5, 0;
    envelope.attributes["tags"] = {DataMatrix<uint8_t>::Ones(1, 1).eval(), {"EnvelopeSurface"}};
    const auto input = simwild::read_mesh_data(data);
    REQUIRE(input.V_input.rows() == 4);
    REQUIRE(input.V_input.colwise().maxCoeff().maxCoeff() == 1);
    REQUIRE(input.V_envelope.rows() == 3);
    REQUIRE(input.F_envelope.rows() == 1);
    REQUIRE(input.envelope_name == "EnvelopeSurface");
    const auto separate = triangle();
    REQUIRE_THROWS(simwild::read_mesh_data(data, &separate));
    data.cells[CellType::Edge].connectivity.resize(1, 2);
    data.cells[CellType::Edge].connectivity << 0, 1;
    REQUIRE_THROWS(simwild::read_mesh_data(data));
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
TEST_CASE("TetWild embeds a named surface in memory", "[component_io]")
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
    const auto result = manifold_extraction::manifold_extraction(
        {{"input", "tagged"}, {"tag_selection", "tag_0"}, {"DEBUG_output", true}},
        {{"tagged", data}});
    check(result);
    REQUIRE(result.first.count("surface"));
    REQUIRE(result.first.at("surface").cells.count(CellType::Edge));
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
    "[component_io][memory_pipeline]")
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
}
#endif
