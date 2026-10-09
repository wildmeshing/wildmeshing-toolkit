#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;
using wmtk::MatrixXd;
using wmtk::Vector3d;

namespace {
PrismaticMeshInput make_mesh(
    MatrixXd vertices,
    std::vector<std::array<size_t, 4>> tets,
    const std::vector<size_t>& offset,
    size_t parent,
    const Vector3d& target)
{
    PrismaticMeshInput data;
    data.vertices = std::move(vertices);
    const size_t n = data.vertices.rows();
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(data.vertices, tets[i], 0))
            std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(data.vertices, tets[i], 0));
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = static_cast<int>(tets[i][j]);
    }
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(n, tets);
    data.vertex_tags.assign(n, 1);
    data.source_vertex_ids.resize(n);
    data.corr_input_vid.assign(n, -1);
    data.corr_input_vertex.assign(n, -1);
    data.vertex_component_ids.assign(n, -1);
    data.singular_vertex_tags.assign(n, -1);
    data.optimal_normals = MatrixXd::Zero(n, 3);
    data.target_positions = data.vertices;
    data.input_to_offset_vertices.resize(n);
    data.input_to_components.resize(n);
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), 1);
    for (size_t i = 0; i < n; ++i) data.source_vertex_ids[i] = static_cast<int64_t>(100 + i);
    OffsetComponent component;
    component.input_vertex = parent;
    component.vertices = offset;
    component.target_position = target;
    component.optimal_normal = Vector3d::UnitZ();
    component.singular = false;
    data.offset_components.push_back(component);
    data.input_to_components[parent] = {0};
    data.offset_vertices = offset;
    data.input_to_offset_vertices[parent] = offset;
    for (size_t v : offset) {
        data.vertex_tags[v] = 2;
        data.corr_input_vid[v] = data.source_vertex_ids[parent];
        data.corr_input_vertex[v] = parent;
        data.vertex_component_ids[v] = 0;
        data.singular_vertex_tags[v] = 0;
        data.target_positions.row(v) = target.transpose();
        data.optimal_normals.row(v) = Vector3d::UnitZ().transpose();
    }
    for (size_t v = 0; v < n; ++v)
        if (data.vertex_tags[v] == 1) data.input_vertices.push_back(v);
    return data;
}
PrismaticMeshInput collapse_fixture(double y = -1)
{
    MatrixXd v(6, 3);
    v << 0, 0, -1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 2, 0, 0, 1, y, 0;
    return make_mesh(
        v,
        {{0, 3, 2, 4}, {1, 2, 3, 4}, {0, 1, 2, 3}, {0, 1, 3, 5}, {1, 3, 5, 4}, {0, 3, 4, 5}},
        {3, 5},
        0,
        Vector3d(1, 0, 0.1));
}
PrismaticMeshInput smooth_fixture(const Vector3d& target, bool extra_tet = false)
{
    MatrixXd v(extra_tet ? 5 : 4, 3);
    v.topRows(4) << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    std::vector<std::array<size_t, 4>> tets = {{0, 1, 2, 3}};
    if (extra_tet) {
        v.row(4) << 0, -1, 1;
        tets.push_back({0, 1, 3, 4});
    }
    return make_mesh(v, tets, {3}, 0, target);
}
void check_synced(const PrismaticMeshInput& data, double threshold)
{
    REQUIRE(data.tetrahedra.rows() == data.mesh->get_tets().size());
    REQUIRE(data.input_cells.size() == data.mesh->get_tets().size());
    REQUIRE(data.offset_tet_tags.size() == data.mesh->get_tets().size());
    for (const auto& tet : data.mesh->get_tets()) {
        auto ids = data.mesh->oriented_tet_vids(tet);
        REQUIRE(tet_volume_above_threshold(data.vertices, ids, threshold));
        for (int j = 0; j < 4; ++j) REQUIRE(data.tetrahedra(tet.tid(*data.mesh), j) == ids[j]);
        REQUIRE(data.offset_tet_tags[tet.tid(*data.mesh)] == 1);
    }
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
}
} // namespace

TEST_CASE("exact volume checks reject inversion degeneracy and equality at the threshold")
{
    auto data = smooth_fixture(Vector3d(0, 0, 1));
    std::array<size_t, 4> tet = {0, 1, 2, 3};
    data.vertices(3, 2) = 0.75; // volume exactly 1/8
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, tet, 0.125));
    REQUIRE(tet_volume_above_threshold(data.vertices, tet, std::nextafter(0.125, 0.0)));
    data.vertices(3, 2) = 0;
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, tet, 0));
    data.vertices(3, 2) = -1;
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, tet, 0));
}

TEST_CASE("collapse preserves component correspondence tags fixed targets and synchronized cells")
{
    auto data = collapse_fixture();
    const auto normal = data.offset_components[0].optimal_normal;
    const auto target = data.offset_components[0].target_position;
    REQUIRE(data.mesh->link_condition(data.mesh->tuple_from_edge({5, 3})));
    REQUIRE(try_collapse_offset_edge(data, 5, 3, 1e-6));
    REQUIRE(data.mesh->get_vertices().size() == 5);
    REQUIRE(data.mesh->get_tets().size() == 3);
    REQUIRE(data.vertex_tags[3] == 2);
    REQUIRE(data.source_vertex_ids[3] == 103);
    REQUIRE(data.corr_input_vid[3] == 100);
    REQUIRE(data.corr_input_vertex[3] == 0);
    REQUIRE(data.vertex_component_ids[3] == 0);
    REQUIRE(data.vertex_component_ids[5] == -1);
    REQUIRE(data.offset_components[0].vertices == std::vector<size_t>{3});
    REQUIRE(data.input_to_offset_vertices[0] == std::vector<size_t>{3});
    REQUIRE(data.offset_components[0].optimal_normal == normal);
    REQUIRE(data.offset_components[0].target_position == target);
    check_synced(data, 1e-6);
}

TEST_CASE("collapse updates incident background connectivity and preserves region tags")
{
    auto data = collapse_fixture();
    data.vertex_tags[4] = -1;
    data.offset_tet_tags = {-1, -1, 1, 1, -1, -1};
    const auto vertices = data.vertices;
    REQUIRE(try_collapse_offset_edge(data, 3, 5, 1e-6));
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    REQUIRE(data.tetrahedra.rows() == 3);
    REQUIRE(std::count(data.offset_tet_tags.begin(), data.offset_tet_tags.end(), -1) == 2);
    REQUIRE(std::count(data.offset_tet_tags.begin(), data.offset_tet_tags.end(), 1) == 1);
    for (const auto& t : data.mesh->get_tets()) {
        const auto tet = data.mesh->oriented_tet_vids(t);
        REQUIRE(std::find(tet.begin(), tet.end(), 3) == tet.end());
        REQUIRE(std::find(tet.begin(), tet.end(), 5) != tet.end());
        REQUIRE(tet_volume_above_threshold(data.vertices, tet, 1e-6));
        if (data.offset_tet_tags[t.tid(*data.mesh)] == -1)
            REQUIRE(std::find(tet.begin(), tet.end(), 4) != tet.end());
    }
}

TEST_CASE("collapse rejects inversion and low positive candidate volumes without changing state")
{
    double y = 2, threshold = 1e-6;
    SECTION("inversion") {}
    SECTION("positive but too small")
    {
        y = 0.9;
        threshold = 0.1;
    }
    auto data = collapse_fixture(y);
    for (const auto& t : data.mesh->get_tets()) {
        REQUIRE(
            tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), threshold));
    }
    REQUIRE(data.mesh->link_condition(data.mesh->tuple_from_edge({3, 5})));
    const auto vertices = data.vertices;
    const auto tetrahedra = data.tetrahedra;
    const auto tags = data.vertex_tags;
    REQUIRE_FALSE(try_collapse_offset_edge(data, 3, 5, threshold));
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.tetrahedra == tetrahedra);
    REQUIRE(data.vertex_tags == tags);
    REQUIRE(data.offset_components[0].vertices == std::vector<size_t>{3, 5});
    check_synced(data, threshold);
}

TEST_CASE("collapse accepts small positive background cells but rejects their degeneration")
{
    double y = 0.9;
    bool accepted = true, shell_floor = false;
    SECTION("modified background can be below the shell floor") {}
    SECTION("the same small cells in the shell still block collapse")
    {
        shell_floor = true;
        accepted = false;
    }
    SECTION("zero-volume background blocks collapse")
    {
        y = 1;
        accepted = false;
    }
    SECTION("inverted background blocks collapse")
    {
        y = 2;
        accepted = false;
    }
    auto data = collapse_fixture(y);
    data.offset_tet_tags = {-1, -1, y < 1 ? 1 : -1, 1, -1, -1};
    if (shell_floor) data.offset_tet_tags[0] = 1;
    const auto vertices = data.vertices;
    const auto tets = data.tetrahedra;
    const auto tags = data.offset_tet_tags;
    REQUIRE(try_collapse_offset_edge(data, 3, 5, 0.025) == accepted);
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    if (!accepted) {
        REQUIRE(data.tetrahedra == tets);
        REQUIRE(data.offset_tet_tags == tags);
        return;
    }
    size_t small_background = 0;
    for (const auto& t : data.mesh->get_tets()) {
        const auto tet = data.mesh->oriented_tet_vids(t);
        REQUIRE(tet_volume_above_threshold(data.vertices, tet, 0));
        if (data.offset_tet_tags.at(t.tid(*data.mesh)) == 1)
            REQUIRE(tet_volume_above_threshold(data.vertices, tet, 0.025));
        else
            small_background += !tet_volume_above_threshold(data.vertices, tet, 0.025);
    }
    REQUIRE(small_background == 2);
}

TEST_CASE("collapse rejects different components input vertices and failed link conditions")
{
    auto data = collapse_fixture();
    data.vertex_component_ids[5] = 1;
    REQUIRE_FALSE(try_collapse_offset_edge(data, 5, 3, 0));
    REQUIRE_FALSE(try_collapse_offset_edge(data, 0, 3, 0));
    auto single = smooth_fixture(Vector3d(0, 0, 1));
    single.vertex_tags[1] = 2;
    single.vertex_component_ids[1] = 0;
    single.corr_input_vid[1] = 100;
    single.corr_input_vertex[1] = 0;
    REQUIRE_FALSE(single.mesh->link_condition(single.mesh->tuple_from_edge({1, 3})));
    REQUIRE_FALSE(try_collapse_offset_edge(single, 1, 3, 0));
    REQUIRE(single.mesh->get_tets().size() == 1);
}

TEST_CASE("collapse can delete small tets and preserve unchanged small survivor tets")
{
    auto data = collapse_fixture();
    SECTION("small disappearing cells")
    {
        data.vertices(5, 1) = -1e-6;
    }
    SECTION("small unchanged survivor cells")
    {
        data.vertices(4, 0) = 1 + 1e-6;
    }
    const auto vertices = data.vertices;
    const auto small_tet = data.mesh->oriented_tet_vids(5);
    REQUIRE(tet_volume_above_threshold(data.vertices, small_tet, 0));
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, small_tet, 0.01));
    REQUIRE(try_collapse_offset_edge(data, 5, 3, 0.01));
    REQUIRE(data.tetrahedra.rows() == 3);
    REQUIRE(data.vertices == vertices);
    check_synced(data, 0);
}

TEST_CASE("smoothing may raise an existing small positive tet above the operation floor")
{
    auto data = smooth_fixture(Vector3d(0, 0, 1));
    data.vertices(3, 2) = 0.01;
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, {0, 1, 2, 3}, 0.05));
    REQUIRE(smooth_offset_vertex(data, 3, 0.05, 8) == 1);
    REQUIRE(data.vertices(3, 2) == 1);
    check_synced(data, 0.05);
}

TEST_CASE("smoothing uses only positive-volume checks for incident background cells")
{
    double target_y = -0.94, expected_alpha = 1;
    bool shell = false;
    SECTION("small positive background allows a full step") {}
    SECTION("the same small shell cell requires backtracking")
    {
        shell = true;
        expected_alpha = 0.5;
    }
    SECTION("zero-volume background requires backtracking")
    {
        target_y = -1;
        expected_alpha = 0.5;
    }
    SECTION("inverted background requires backtracking")
    {
        target_y = -2;
        expected_alpha = 0.25;
    }
    auto data = smooth_fixture(Vector3d(0, target_y, 1), true);
    data.offset_tet_tags[1] = shell ? 1 : -1;
    REQUIRE(smooth_offset_vertex(data, 3, 0.05, 8) == expected_alpha);
    REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(0), 0.05));
    const auto second = data.mesh->oriented_tet_vids(1);
    REQUIRE(tet_volume_above_threshold(data.vertices, second, 0));
    if (expected_alpha == 1)
        REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, second, 0.05));
    else
        REQUIRE(tet_volume_above_threshold(data.vertices, second, 0.05));
}

TEST_CASE("smoothing halves towards the target and checks every incident tet")
{
    Vector3d target(0, 0, 0.75);
    double expected = 1;
    bool extra = false;
    SECTION("full safe step") {}
    SECTION("inverting full step")
    {
        target = Vector3d(0, 0, -1);
        expected = 0.25;
    }
    SECTION("positive but below threshold")
    {
        target = Vector3d(0, 0, 0.1);
        expected = 0.5;
    }
    SECTION("second incident tet limits the move")
    {
        target = Vector3d(0, -2, 1);
        expected = 0.25;
        extra = true;
    }
    auto data = smooth_fixture(target, extra);
    const Vector3d start = data.vertices.row(3);
    const auto tets = data.tetrahedra;
    REQUIRE(smooth_offset_vertex(data, 3, 0.05, 40) == expected);
    REQUIRE(
        (data.vertices.row(3).transpose() - ((1 - expected) * start + expected * target)).norm() <
        1e-12);
    REQUIRE(data.tetrahedra == tets);
    REQUIRE(data.offset_components[0].target_position == target);
    check_synced(data, 0.05);
}

TEST_CASE("smoothing skips singular components and leaves coordinates unchanged on exhaustion")
{
    auto data = smooth_fixture(Vector3d(0, 0, -1));
    const auto vertices = data.vertices;
    REQUIRE(smooth_offset_vertex(data, 3, 0.05, 0) == 0);
    REQUIRE(data.vertices == vertices);
    data.offset_components[0].singular = true;
    data.singular_vertex_tags[3] = 1;
    REQUIRE(smooth_offset_vertex(data, 3, 0.05, 40) == 0);
    REQUIRE(data.vertices == vertices);
    REQUIRE(smooth_offset_vertex(data, 0, 0.05, 40) == 0);
}

TEST_CASE("optimization executes configured iterations and keeps the original target")
{
    auto data = collapse_fixture();
    const auto target = data.offset_components[0].target_position;
    OptimizationOptions options;
    options.iterations = 3;
    options.min_tet_volume = 1e-6;
    optimize_prismatic_mesh(data, options);
    REQUIRE(data.optimization_iterations.size() == 3);
    REQUIRE(data.optimization_iterations[0].collapses == 1);
    REQUIRE(data.optimization_iterations[0].smoothed_vertices == 1);
    REQUIRE(data.offset_components[0].target_position == target);
    REQUIRE(data.offset_components[0].vertices == std::vector<size_t>{3});
    REQUIRE((data.vertices.row(3).transpose() - target).norm() < 1e-12);
    check_synced(data, 1e-6);
    options.iterations = 0;
    const auto vertices = data.vertices;
    optimize_prismatic_mesh(data, options);
    REQUIRE(data.optimization_iterations.empty());
    REQUIRE(data.vertices == vertices);
    options.iterations = -1;
    REQUIRE_THROWS(optimize_prismatic_mesh(data, options));
    options.iterations = 1;
    options.min_tet_volume = 100;
    REQUIRE_NOTHROW(optimize_prismatic_mesh(data, options));
    REQUIRE(data.vertices == vertices);
    check_synced(data, 0);
}

TEST_CASE("optimization skips a below-floor operation and continues elsewhere")
{
    MatrixXd vertices(8, 3);
    vertices << 0, 0, 0, 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1, 0, 0, 2, 1, 0, 2, 0, 1, 2, 0, 0, 3;
    auto data = make_mesh(vertices, {{0, 1, 2, 3}, {4, 5, 6, 7}}, {3, 7}, 0, Vector3d(0, 0, 4));
    OptimizationOptions options;
    options.iterations = 1;
    options.min_tet_volume = 0.05;
    REQUIRE_NOTHROW(optimize_prismatic_mesh(data, options));
    REQUIRE(data.optimization_iterations.size() == 1);
    REQUIRE(data.optimization_iterations[0].smoothed_vertices == 1);
    REQUIRE(data.optimization_iterations[0].smoothing_failures == 1);
    REQUIRE(data.vertices.row(3) == vertices.row(3));
    REQUIRE(data.vertices(7, 2) == 4);
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, {0, 1, 2, 3}, options.min_tet_volume));
    REQUIRE(tet_volume_above_threshold(data.vertices, {4, 5, 6, 7}, options.min_tet_volume));
    check_synced(data, 0);
}

TEST_CASE("initial optimization cells must still have positive volume")
{
    auto data = smooth_fixture(Vector3d(0, 0, 1));
    SECTION("zero volume")
    {
        data.vertices(3, 2) = 0;
    }
    SECTION("negative volume")
    {
        data.vertices(3, 2) = -1;
    }
    OptimizationOptions options;
    options.iterations = 1;
    REQUIRE_THROWS(optimize_prismatic_mesh(data, options));
}

TEST_CASE("a collapse pass preserves cell attributes across multiple local updates")
{
    const auto fixture = collapse_fixture();
    MatrixXd vertices(12, 3);
    vertices.topRows(6) = fixture.vertices;
    vertices.bottomRows(6) = fixture.vertices.rowwise() + Vector3d(10, 0, 0).transpose();
    std::vector<std::array<size_t, 4>> tets;
    for (size_t shift : {0, 6}) {
        for (Eigen::Index i = 0; i < fixture.tetrahedra.rows(); ++i) {
            std::array<size_t, 4> tet;
            for (int j = 0; j < 4; ++j) tet[j] = fixture.tetrahedra(i, j) + shift;
            tets.push_back(tet);
        }
    }
    auto prepare = [&]() {
        auto data = make_mesh(vertices, tets, {3, 5, 9, 11}, 0, Vector3d::Zero());
        data.offset_components[0].singular = true; // Test collapse without smoothing.
        for (size_t v : data.offset_vertices) data.singular_vertex_tags[v] = 1;
        data.offset_tet_tags = {-1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1};
        return data;
    };
    auto reference = prepare();
    REQUIRE(try_collapse_offset_edge(reference, 5, 3, 1e-6));
    REQUIRE(try_collapse_offset_edge(reference, 11, 9, 1e-6));
    auto local = prepare();
    OptimizationOptions options;
    options.iterations = 1;
    options.min_tet_volume = 1e-6;
    optimize_prismatic_mesh(local, options);
    REQUIRE(local.optimization_iterations[0].collapses == 2);
    REQUIRE(local.tetrahedra == reference.tetrahedra);
    REQUIRE(local.vertices == reference.vertices);
    REQUIRE(local.input_cells == reference.input_cells);
    REQUIRE(local.offset_tet_tags == reference.offset_tet_tags);
    REQUIRE(local.offset_vertices == reference.offset_vertices);
    REQUIRE(local.input_to_offset_vertices == reference.input_to_offset_vertices);
    REQUIRE(local.vertex_component_ids == reference.vertex_component_ids);
    REQUIRE(local.mesh->check_mesh_connectivity_validity());
    REQUIRE(local.tetrahedra.rows() == local.mesh->get_tets().size());
    for (const auto& t : local.mesh->get_tets()) {
        const auto tet = local.mesh->oriented_tet_vids(t);
        for (int j = 0; j < 4; ++j) REQUIRE(local.tetrahedra(t.tid(*local.mesh), j) == tet[j]);
        REQUIRE(tet_volume_above_threshold(local.vertices, tet, options.min_tet_volume));
    }
}
