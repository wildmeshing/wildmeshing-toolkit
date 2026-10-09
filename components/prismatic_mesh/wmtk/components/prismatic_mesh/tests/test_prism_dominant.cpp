#include <mshio/mshio.h>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <fstream>
#include <wmtk/components/prismatic_mesh/prism_jacobian.hpp>
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;

namespace {
PrismaticMeshInput fixture(
    const std::vector<wmtk::Vector3d>& extra = {},
    const std::vector<std::array<size_t, 4>>& extra_tets = {},
    const std::vector<int64_t>& extra_parents = {})
{
    PrismaticMeshInput data;
    const size_t n = 6 + extra.size();
    data.vertices.resize(n, 3);
    data.vertices.topRows(6) << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1;
    data.vertex_tags = {1, 1, 1, 2, 2, 2};
    data.corr_input_vertex = {-1, -1, -1, 0, 1, 2};
    for (size_t i = 0; i < extra.size(); ++i) {
        data.vertices.row(6 + i) = extra[i].transpose();
        const int64_t parent = extra_parents.empty() ? -1 : extra_parents.at(i);
        data.vertex_tags.push_back(parent >= 0 ? 2 : -1);
        data.corr_input_vertex.push_back(parent);
    }
    data.source_vertex_ids.resize(n);
    data.corr_input_vid.resize(n);
    data.input_to_offset_vertices.resize(n);
    for (size_t v = 0; v < n; ++v) {
        data.source_vertex_ids[v] = 100 + v;
        data.corr_input_vid[v] =
            data.corr_input_vertex[v] < 0 ? -1 : 100 + data.corr_input_vertex[v];
        if (data.vertex_tags[v] == 1) data.input_vertices.push_back(v);
        if (data.vertex_tags[v] == 2) {
            data.offset_vertices.push_back(v);
            data.input_to_offset_vertices[data.corr_input_vertex[v]].push_back(v);
        }
    }
    std::vector<std::array<size_t, 4>> tets = {{0, 1, 2, 5}, {0, 1, 5, 4}, {0, 3, 4, 5}};
    tets.insert(tets.end(), extra_tets.begin(), extra_tets.end());
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(data.vertices, tets[i], 0))
            std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(data.vertices, tets[i], 0));
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = tets[i][j];
    }
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), 1);
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(n, tets);
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    return data;
}

size_t count(const PrismDominantMesh& hybrid, HybridCellType type)
{
    return std::count_if(hybrid.cells.begin(), hybrid.cells.end(), [&](const auto& c) {
        return c.type == type;
    });
}

void check_partition(const PrismaticMeshInput& input, const PrismDominantMesh& hybrid)
{
    std::vector<size_t> visits(input.tetrahedra.rows(), 0);
    for (const auto& c : hybrid.cells)
        for (size_t tid : c.source_tets) ++visits.at(tid);
    REQUIRE(std::all_of(visits.begin(), visits.end(), [](size_t n) { return n == 1; }));
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}

PrismaticMeshInput grid_fixture()
{
    constexpr size_t n = 7, layer = n * n;
    PrismaticMeshInput data;
    data.vertices.resize(2 * layer, 3);
    data.vertex_tags.resize(2 * layer);
    data.source_vertex_ids.resize(2 * layer);
    data.corr_input_vid.assign(2 * layer, -1);
    data.corr_input_vertex.assign(2 * layer, -1);
    data.input_to_offset_vertices.resize(2 * layer);
    for (size_t k = 0; k < 2; ++k)
        for (size_t j = 0; j < n; ++j)
            for (size_t i = 0; i < n; ++i) {
                const size_t v = k * layer + j * n + i;
                data.vertices.row(v) << i, j, k;
                data.vertex_tags[v] = k ? 2 : 1;
                data.source_vertex_ids[v] = 100 + v;
                if (k) {
                    const size_t parent = (i == 3 && j == 3) ? v - layer - 1 : v - layer;
                    data.corr_input_vertex[v] = parent;
                    data.corr_input_vid[v] = 100 + parent;
                    data.input_to_offset_vertices[parent].push_back(v);
                    data.offset_vertices.push_back(v);
                } else
                    data.input_vertices.push_back(v);
            }
    std::vector<std::array<size_t, 4>> tets;
    for (size_t j = 0; j + 1 < n; ++j)
        for (size_t i = 0; i + 1 < n; ++i) {
            const size_t a = j * n + i, b = a + 1, c = a + n, d = c + 1;
            for (auto f : {std::array<size_t, 3>{a, b, d}, std::array<size_t, 3>{a, c, d}}) {
                const auto x = f[0], y = f[1], z = f[2];
                for (auto t :
                     {std::array<size_t, 4>{x, y, z, z + layer},
                      std::array<size_t, 4>{x, y, z + layer, y + layer},
                      std::array<size_t, 4>{x, x + layer, y + layer, z + layer}}) {
                    if (!tet_volume_above_threshold(data.vertices, t, 0)) std::swap(t[0], t[1]);
                    tets.push_back(t);
                }
            }
        }
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = tets[i][j];
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), 1);
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(2 * layer, tets);
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    return data;
}
} // namespace

TEST_CASE("input-apex pyramids drive compatible retriangulation of neighboring prism regions")
{
    auto data = grid_fixture();
    const auto positions = data.vertices;
    const auto original = data.tetrahedra;
    const auto correspondence = data.corr_input_vid;
    const auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(count(hybrid, HybridCellType::Pyramid) > 0);
    REQUIRE(count(hybrid, HybridCellType::Prism) > 0);
    REQUIRE(hybrid.report["retriangulated_prisms"].get<size_t>() > 0);
    REQUIRE(hybrid.report["direction_search_limit_fallbacks"] == 0);
    REQUIRE(data.vertices == positions);
    REQUIRE(data.corr_input_vid == correspondence);
    REQUIRE(data.tetrahedra != original);
    for (const auto& cell : hybrid.cells)
        if (cell.type == HybridCellType::Pyramid) REQUIRE(data.vertex_tags[cell.vertices[4]] == 1);
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    for (const auto& t : data.mesh->get_tets()) {
        const auto vids = data.mesh->oriented_tet_vids(t);
        REQUIRE(tet_volume_above_threshold(data.vertices, vids, 0));
    }
    check_partition(data, hybrid);
}

TEST_CASE("interface reconstruction filters bad splits of Jacobian-valid sheared prisms")
{
    auto data = grid_fixture();
    // This shear preserves det(J)=1 in matched prism columns and the original
    // tet volumes, but makes some alternative three-tet triangulations inverted.
    for (Eigen::Index i = 0; i < data.vertices.rows(); ++i)
        data.vertices(i, 0) -= 2 * data.vertices(i, 1) * data.vertices(i, 2);
    double floor = 0;
    SECTION("positive retriangulation") {}
    SECTION("new tets below the operation floor stay excluded")
    {
        floor = 0.2; // Existing source tets have volume 1/6 and may remain.
    }
    const auto original = data.tetrahedra;
    const auto hybrid = build_prism_dominant_mesh(data, floor);
    REQUIRE(hybrid.report["jacobian_valid_candidate_admissible_split_counts"].value("6", 0) == 0);
    if (floor > 0) {
        REQUIRE(data.tetrahedra == original);
        REQUIRE(hybrid.report["retriangulated_prisms"] == 0);
        REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 216);
    } else {
        REQUIRE(count(hybrid, HybridCellType::Prism) > 0);
        REQUIRE(count(hybrid, HybridCellType::Pyramid) > 0);
        // Incompatible transition directions must expand the retained region
        // rather than select an inverted alternative that satisfies only topology.
        REQUIRE(hybrid.report["direction_region_expansions"].get<size_t>() > 0);
    }
    for (const auto& t : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), 0));
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(data, hybrid, floor));
    check_partition(data, hybrid);
}

TEST_CASE("MSH round trip preserves hybrid connectivity, regions and reference decompositions")
{
    auto data = fixture({{0, 0, -1}, {0, 0, 2}}, {{0, 2, 1, 6}, {3, 4, 5, 7}});
    bool split_prism = false;
    bool include_background = true;
    SECTION("prism with fixed input and background") {}
    SECTION("pyramid transition with fixed input and background")
    {
        split_prism = true;
    }
    SECTION("input and shell prism export")
    {
        include_background = false;
    }
    SECTION("input and shell pyramid export")
    {
        split_prism = true;
        include_background = false;
    }
    data.vertex_tags[6] = 1;
    data.input_vertices.push_back(6);
    data.input_cells[3] = 1;
    data.offset_tet_tags[3] = data.offset_tet_tags[4] = -1;
    data.vertices.array() += 0.12345678901234566; // exercise full double precision
    evaluate_target_positions(data, 0.1);
    label_offset_faces(data);
    auto hybrid = build_prism_dominant_mesh(data, 0);
    if (split_prism) {
        // Exercise pyramid serialization independently of frontier selection.
        hybrid.cells = {
            {HybridCellType::Pyramid, {1, 4, 5, 2, 0}, {0, 1}, 0},
            {HybridCellType::Tetrahedron, {0, 3, 4, 5}, {2}, 0}};
        for (size_t tid : {3, 4}) {
            HybridCell cell;
            cell.source_tets = {tid};
            for (int j = 0; j < 4; ++j) cell.vertices.push_back(data.tetrahedra(tid, j));
            hybrid.cells.push_back(cell);
        }
        for (size_t ci = 0; ci < hybrid.cells.size(); ++ci)
            for (size_t tid : hybrid.cells[ci].source_tets) hybrid.source_tet_to_cell[tid] = ci;
        hybrid.report["prisms"] = 0;
        hybrid.report["pyramids"] = 1;
        hybrid.report["band_tetrahedra"] = 1;
        hybrid.report["output_cells"] = hybrid.cells.size();
        REQUIRE_NOTHROW(validate_prism_dominant_mesh(data, hybrid, 0));
    }

    struct Output
    {
        std::filesystem::path path =
            std::filesystem::temp_directory_path() /
            ("prismatic_msh_test_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".msh");
        ~Output()
        {
            std::error_code ec;
            std::filesystem::remove(path, ec);
            for (const auto* suffix : {"_hybrid.json", "_offset_faces.vtu"})
                std::filesystem::remove(path.parent_path() / (path.stem().string() + suffix), ec);
        }
    } output;
    const auto original_vertices = data.vertices;
    const auto original_tetrahedra = data.tetrahedra;
    write_prismatic_mesh(data, output.path, &hybrid, include_background);
    REQUIRE(data.vertices == original_vertices);
    REQUIRE(data.tetrahedra == original_tetrahedra);
    const auto msh = mshio::load_msh(output.path.string());
    REQUIRE_NOTHROW(mshio::validate_spec(msh));
    REQUIRE(msh.mesh_format.version == "4.1");
    REQUIRE(msh.mesh_format.file_type == 0);
    REQUIRE(msh.nodes.num_nodes == data.vertices.rows() - (include_background ? 0 : 1));
    REQUIRE(msh.elements.num_elements == hybrid.cells.size() - (include_background ? 0 : 1));
    REQUIRE(msh.physical_groups.size() == (include_background ? 3 : 2));
    REQUIRE(msh.entities.volumes.size() == msh.physical_groups.size());
    for (size_t i = 0; i < msh.physical_groups.size(); ++i) {
        REQUIRE(msh.physical_groups[i].dim == 3);
        REQUIRE(msh.physical_groups[i].tag == i + 1);
        REQUIRE(msh.entities.volumes[i].physical_group_tags == std::vector<int>{int(i + 1)});
    }
    REQUIRE(msh.physical_groups[0].name == "input");
    REQUIRE(msh.physical_groups[1].name == "offset_band");
    if (include_background) REQUIRE(msh.physical_groups[2].name == "background");
    const auto field = [](const auto& fields, const std::string& name) -> const mshio::Data& {
        const auto it = std::find_if(fields.begin(), fields.end(), [&](const auto& f) {
            return f.header.string_tags.at(0) == name;
        });
        REQUIRE(it != fields.end());
        return *it;
    };
    std::vector<size_t> node_tags(data.vertices.rows());
    for (const auto& block : msh.nodes.entity_blocks) {
        for (size_t i = 0; i < block.tags.size(); ++i) {
            const size_t idx = block.tags[i] - 1;
            const size_t v =
                static_cast<size_t>(field(msh.node_data, "vid").entries[idx].data[0]) - 100;
            node_tags.at(v) = block.tags[i];
            REQUIRE(
                field(msh.node_data, "corr_input_vid").entries[idx].data[0] ==
                data.corr_input_vid[v]);
            REQUIRE(field(msh.node_data, "labels").entries[idx].data[0] == data.vertex_tags[v]);
            for (int j = 0; j < 3; ++j) {
                REQUIRE(block.data[3 * i + j] == data.vertices(v, j));
                REQUIRE(
                    field(msh.node_data, "target_position").entries[idx].data[j] ==
                    data.target_positions(v, j));
            }
        }
    }
    std::vector<bool> visited(hybrid.cells.size(), false);
    size_t tag = 0;
    for (const auto& block : msh.elements.entity_blocks) {
        const size_t corners = mshio::nodes_per_element(block.element_type);
        for (size_t i = 0; i < block.num_elements_in_block; ++i) {
            REQUIRE(block.data[i * (corners + 1)] == ++tag);
            const auto value = [&](const std::string& name, int component = 0) {
                const auto& entry = field(msh.element_data, name).entries[tag - 1];
                REQUIRE(entry.tag == tag);
                return entry.data[component];
            };
            const size_t tid = static_cast<size_t>(value("source_tet_ids"));
            const size_t cid = hybrid.source_tet_to_cell.at(tid);
            REQUIRE_FALSE(visited[cid]);
            visited[cid] = true;
            const auto& cell = hybrid.cells[cid];
            const int expected_type = cell.type == HybridCellType::Prism     ? 6
                                      : cell.type == HybridCellType::Pyramid ? 7
                                                                             : 4;
            REQUIRE(block.element_type == expected_type);
            REQUIRE(corners == cell.vertices.size());
            for (size_t j = 0; j < corners; ++j)
                REQUIRE(block.data[i * (corners + 1) + 1 + j] == node_tags[cell.vertices[j]]);
            REQUIRE(value("tag_0") == data.input_cells[tid]);
            REQUIRE(value("offset_tag") == data.offset_tet_tags[tid]);
            const int region = data.offset_tet_tags[tid] == 1 ? 2
                               : data.input_cells[tid] == 1   ? 1
                                                              : 3;
            REQUIRE(block.entity_tag == region);
            REQUIRE(value("source_tet_count") == cell.source_tets.size());
            REQUIRE(value("prism_candidate") == cell.prism_candidate);
            REQUIRE(value("cell_type") == static_cast<int>(cell.type));
            for (size_t k = 0; k < 3; ++k) {
                const double source =
                    k < cell.source_tets.size() ? double(cell.source_tets[k]) : -1;
                REQUIRE(value("source_tet_ids", k) == source);
                for (size_t j = 0; j < 4; ++j) {
                    const int local =
                        static_cast<int>(value("tet_decomposition_" + std::to_string(4 * k + j)));
                    if (k < cell.source_tets.size())
                        REQUIRE(cell.vertices.at(local) == data.tetrahedra(cell.source_tets[k], j));
                    else
                        REQUIRE(local == -1);
                }
            }
        }
    }
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci) {
        const auto tid = hybrid.cells[ci].source_tets[0];
        REQUIRE(
            visited[ci] ==
            (include_background || data.input_cells[tid] == 1 || data.offset_tet_tags[tid] == 1));
    }
    if (!include_background) {
        std::ifstream stream(
            output.path.parent_path() / (output.path.stem().string() + "_hybrid.json"));
        nlohmann::json report;
        stream >> report;
        REQUIRE(report["background_tetrahedra"] == 0);
        REQUIRE(report["source_tetrahedra"] == 4);
        REQUIRE(report["full_reference_tetrahedra"] == 5);
        REQUIRE(report["output_cells"] == msh.elements.num_elements);
        REQUIRE(report["export_scope"] == "input_and_shell");
    }
    REQUIRE(
        std::filesystem::exists(
            output.path.parent_path() / (output.path.stem().string() + "_hybrid.json")));
    REQUIRE(
        std::filesystem::exists(
            output.path.parent_path() / (output.path.stem().string() + "_offset_faces.vtu")));
}

TEST_CASE("bijective offset triangle produces a prism while input and background tets stay fixed")
{
    auto data = fixture({{0, 0, -1}, {0, 0, 2}}, {{0, 2, 1, 6}, {3, 4, 5, 7}});
    data.vertex_tags[6] = 1;
    data.input_vertices.push_back(6);
    data.input_cells[3] = 1;
    data.offset_tet_tags[3] = data.offset_tet_tags[4] = -1;
    const auto vertices = data.vertices;
    const auto tetrahedra = data.tetrahedra;
    const auto correspondence = data.corr_input_vid;
    auto hybrid = build_prism_dominant_mesh(data, 1e-8);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 1);
    REQUIRE(count(hybrid, HybridCellType::Pyramid) == 0);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 2);
    REQUIRE(hybrid.report["input_tetrahedra"] == 1);
    REQUIRE(hybrid.report["background_tetrahedra"] == 1);
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.tetrahedra == tetrahedra);
    REQUIRE(data.corr_input_vid == correspondence);
    for (size_t tid : {3, 4}) {
        const auto& cell = hybrid.cells[hybrid.source_tet_to_cell[tid]];
        REQUIRE(cell.source_tets == std::vector<size_t>{tid});
        for (int j = 0; j < 4; ++j) REQUIRE(cell.vertices[j] == tetrahedra(tid, j));
    }
    check_partition(data, hybrid);
}

TEST_CASE("a transition blocked by original core tets expands the retained tet region")
{
    // The extra two band tets expose a repeated-correspondence offset face (5,6,7).
    // Only corner 5 of the regular prism touches it; its opposite quad remains a quad.
    auto data = fixture({{-1, 1, 0.5}, {-1, 2, 0.5}}, {{0, 2, 5, 6}, {2, 5, 6, 7}}, {2, 2});
    auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(hybrid.report["repeated_correspondence_faces"] == 1);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 0);
    REQUIRE(count(hybrid, HybridCellType::Pyramid) == 0);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 5);
    REQUIRE(hybrid.report["direction_region_expansions"] == 1);
    REQUIRE(hybrid.report["unassigned_band_tets_retained"] == 2);
    check_partition(data, hybrid);
}

TEST_CASE("two sides of an input sheet can share the same triangular prism cap")
{
    auto data = fixture(
        {{0, 0, -1}, {1, 0, -1}, {0, 1, -1}},
        {{0, 1, 2, 8}, {0, 1, 8, 7}, {0, 6, 7, 8}},
        {0, 1, 2});
    const auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 2);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 0);
    check_partition(data, hybrid);
}

TEST_CASE("two repeated-correspondence endpoints retain the full local tetrahedral partition")
{
    auto data = fixture({{0.5, -1, 1}}, {{0, 3, 4, 6}}, {0});
    auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(hybrid.report["repeated_correspondence_faces"] == 1);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 0);
    REQUIRE(count(hybrid, HybridCellType::Pyramid) == 0);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 4);
    check_partition(data, hybrid);
}

TEST_CASE("a Jacobian-valid prism can keep its reference split despite bad alternatives")
{
    auto data = fixture();
    double x = -2, floor = 0;
    SECTION("inverted alternative") {}
    SECTION("zero-volume alternative")
    {
        x = -1;
    }
    SECTION("positive alternative below the configured floor")
    {
        x = -0.99;
        floor = 0.01;
    }
    data.vertices(5, 0) = x;
    // x(r,s,t) = (r + x*s*t, s, t) has det(J)=1 throughout the reference prism.
    // Some unused diagonal choices nevertheless have negative, zero or small volume.
    const auto original = data.tetrahedra;
    for (const auto& t : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), floor));
    auto hybrid = build_prism_dominant_mesh(data, floor);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 1);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 0);
    REQUIRE(hybrid.report["expanded_special_offset_vertices"] == 0);
    REQUIRE(hybrid.report["prism_jacobian_check"]["minimum_output_prism_det_j"] == 1);
    REQUIRE(hybrid.report["jacobian_valid_candidate_admissible_split_counts"].value("6", 0) == 0);
    REQUIRE(data.tetrahedra == original);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(data, hybrid, floor));
    check_partition(data, hybrid);
}

TEST_CASE("unchanged below-floor source tets do not veto a Jacobian-valid prism")
{
    auto data = fixture();
    const auto original = data.tetrahedra;
    const auto positions = data.vertices;
    const auto hybrid = build_prism_dominant_mesh(data, 1);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 1);
    REQUIRE(count(hybrid, HybridCellType::Pyramid) == 0);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 0);
    REQUIRE(hybrid.report["jacobian_valid_candidate_admissible_split_counts"]["1"] == 1);
    REQUIRE(data.tetrahedra == original);
    REQUIRE(data.vertices == positions);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(data, hybrid, 1));
    check_partition(data, hybrid);
}

TEST_CASE("prism Jacobians are checked between the caps as well as at the corners")
{
    auto data = fixture();
    auto hybrid = build_prism_dominant_mesh(data, 0);
    data.vertices.row(3) << -2, 0, 2;
    data.vertices.row(4) << -2, -2, 1;
    data.vertices.row(5) << 0, 0, 1;
    // Original tet determinants are 1,2,4 and corner Jacobians are 2,1,1,4,2,6.
    // At (r,s,t)=(1,0,1/2), det(J)=-1/2; corner-only checks would miss this.
    for (const auto& t : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), 0));
    REQUIRE_THROWS(validate_prism_dominant_mesh(data, hybrid, 0));
    const auto original = data.tetrahedra;
    hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(hybrid.report["candidate_rejections"]["invalid_prism_jacobian"] == 1);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 3);
    REQUIRE(data.tetrahedra == original);
    check_partition(data, hybrid);
}

TEST_CASE("a zero sampled prism Jacobian retains positive source tetrahedra")
{
    auto data = fixture();
    // The input corner (0,0,0) has a horizontal extrusion direction, while the
    // original three tetrahedra are still strictly positive.
    data.vertices.row(3) << -1, -1, 0;
    for (const auto& t : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), 0));
    const auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(hybrid.report["candidate_rejections"]["invalid_prism_jacobian"] == 1);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 3);
    check_partition(data, hybrid);
}

TEST_CASE("a small unmatched shell tet does not prevent other prisms from being built")
{
    auto data = fixture({{2, 0, 0}, {2.001, 0, 0}, {2, 0.001, 0}, {2, 0, 0.001}}, {{6, 7, 8, 9}});
    const auto original_small_tet = data.tetrahedra.row(3).eval();
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, {6, 7, 8, 9}, 0.01));
    const auto hybrid = build_prism_dominant_mesh(data, 0.01);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 1);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 1);
    REQUIRE(hybrid.report["unassigned_band_tets_retained"] == 1);
    REQUIRE(data.tetrahedra.row(3) == original_small_tet);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(data, hybrid, 0.01));
    check_partition(data, hybrid);
}

TEST_CASE("a fixed lateral interface expands the input-apex transition region")
{
    auto data = fixture({{-1, 1, 0.5}}, {{0, 2, 5, 6}});
    data.offset_tet_tags[3] = -1;
    auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(hybrid.report["initial_special_offset_vertices"] == 0);
    REQUIRE(hybrid.report["quad_constraints_added"].get<size_t>() >= 1);
    REQUIRE(hybrid.report["compatibility_passes"].get<size_t>() >= 2);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 0);
    REQUIRE(count(hybrid, HybridCellType::Pyramid) == 0);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 4);
    REQUIRE(hybrid.report["direction_region_expansions"] == 1);
    check_partition(data, hybrid);
}

TEST_CASE("adjacent prisms share a quadrilateral and reject a one-sided tetrahedral fallback")
{
    auto data =
        fixture({{1, 1, 0}, {1, 1, 1}}, {{1, 6, 2, 5}, {1, 6, 5, 7}, {1, 4, 7, 5}}, {-1, 6});
    data.vertex_tags[6] = 1;
    auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 2);
    check_partition(data, hybrid);
    // Independently split just one output prism. The common quad is now two triangles,
    // so exact source coverage alone is insufficient and validation must reject it.
    auto first = hybrid.cells[0];
    hybrid.cells.erase(hybrid.cells.begin());
    for (size_t tid : first.source_tets) {
        HybridCell c;
        c.source_tets = {tid};
        for (int j = 0; j < 4; ++j) c.vertices.push_back(data.tetrahedra(tid, j));
        hybrid.cells.push_back(c);
    }
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci)
        for (size_t tid : hybrid.cells[ci].source_tets) hybrid.source_tet_to_cell[tid] = ci;
    REQUIRE_THROWS(validate_prism_dominant_mesh(data, hybrid, 0));
}

TEST_CASE("pyramid validation checks the second diagonal as well as its source tets")
{
    auto data = fixture();
    PrismDominantMesh hybrid;
    hybrid.cells = {
        {HybridCellType::Pyramid, {1, 4, 5, 2, 0}, {0, 1}, 0},
        {HybridCellType::Tetrahedron, {0, 3, 4, 5}, {2}, 0}};
    hybrid.source_tet_to_cell = {0, 0, 1};
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(data, hybrid, 0));
    data.vertices.row(4) << 1, -1, -0.5;
    for (const auto& t : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), 0));
    REQUIRE_FALSE(tet_volume_above_threshold(data.vertices, {1, 4, 2, 0}, 0));
    REQUIRE_THROWS(validate_prism_dominant_mesh(data, hybrid, 0));
}

TEST_CASE("a positive offset-apex pyramid is rejected by the uniform input-apex rule")
{
    auto data = fixture();
    PrismDominantMesh hybrid;
    hybrid.cells = {
        {HybridCellType::Pyramid, {0, 3, 4, 1, 5}, {1, 2}, 0},
        {HybridCellType::Tetrahedron, {0, 1, 2, 5}, {0}, 0}};
    hybrid.source_tet_to_cell = {1, 0, 0};
    REQUIRE_THROWS(validate_prism_dominant_mesh(data, hybrid, 0));
}

TEST_CASE("an invalid prism expands the tetrahedral region into its neighbor")
{
    auto data =
        fixture({{1, 1, 0}, {1, 1, 1}}, {{1, 6, 2, 5}, {1, 6, 5, 7}, {1, 4, 7, 5}}, {-1, 6});
    data.vertex_tags[6] = 1;
    data.vertices.row(3) << -2, -2, -2;
    for (const auto& t : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), 0));
    const auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(hybrid.report["candidate_rejections"]["invalid_prism_jacobian"] == 1);
    REQUIRE(hybrid.report["fully_tetrahedral_candidates"] == 2);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 6);
    check_partition(data, hybrid);
}

TEST_CASE("distinct correspondence IDs require an actual matching input triangle")
{
    auto data = fixture({{0, 2, 0}});
    data.vertex_tags[6] = 1;
    data.corr_input_vertex[5] = 6;
    data.corr_input_vid[5] = data.source_vertex_ids[6];
    const auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(hybrid.report["candidate_rejections"]["missing_input_face"] == 1);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 3);
    check_partition(data, hybrid);
}

TEST_CASE("hybrid validation rejects duplicate or missing source ownership")
{
    auto data = fixture();
    auto hybrid = build_prism_dominant_mesh(data, 0);
    SECTION("duplicate cell")
    {
        hybrid.cells.push_back(hybrid.cells[0]);
    }
    SECTION("missing cell")
    {
        hybrid.cells.clear();
    }
    SECTION("incorrect ownership")
    {
        hybrid.source_tet_to_cell[0] = 100;
    }
    REQUIRE_THROWS(validate_prism_dominant_mesh(data, hybrid, 0));
}

TEST_CASE("conflicting fixed interfaces terminate by retaining the original prism regions")
{
    // Both prisms touch fixed lateral tets. Input-apex transitions cannot create the
    // required prism-derived buffer here; expansion must restore the original regions.
    auto data = fixture(
        {{0, -1, 0}, {0, -1, 1}, {1, -1, 0.5}, {-1, 1, 0.5}},
        {{0, 6, 1, 4}, {0, 6, 4, 7}, {0, 3, 7, 4}, {6, 1, 4, 8}, {0, 2, 5, 9}},
        {-1, 6, -1, -1});
    data.vertex_tags[6] = 1;
    data.offset_tet_tags[6] = data.offset_tet_tags[7] = -1;
    const auto hybrid = build_prism_dominant_mesh(data, 0);
    REQUIRE(count(hybrid, HybridCellType::Pyramid) == 0);
    REQUIRE(count(hybrid, HybridCellType::Tetrahedron) == 8);
    REQUIRE(hybrid.report["compatibility_passes"].get<size_t>() >= 3);
    for (size_t tid : {0, 1, 2})
        REQUIRE(hybrid.cells[hybrid.source_tet_to_cell[tid]].type == HybridCellType::Tetrahedron);
    check_partition(data, hybrid);
}

TEST_CASE("analytic prism minima detect a fold between the 21 Jacobian samples")
{
    auto data = fixture();
    HybridCell prism{HybridCellType::Prism, {0, 1, 2, 3, 4, 5}, {0, 1, 2}, 0};
    data.vertices.row(4) << -3, 0, 1;
    data.vertices.row(5) << 0, -5, 1;
    // det(J)=(1-4t)(1-6t). All t=0,1/2,1 samples are positive, but
    // the true minimum is -1/24 at t=5/24 on each column edge.
    for (const auto& q : prism_jacobian_sample_points())
        REQUIRE(prism_jacobian(data.vertices, prism, q).determinant > 0);
    const auto minimum = minimum_prism_jacobian(data.vertices, prism);
    REQUIRE(std::abs(minimum.determinant + 1. / 24) < 1e-12);
    for (const auto& q : minimum.edge_points) REQUIRE(std::abs(q[2] - 5. / 24) < 1e-12);
}

TEST_CASE("prism determinant gradients predict finite single-corner displacements exactly")
{
    auto data = fixture();
    HybridCell prism{HybridCellType::Prism, {0, 1, 2, 3, 4, 5}, {0, 1, 2}, 0};
    data.vertices.row(3) << -.2, .3, 1.1;
    data.vertices.row(4) << .9, -.1, .8;
    data.vertices.row(5) << .1, .9, 1.3;
    const wmtk::Vector3d q(.2, .3, .7), d(.13, -.27, .09);
    const auto value = prism_jacobian(data.vertices, prism, q);
    for (size_t i = 0; i < 6; ++i) {
        const auto before = data.vertices.row(i).eval();
        data.vertices.row(i) += d.transpose();
        REQUIRE(
            std::abs(
                prism_jacobian(data.vertices, prism, q).determinant - value.determinant -
                value.gradients[i].dot(d)) < 1e-12);
        data.vertices.row(i) = before;
    }
    data.vertices.row(3) = data.vertices.row(0);
    const auto singular = prism_jacobian(data.vertices, prism, wmtk::Vector3d::Zero());
    REQUIRE(singular.determinant == 0);
    for (const auto& g : singular.gradients) REQUIRE(g.allFinite());
}

TEST_CASE("Jacobian smoothing untangles a prism while preserving its positive tet partition")
{
    auto data = fixture();
    data.vertices.row(5) << 0, .25, .25;
    const auto before = data.vertices;
    const auto connectivity = data.tetrahedra;
    const auto correspondence = data.corr_input_vid;
    JacobianSmoothingOptions options;
    options.iterations = 15;
    smooth_prism_jacobians(data, 1e-9, options);
    const auto& report = data.jacobian_smoothing_report;
    REQUIRE(report["initial"]["invalid_prisms"] == 1);
    REQUIRE(report["final"]["invalid_prisms"] == 0);
    REQUIRE(report["accepted_moves"].get<size_t>() > 0);
    REQUIRE(
        report["maximum_displacement_ratio"].get<double>() <=
        options.max_displacement_ratio + 1e-10);
    REQUIRE(data.vertices.topRows(3) == before.topRows(3));
    REQUIRE(data.tetrahedra == connectivity);
    REQUIRE(data.corr_input_vid == correspondence);
    for (const auto& t : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), 1e-9));
    const auto hybrid = build_prism_dominant_mesh(data, 1e-9);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 1);
    check_partition(data, hybrid);
}

TEST_CASE("Jacobian smoothing protects an initially valid neighboring prism")
{
    auto data =
        fixture({{1, 1, 0}, {1, 1, 1}}, {{1, 6, 2, 5}, {1, 6, 5, 7}, {1, 4, 7, 5}}, {-1, 6});
    data.vertex_tags[6] = 1;
    data.vertices.row(5) << 0, .25, .25;
    const auto candidates = prism_candidates_for_smoothing(data);
    std::vector<HybridCell> initially_valid;
    for (const auto& c : candidates)
        if (minimum_prism_jacobian(data.vertices, c).determinant > 0) initially_valid.push_back(c);
    REQUIRE(initially_valid.size() == 1);
    JacobianSmoothingOptions options;
    options.iterations = 15;
    smooth_prism_jacobians(data, 1e-9, options);
    for (const auto& c : initially_valid)
        REQUIRE(minimum_prism_jacobian(data.vertices, c).determinant > 0);
    REQUIRE(data.jacobian_smoothing_report["final"]["invalid_prisms"] == 0);
    const auto hybrid = build_prism_dominant_mesh(data, 1e-9);
    REQUIRE(count(hybrid, HybridCellType::Prism) == 2);
    check_partition(data, hybrid);
}

TEST_CASE("Jacobian smoothing leaves constrained or already valid meshes unchanged")
{
    auto data = fixture();
    JacobianSmoothingOptions options;
    options.iterations = 5;
    double floor = 0;
    SECTION("already above target") {}
    SECTION("disabled")
    {
        options.iterations = 0;
        data.vertices.row(5) << 0, .25, .25;
    }
    SECTION("unreachable volume floor is a local rejection")
    {
        floor = 1;
        data.vertices.row(5) << 0, .25, .25;
    }
    const auto before = data.vertices;
    const auto connectivity = data.tetrahedra;
    REQUIRE_NOTHROW(smooth_prism_jacobians(data, floor, options));
    REQUIRE(data.vertices == before);
    REQUIRE(data.tetrahedra == connectivity);
}
