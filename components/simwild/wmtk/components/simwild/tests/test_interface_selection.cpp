#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/polyfem_helpers/InterfaceSelection.hpp>

#include <mshio/mshio.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace wmtk::components::simwild::polyfem_helpers;

using Edges = std::vector<std::array<int64_t, 2>>;

namespace {

namespace fs = std::filesystem;

/// One physical group of the fixture: its name and its triangles, as 1-based node tags.
using Group = std::pair<std::string, std::vector<std::array<size_t, 3>>>;

/// A 2D physical-groups .msh in the layout pysimwild's conftest writes its fixtures in
/// (`_write_groups_msh`, mirrored by `groups_msh` in test_polyfem_in_process.cpp): the nodes
/// 1..n on the first entity, then one entity and one physical group per group, in order. Binary
/// 4.1, so the coordinates are stored exactly.
mshio::MshSpec groups_msh_2d(
    const std::vector<std::array<double, 3>>& coords,
    const std::vector<Group>& groups)
{
    mshio::MshSpec spec;
    spec.mesh_format.version = "4.1";
    spec.mesh_format.file_type = 1;
    spec.mesh_format.data_size = sizeof(size_t);

    mshio::NodeBlock nodes;
    nodes.entity_dim = 2;
    nodes.entity_tag = 1;
    nodes.num_nodes_in_block = coords.size();
    for (size_t i = 0; i < coords.size(); ++i) {
        nodes.tags.push_back(i + 1);
        nodes.data.insert(nodes.data.end(), coords[i].begin(), coords[i].end());
    }
    spec.nodes.num_entity_blocks = 1;
    spec.nodes.num_nodes = coords.size();
    spec.nodes.min_node_tag = 1;
    spec.nodes.max_node_tag = coords.size();
    spec.nodes.entity_blocks.push_back(std::move(nodes));

    size_t next_id = 1;
    for (size_t g = 0; g < groups.size(); ++g) {
        const int tag = int(g) + 1;
        spec.physical_groups.push_back({2, tag, groups[g].first});
        spec.entities.surfaces.push_back({tag, 0, 0, 0, 0, 0, 0, {tag}, {}});
        mshio::ElementBlock block;
        block.entity_dim = 2;
        block.entity_tag = tag;
        block.element_type = 2; // gmsh element type 2 = triangle
        block.num_elements_in_block = groups[g].second.size();
        for (const auto& cell : groups[g].second) {
            block.data.push_back(next_id++);
            block.data.insert(block.data.end(), cell.begin(), cell.end());
        }
        spec.elements.entity_blocks.push_back(std::move(block));
    }
    spec.elements.num_entity_blocks = spec.elements.entity_blocks.size();
    spec.elements.num_elements = next_id - 1;
    spec.elements.min_element_tag = 1;
    spec.elements.max_element_tag = next_id - 1;
    return spec;
}

/**
 * @brief The fixture behind the weight tests, written once and returned by path.
 *
 * A 2x2 block of unit squares on a 3x3 lattice, two triangles each, three materials: tag_0 is the
 * square at the origin, tag_1 the two squares beside it, ambient the far one. So the interface
 * tag_0|tag_1 is the two edges 2-5 and 4-5, the interface tag_1|ambient the two edges 5-6 and
 * 5-8, and they MEET at node 5 -- the shared node the per-node weight rule has to resolve. The
 * centre node is pulled off the lattice so that no cotangent weight comes out of a symmetry.
 *
 * The interface nodes are the gmsh tags 2, 4, 5, 6, 8, so the constraint's local2global (0-based,
 * tag - 1 on this contiguous mesh) is 1, 3, 4, 5, 7, and rows 0, 1, 2 are the tag_0|tag_1 nodes.
 */
fs::path two_interfaces_2d()
{
    const auto nid = [](const size_t i, const size_t j) { return 1 + i + 3 * j; };
    std::vector<std::array<double, 3>> coords;
    for (size_t j = 0; j < 3; ++j) {
        for (size_t i = 0; i < 3; ++i) {
            coords.push_back({double(i), double(j), 0.0});
        }
    }
    coords[nid(1, 1) - 1] = {1.1, 0.9, 0.0};

    std::vector<Group> groups{{"ambient", {}}, {"tag_0", {}}, {"tag_1", {}}};
    for (size_t j = 0; j < 2; ++j) {
        for (size_t i = 0; i < 2; ++i) {
            const size_t a = nid(i, j), b = nid(i + 1, j), c = nid(i, j + 1), d = nid(i + 1, j + 1);
            const size_t group = i == 0 && j == 0 ? 1 : (i == 1 && j == 1 ? 0 : 2);
            groups[group].second.push_back({a, b, d});
            groups[group].second.push_back({a, d, c});
        }
    }

    const fs::path root = fs::temp_directory_path() / "wmtk_polyfem_helpers_interface_weights";
    fs::create_directories(root);
    const fs::path path = root / "two_interfaces.msh";
    mshio::save_msh(path.string(), groups_msh_2d(coords, groups));
    return path;
}

/// The two interfaces of that fixture, in the order that gives them the selection ids 1 and 2
/// (assign_selection_ids numbers them in first-appearance order).
const std::vector<Selection> two_interfaces{
    Selection{"tag_0", std::string("tag_1"), std::nullopt, std::nullopt},
    Selection{"tag_1", std::string("ambient"), std::nullopt, std::nullopt}};

/// One call of the constraint builder on the fixture, with the smoothing operation's settings:
/// the geometric Laplacian, normalized, positions mode so `b` is not all zeros.
InterfaceConstraint build(const fs::path& mesh, const std::map<int64_t, double>& row_factors)
{
    return make_interface_constraint(
        TaggedMesh(mesh.string()),
        two_interfaces,
        /*use_graph=*/false,
        /*normalize=*/true,
        /*scale=*/1e-3,
        /*smooth_positions=*/true,
        row_factors);
}

} // namespace

// The 2D collision proxy's 'l' lines come out of orient_edge_loops_2d, so a loop that comes back
// wound the wrong way is a proxy whose normals point into the material. Every expectation below
// was read off the Python it mirrors (constraints._orient_edge_loops_2d) on the same input.
TEST_CASE("orient_edge_loops_2d keeps the direction the input edges agree on", "[polyfem_helpers]")
{
    // A square, its four edges all wound the same way: the loop is returned unchanged, starting
    // at the minimum vertex (the deterministic seed).
    const Edges ccw{{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    CHECK(orient_edge_loops_2d(ccw) == ccw);

    // The same square wound the other way: the reconstruction always walks 0 -> 1 -> 2 -> 3, so
    // the loop is reversed to put the input's direction back.
    const Edges cw{{1, 0}, {2, 1}, {3, 2}, {0, 3}};
    const Edges cw_expected{{3, 2}, {2, 1}, {1, 0}, {0, 3}};
    CHECK(orient_edge_loops_2d(cw) == cw_expected);
}

TEST_CASE("orient_edge_loops_2d passes non-loop components through", "[polyfem_helpers]")
{
    // An open chain is not a loop (its ends have degree 1), so it keeps its original order and
    // orientation. This is the common 2D case: an interface that ends on the domain boundary.
    const Edges chain{{0, 1}, {1, 2}};
    CHECK(orient_edge_loops_2d(chain) == chain);

    // Components are emitted in order of first vertex appearance, and a non-loop component's
    // edges in their original index order -- the chain (seeded at 5) before the loop.
    const Edges mixed{{5, 6}, {0, 1}, {1, 2}, {2, 3}, {3, 0}, {6, 7}};
    const Edges mixed_expected{{5, 6}, {6, 7}, {0, 1}, {1, 2}, {2, 3}, {3, 0}};
    CHECK(orient_edge_loops_2d(mixed) == mixed_expected);
}

TEST_CASE("orient_edge_loops_2d rejects a contradictory loop", "[polyfem_helpers]")
{
    // Three edges one way and one the other: the same interface selected from both sides, so no
    // single orientation is correct for both bodies. Guessing here would silently flip a normal.
    const Edges contradictory{{0, 1}, {1, 2}, {2, 3}, {0, 3}};
    CHECK_THROWS(orient_edge_loops_2d(contradictory));
}

// A run that asks for no per-interface weight must produce the constraint it produced before the
// weight existed: the writer skips the scaling pass rather than multiplying every row by one, so
// the check is equality of the bits, not a tolerance.
TEST_CASE("an empty weight map leaves the Laplacian constraint untouched", "[polyfem_helpers]")
{
    const fs::path mesh = two_interfaces_2d();
    // The call the operations made before the weight existed: no map argument at all.
    const ConstraintHdf5 base = make_interface_constraint(
                                    TaggedMesh(mesh.string()),
                                    two_interfaces,
                                    /*use_graph=*/false,
                                    /*normalize=*/true,
                                    /*scale=*/1e-3,
                                    /*smooth_positions=*/true)
                                    .laplacian;
    const ConstraintHdf5 with_empty_map = build(mesh, {}).laplacian;

    CHECK(with_empty_map.local2global == base.local2global);
    CHECK(with_empty_map.a.rows == base.a.rows);
    CHECK(with_empty_map.a.cols == base.a.cols);
    CHECK(with_empty_map.a.values == base.a.values);
    CHECK(with_empty_map.b == base.b);
}

// The weight reaches polyfem as a row factor, because polyfem applies one global
// `weight_laplacian` W: a selection asking for 4 W scales its nodes' rows of A and of b by
// sqrt(4) = 2. Doubling is exact in binary floating point, so these too are equalities.
TEST_CASE("a per-interface weight scales exactly its own rows", "[polyfem_helpers]")
{
    const fs::path mesh = two_interfaces_2d();
    const InterfaceConstraint base = build(mesh, {});
    REQUIRE(base.laplacian.local2global == std::vector<int32_t>{1, 3, 4, 5, 7});

    // The fixture's premise, checked rather than assumed: two selected interfaces of two edges
    // each, and every one of those edges ends at local row 2 (the centre node), so that node is
    // on both interfaces and the max rule is what decides its factor.
    std::vector<std::vector<int64_t>> tag_rows = base.collision_body_ids;
    std::sort(tag_rows.begin(), tag_rows.end()); // the edge order is the 2D loop pass's, not the
                                                 // selections': only the counts are the premise
    REQUIRE(tag_rows == std::vector<std::vector<int64_t>>{{1}, {1}, {2}, {2}});
    for (const auto& edge : base.collision_mesh.edges) {
        REQUIRE((edge[0] == 2 || edge[1] == 2));
    }

    // Selection 1 is tag_0|tag_1; selection 2 (tag_1|ambient) carries no weight and stays at 1.
    const InterfaceConstraint weighted = build(mesh, {{1, 2.0}});
    REQUIRE(weighted.laplacian.a.rows == base.laplacian.a.rows);
    REQUIRE(weighted.laplacian.b.size() == base.laplacian.b.size());
    CHECK(weighted.laplacian.a.values != base.laplacian.a.values); // the factor did reach the rows

    // Node 4 (local row 2) lies on both interfaces and takes the larger of the two factors.
    const std::set<int32_t> weighted_nodes{1, 3, 4};
    const auto factor_of_row = [&](const int32_t row) {
        return weighted_nodes.count(base.laplacian.local2global[size_t(row)]) != 0 ? 2.0 : 1.0;
    };

    for (size_t k = 0; k < base.laplacian.a.values.size(); ++k) {
        CHECK(
            weighted.laplacian.a.values[k] ==
            base.laplacian.a.values[k] * factor_of_row(base.laplacian.a.rows[k]));
    }
    const int64_t dim = base.laplacian.b_cols;
    for (int64_t i = 0; i < base.laplacian.b_rows; ++i) {
        for (int64_t k = 0; k < dim; ++k) {
            const size_t e = size_t(i * dim + k);
            CHECK(weighted.laplacian.b[e] == base.laplacian.b[e] * factor_of_row(int32_t(i)));
        }
    }

    // Nothing else moves: the fitting constraint has its own weight and is not scaled.
    CHECK(weighted.fitting.a.values == base.fitting.a.values);
    CHECK(weighted.fitting.b == base.fitting.b);
}
