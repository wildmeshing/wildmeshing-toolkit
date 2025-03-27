#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/isotropic_remeshing/internal/IsotropicRemeshing.hpp>
#include <wmtk/components/isotropic_remeshing/internal/IsotropicRemeshingOptions.hpp>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/Paths.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/utils/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/merkle_tree_diff.hpp>

using namespace wmtk::components::utils;

using json = nlohmann::json;
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

void print_tuple_map_iso(const DEBUG_TriMesh& parent, const DEBUG_MultiMeshManager& p_mul_manager)
{
    int64_t child_id = 0;
    for (auto& child_data : p_mul_manager.children()) {
        PrimitiveType map_ptype = child_data.mesh->top_simplex_type();
        auto parent_to_child_accessor =
            parent.create_const_accessor<int64_t>(child_data.map_handle);
        for (int64_t parent_gid = 0; parent_gid < parent.capacity(map_ptype); ++parent_gid) {
            auto parent_to_child_data = parent_to_child_accessor.const_vector_attribute(
                parent.tuple_from_id(map_ptype, parent_gid));
            auto [parent_tuple, child_tuple] =
                wmtk::multimesh::utils::vectors_to_tuples(parent_to_child_data);
        }
    }
}

TEST_CASE("smoothing_mesh", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;

    wmtk::io::Cache cache("wmtk_cache", ".");

    // input
    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 2},
        {"file", (data_dir / "bumpyDice.msh").string()},
        {"ignore_z", false},
        {"tetrahedron_attributes", json::array()}};
    wmtk::components::input(Paths(), input_component_json, cache);


    auto mesh_in = cache.read_mesh(input_component_json["name"]);

    TriMesh& m = static_cast<TriMesh&>(*mesh_in);

    wmtk::attribute::MeshAttributeHandle pos_attribute =
        m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    AttributesUpdateWithFunction op(m);
    op.set_function(VertexLaplacianSmooth(pos_attribute));

    op.add_invariant(
        std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Vertex));

    Scheduler scheduler;

    for (int i = 0; i < 3; ++i) {
        scheduler.run_operation_on_all(op);
    }

    // output
    {
        ParaviewWriter writer(
            cache.get_cache_path() / "mesh_smooth",
            "vertices",
            m,
            false,
            false,
            true,
            false);
        m.serialize(writer);
    }
}

TEST_CASE("smoothing_simple_examples", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    SECTION("hex_plus_two")
    {
        DEBUG_TriMesh mesh = wmtk::tests::hex_plus_two_with_position();

        wmtk::attribute::MeshAttributeHandle pos_attribute =
            mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        // offset interior vertex
        auto pos = mesh.create_accessor<double>(pos_attribute);
        Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};

        AttributesUpdateWithFunction op(mesh);
        op.set_function(VertexLaplacianSmooth(pos_attribute));
        op.add_invariant(
            std::make_shared<invariants::InteriorSimplexInvariant>(mesh, PrimitiveType::Vertex));

        Scheduler scheduler;
        scheduler.run_operation_on_all(op);

        v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        Eigen::Vector3d after_smooth = pos.vector_attribute(v4);
        CHECK((after_smooth - Eigen::Vector3d{1, 0, 0}).squaredNorm() == 0);
    }

    SECTION("edge_region")
    {
        DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

        wmtk::attribute::MeshAttributeHandle pos_attribute =
            mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        // offset interior vertex
        auto pos = mesh.create_accessor<double>(pos_attribute);
        Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        pos.vector_attribute(v5) = Eigen::Vector3d{1.4, -0.9, 0};

        AttributesUpdateWithFunction op(mesh);
        op.set_function(VertexLaplacianSmooth(pos_attribute));
        op.add_invariant(
            std::make_shared<invariants::InteriorSimplexInvariant>(mesh, PrimitiveType::Vertex));

        Scheduler scheduler;

        for (size_t i = 0; i < 10; ++i) {
            scheduler.run_operation_on_all(op);
            // v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            // /*Eigen::Vector3d p4_after_smooth =*/pos.vector_attribute(v4);
        }

        v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        Eigen::Vector3d p4_after_smooth = pos.vector_attribute(v4);
        CHECK((p4_after_smooth - Eigen::Vector3d{1, 0, 0}).squaredNorm() < 1e-10);

        v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        Eigen::Vector3d p5_after_smooth = pos.vector_attribute(v5);
        CHECK((p5_after_smooth - Eigen::Vector3d{2, 0, 0}).squaredNorm() < 1e-10);
    }
}

TEST_CASE("tangential_smoothing", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;

    DEBUG_TriMesh mesh = wmtk::tests::hex_plus_two_with_position();

    wmtk::attribute::MeshAttributeHandle pos_attribute =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // offset interior vertex
    auto pos = mesh.create_accessor<double>(pos_attribute);
    Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);

    Eigen::Vector3d p_init;
    SECTION("1_0_1")
    {
        p_init = Eigen::Vector3d{1, 0, 1};
    }
    SECTION("0.5_0.5_1")
    {
        p_init = Eigen::Vector3d{0.5, 0.5, 1};
    }
    SECTION("0_0_7")
    {
        p_init = Eigen::Vector3d{0, 0, 7};
    }

    pos.vector_attribute(v4) = p_init;

    AttributesUpdateWithFunction op(mesh);
    op.set_function(VertexTangentialLaplacianSmooth(pos_attribute));
    op.add_invariant(
        std::make_shared<invariants::InteriorSimplexInvariant>(mesh, PrimitiveType::Vertex));

    Scheduler scheduler;
    scheduler.run_operation_on_all(op);

    v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
    Eigen::Vector3d after_smooth = pos.vector_attribute(v4);
    Eigen::Vector3d target = Eigen::Vector3d{1, 0, p_init[2]};
    CHECK((after_smooth - target).squaredNorm() == 0);
}

TEST_CASE("tangential_smoothing_boundary", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    DEBUG_TriMesh mesh = wmtk::tests::hex_plus_two_with_position();

    wmtk::attribute::MeshAttributeHandle pos_attribute =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // offset interior vertex
    auto pos = mesh.create_accessor<double>(pos_attribute);
    Tuple v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);

    Eigen::Vector3d p_init;
    SECTION("1.7_1.1_0")
    {
        p_init = Eigen::Vector3d{1.7, 1.1, 0};
    }
    SECTION("2.2_2_0")
    {
        p_init = Eigen::Vector3d{2.2, 2, 0};
    }
    SECTION("2.2_2_5")
    {
        p_init = Eigen::Vector3d{2.2, 2, 5};
    }

    pos.vector_attribute(v1) = p_init;

    AttributesUpdateWithFunction op(mesh);
    op.set_function(VertexTangentialLaplacianSmooth(pos_attribute));

    const bool success = !op(Simplex::vertex(mesh, v1)).empty();
    REQUIRE(success);

    v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);
    Eigen::Vector3d after_smooth = pos.vector_attribute(v1);
    CHECK((after_smooth - Eigen::Vector3d{1.5, p_init[1], p_init[2]}).squaredNorm() == 0);
}

TEST_CASE("split_long_edges", "[components][isotropic_remeshing][split][2D]")
{
    using namespace operations;

    // This test does not fully work yet

    DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

    wmtk::attribute::MeshAttributeHandle pos_attribute =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    EdgeSplit op(mesh);
    op.set_new_attribute_strategy(
        pos_attribute,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::Mean);

    {
        auto pos = mesh.create_accessor<double>(pos_attribute);
        const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        // reposition interior vertices
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        pos.vector_attribute(v5) = Eigen::Vector3d{2.4, -0.9, 0};
        //           << std::endl;
    }

    double min_split_length_squared = -1;

    SECTION("6.4")
    {
        min_split_length_squared = 6.4;
        op.add_invariant(std::make_shared<MinEdgeLengthInvariant>(
            mesh,
            pos_attribute.as<double>(),
            min_split_length_squared));

        Scheduler scheduler;

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(op);

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        CHECK(n_iterations == 1);
        REQUIRE(n_vertices == 11);

        // check position of new vertex
        auto pos = mesh.create_accessor<double>(pos_attribute);
        const Tuple v10 = mesh.tuple_from_id(PrimitiveType::Vertex, 10);
        CHECK((pos.vector_attribute(v10) - Eigen::Vector3d{1.5, 0, 0}).squaredNorm() == 0);
    }
    SECTION("3.5")
    {
        min_split_length_squared = 3.5;
        op.add_invariant(std::make_shared<MinEdgeLengthInvariant>(
            mesh,
            pos_attribute.as<double>(),
            min_split_length_squared));

        Scheduler scheduler;

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(op);

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        CHECK(n_iterations < 5);
        CHECK(n_vertices == 15);
    }

    // check edge lengths
    auto pos = mesh.create_accessor<double>(pos_attribute);
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        const Eigen::Vector3d p0 = pos.vector_attribute(e);
        const Eigen::Vector3d p1 = pos.vector_attribute(mesh.switch_vertex(e));
        const double l_squared = (p1 - p0).squaredNorm();
        CHECK(l_squared < min_split_length_squared);
    }
}

TEST_CASE("collapse_short_edges", "[components][isotropic_remeshing][collapse][2D]")
{
    using namespace operations;

    DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

    auto pos_attribute = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);


    EdgeCollapse op(mesh);
    op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(mesh));
    op.set_new_attribute_strategy(pos_attribute, CollapseBasicStrategy::Mean);

    SECTION("interior")
    {
        {
            auto pos = mesh.create_accessor<double>(pos_attribute);
            const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
            // reposition interior vertices
            pos.vector_attribute(v4) = Eigen::Vector3d{1.4, 0, 0};
            pos.vector_attribute(v5) = Eigen::Vector3d{1.6, 0, 0};
        }

        op.add_invariant(
            std::make_shared<MaxEdgeLengthInvariant>(mesh, pos_attribute.as<double>(), 0.1));

        Scheduler scheduler;

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(op);

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        REQUIRE(n_iterations == 1);
        REQUIRE(n_vertices == 9);

        // CHECK_THROWS(mesh.tuple_from_id(PrimitiveType::Vertex, 4));
        const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
#if defined(WMTK_ENABLE_HASH_UPDATE)
        REQUIRE(mesh.is_valid_with_hash(v5));
#else
        REQUIRE(mesh.is_valid(v5));
#endif

        auto pos = mesh.create_accessor<double>(pos_attribute);
        Eigen::Vector3d p5 = pos.vector_attribute(v5);
        CHECK((p5 - Eigen::Vector3d{1.5, 0, 0}).squaredNorm() == 0);
    }
    SECTION("towards_boundary_true")
    {
        {
            auto pos = mesh.create_accessor<double>(pos_attribute);
            const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            // reposition vertex
            pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        }

        // set collapse towards boundary
        {
            auto tmp = std::make_shared<CollapseNewAttributeStrategy<double>>(pos_attribute);
            tmp->set_strategy(CollapseBasicStrategy::Mean);
            tmp->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            op.set_new_attribute_strategy(pos_attribute, tmp);
        }

        op.add_invariant(
            std::make_shared<MaxEdgeLengthInvariant>(mesh, pos_attribute.as<double>(), 0.1));

        Scheduler scheduler;

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(op);

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        REQUIRE(n_iterations == 1);
        REQUIRE(n_vertices == 9);

        const Tuple v0 = mesh.tuple_from_id(PrimitiveType::Vertex, 0);
#if defined(WMTK_ENABLE_HASH_UPDATE)
        REQUIRE(mesh.is_valid_with_hash(v0));
#else
        REQUIRE(mesh.is_valid(v0));
#endif

        auto pos = mesh.create_accessor<double>(pos_attribute);
        Eigen::Vector3d p0 = pos.vector_attribute(v0);
        CHECK((p0 - Eigen::Vector3d{0.5, 1, 0}).squaredNorm() == 0);
    }
    SECTION("towards_boundary_false")
    {
        {
            auto pos = mesh.create_accessor<double>(pos_attribute);
            const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            // reposition vertex
            pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        }

        op.add_invariant(
            std::make_shared<MaxEdgeLengthInvariant>(mesh, pos_attribute.as<double>(), 0.1));
        // op_settings.collapse_towards_boundary = false; <-- invariant missing


        Scheduler scheduler;

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(op);

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        REQUIRE(n_iterations == 1);
        REQUIRE(n_vertices == 9);

        const Tuple v0 = mesh.tuple_from_id(PrimitiveType::Vertex, 0);
#if defined(WMTK_ENABLE_HASH_UPDATE)
        REQUIRE(mesh.is_valid_with_hash(v0));
#else
        REQUIRE(mesh.is_valid(v0));
#endif

        auto pos = mesh.create_accessor<double>(pos_attribute);
        Eigen::Vector3d p0 = pos.vector_attribute(v0);
        CHECK((p0 - Eigen::Vector3d{0.55, 0.95, 0}).squaredNorm() == 0);
    }
    SECTION("collapse_boundary_true")
    {
        {
            auto pos = mesh.create_accessor<double>(pos_attribute);
            const Tuple v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);
            // reposition vertex
            pos.vector_attribute(v1) = Eigen::Vector3d{0.6, 1, 0};
        }

        op.add_invariant(
            std::make_shared<MaxEdgeLengthInvariant>(mesh, pos_attribute.as<double>(), 0.1));

        Scheduler scheduler;

        scheduler.run_operation_on_all(op);

        REQUIRE(mesh.get_all(PrimitiveType::Vertex).size() == 9);

        const Tuple v0 = mesh.tuple_from_id(PrimitiveType::Vertex, 0);
#if defined(WMTK_ENABLE_HASH_UPDATE)
        REQUIRE(mesh.is_valid_with_hash(v0));
#else
        REQUIRE(mesh.is_valid(v0));
#endif

        auto pos = mesh.create_accessor<double>(pos_attribute);
        Eigen::Vector3d p0 = pos.vector_attribute(v0);
        CHECK((p0 - Eigen::Vector3d{0.55, 1, 0}).squaredNorm() == 0);
    }
    SECTION("collapse_boundary_false")
    {
        {
            auto pos = mesh.create_accessor<double>(pos_attribute);
            const Tuple v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);
            // reposition vertex
            pos.vector_attribute(v1) = Eigen::Vector3d{0.6, 1, 0};
        }

        op.add_invariant(
            std::make_shared<MaxEdgeLengthInvariant>(mesh, pos_attribute.as<double>(), 0.1));
        op.add_invariant(
            std::make_shared<invariants::InteriorSimplexInvariant>(mesh, PrimitiveType::Edge));

        Scheduler scheduler;

        scheduler.run_operation_on_all(op);

        REQUIRE(mesh.get_all(PrimitiveType::Vertex).size() == 10);
    }
}

TEST_CASE("swap_edge_for_valence", "[components][isotropic_remeshing][swap][2D]")
{
    using namespace operations;

    DEBUG_TriMesh mesh = wmtk::tests::embedded_diamond();

    composite::TriEdgeSwap op(mesh);
    op.add_invariant(
        std::make_shared<invariants::InteriorSimplexInvariant>(mesh, PrimitiveType::Edge));
    op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(mesh));

    Tuple swap_edge = mesh.edge_tuple_with_vs_and_t(6, 7, 5);

    auto vertex_one_ring = [](TriMesh& m, const Tuple& t) {
        return simplex::link(m, simplex::Simplex::vertex(m, t))
            .simplex_vector(PrimitiveType::Vertex);
    };

    SECTION("single_op_fail")
    {
        op.add_invariant(std::make_shared<invariants::ValenceImprovementInvariant>(mesh));
        CHECK(op(Simplex::edge(mesh, swap_edge)).empty());
    }
    SECTION("swap_success")
    {
        // swap edge to create inbalence in valence
        {
            const auto ret = op(Simplex::edge(mesh, swap_edge));
            REQUIRE(!ret.empty());
            const Tuple& return_tuple = ret[0].tuple();
            swap_edge = return_tuple;
            long id0 = mesh.id_vertex(swap_edge);
            long id1 = mesh.id_vertex(mesh.switch_vertex(swap_edge));

            // check valence
            const Tuple v3 = mesh.tuple_from_id(PrimitiveType::Vertex, 3);
            const Tuple v6 = mesh.tuple_from_id(PrimitiveType::Vertex, 6);
            const Tuple v7 = mesh.tuple_from_id(PrimitiveType::Vertex, 7);
            const Tuple v10 = mesh.tuple_from_id(PrimitiveType::Vertex, 10);
            CHECK(vertex_one_ring(mesh, v3).size() == 7);
            CHECK(vertex_one_ring(mesh, v10).size() == 7);
            CHECK(vertex_one_ring(mesh, v6).size() == 5);
            CHECK(vertex_one_ring(mesh, v7).size() == 5);
        }

        op.add_invariant(std::make_shared<invariants::ValenceImprovementInvariant>(mesh));


        SECTION("single_op")
        {
            const auto ret = op(Simplex::edge(mesh, swap_edge));
            REQUIRE(!ret.empty());
            const Tuple& return_tuple = ret[0].tuple();
            swap_edge = return_tuple;
            CHECK(mesh.id(Simplex::vertex(mesh, swap_edge)) == 7);
            CHECK(mesh.id(Simplex::vertex(mesh, mesh.switch_vertex(swap_edge))) == 6);
        }
        SECTION("with_scheduler")
        {
            Scheduler scheduler;
            scheduler.run_operation_on_all(op);
        }


        // check valence
        {
            const Tuple v3 = mesh.tuple_from_id(PrimitiveType::Vertex, 3);
            const Tuple v6 = mesh.tuple_from_id(PrimitiveType::Vertex, 6);
            const Tuple v7 = mesh.tuple_from_id(PrimitiveType::Vertex, 7);
            const Tuple v10 = mesh.tuple_from_id(PrimitiveType::Vertex, 10);
            CHECK(vertex_one_ring(mesh, v3).size() == 6);
            CHECK(vertex_one_ring(mesh, v10).size() == 6);
            CHECK(vertex_one_ring(mesh, v6).size() == 6);
            CHECK(vertex_one_ring(mesh, v7).size() == 6);
        }
    }
    SECTION("swap_fail")
    {
        const Tuple e = mesh.edge_tuple_with_vs_and_t(6, 7, 5);
        op.add_invariant(std::make_shared<invariants::ValenceImprovementInvariant>(mesh));
        const bool success = !op(Simplex::edge(mesh, e)).empty();
        CHECK(!success);
    }
}

TEST_CASE("component_isotropic_remeshing", "[components][isotropic_remeshing][2D]")
{
    io::Cache cache("wmtk_cache", ".");

    if constexpr (true) {
        const std::filesystem::path input_file = data_dir / "small.msh";
        json input_component_json = {
            {"name", "input_mesh"},
            {"file", input_file.string()},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()}};
        REQUIRE_NOTHROW(wmtk::components::input(Paths(), input_component_json, cache));
    }

    if constexpr (true) {
        json mesh_isotropic_remeshing_json = R"({
        "input": "input_mesh",
        "output": "output_mesh",
        "attributes": {"position": "vertices", "inversion_position": [], "other_positions": [], "update_other_positions": false},
        "pass_through": [],
        "iterations": 1,
        "length_abs": 0.003,
        "length_rel": -1,
        "lock_boundary": true,
        "use_for_periodic": false,
        "dont_disable_split": false
    })"_json;
        REQUIRE_NOTHROW(
            wmtk::components::isotropic_remeshing(Paths(), mesh_isotropic_remeshing_json, cache));
    }
    if constexpr (true) {
        json component_json = R"({
            "input": "output_mesh",
            "attributes": {"position": "vertices"},
            "file": "bunny_isotropic_remeshing"
        })"_json;

        CHECK_NOTHROW(wmtk::components::output(Paths(), component_json, cache));

        // auto mesh_in = cache.read_mesh("output_mesh");
        // TriMesh& m = static_cast<TriMesh&>(*mesh_in);
        // ParaviewWriter writer(
        //    cache.get_cache_path() / "isotropic_remeshing_output",
        //    "vertices",
        //    m,
        //    false,
        //    false,
        //    true,
        //    false);
        // m.serialize(writer);
    }
}

TEST_CASE("remeshing_tetrahedron", "[components][isotropic_remeshing][2D]")
{
    using namespace wmtk::components::internal;

    io::Cache cache("wmtk_cache", ".");

    // input
    DEBUG_TriMesh mesh = tetrahedron_with_position();

    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    pass_through_attributes.emplace_back(
        mesh.register_attribute<int64_t>("dummy_tag", PrimitiveType::Triangle, 1));

    {
        auto acc = mesh.create_accessor<int64_t>(pass_through_attributes[0]);
        acc.scalar_attribute(mesh.face_tuple_from_vids(0, 1, 2)) = 1;
        acc.scalar_attribute(mesh.face_tuple_from_vids(0, 1, 3)) = 1;
    }


    CHECK_NOTHROW(wmtk::components::internal::isotropic_remeshing(
        pos_handle,
        pass_through_attributes,
        0.1,
        true,
        false,
        false,
        false,
        10));

    {
        ParaviewWriter writer(
            cache.get_cache_path() / "tet_remeshing",
            "vertices",
            mesh,
            false,
            false,
            true,
            false);
        mesh.serialize(writer);
    }
}

TEST_CASE("remeshing_with_boundary", "[components][isotropic_remeshing][2D]")
{
    using namespace wmtk::components::internal;

    io::Cache cache("wmtk_cache", ".");

    // input
    TriMesh mesh = edge_region_with_position();
    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;

    SECTION("lock_boundary_false")
    {
        wmtk::components::internal::isotropic_remeshing(
            pos_handle,
            pass_through_attributes,
            0.5,
            false,
            false,
            false,
            false,
            5);

        size_t n_boundary_edges = 0;
        for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
            if (mesh.is_boundary_edge(e)) {
                ++n_boundary_edges;
            }
        }
        CHECK(n_boundary_edges > 8);

        {
            ParaviewWriter writer(
                cache.get_cache_path() / "w_bd_remeshing_lock_false",
                "vertices",
                mesh,
                false,
                false,
                true,
                false);
            mesh.serialize(writer);
        }
    }

    SECTION("lock_boundary_true")
    {
        wmtk::components::internal::isotropic_remeshing(
            pos_handle,
            pass_through_attributes,
            0.5,
            true,
            false,
            false,
            false,
            5);


        size_t n_boundary_edges = 0;
        for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
            if (mesh.is_boundary_edge(e)) {
                ++n_boundary_edges;
            }
        }
        CHECK(n_boundary_edges == 8);

        {
            ParaviewWriter writer(
                cache.get_cache_path() / "w_bd_remeshing_lock_true",
                "vertices",
                mesh,
                false,
                false,
                true,
                false);
            mesh.serialize(writer);
        }
    }
}

TEST_CASE("remeshing_preserve_topology", "[components][isotropic_remeshing][2D][.]")
{
    using namespace wmtk::components::internal;

    // input
    DEBUG_TriMesh mesh = edge_region_with_position();
    // DEBUG_TriMesh mesh = hex_plus_two_with_position();

    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;

    auto tag_handle = mesh.register_attribute<int64_t>("is_boundary", wmtk::PrimitiveType::Edge, 1);
    auto tag_accessor = mesh.create_accessor<int64_t>(tag_handle);
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(e)) {
            tag_accessor.scalar_attribute(e) = 1;
        } else {
            tag_accessor.scalar_attribute(e) = 0;
        }
    }
    std::shared_ptr<Mesh> child_ptr =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            mesh,
            "is_boundary",
            1,
            PrimitiveType::Edge);

    REQUIRE(mesh.get_child_meshes().size() == 1);
    mesh.multi_mesh_manager().check_map_valid(mesh);
    const auto& child_mesh = *child_ptr;
    CHECK(child_mesh.get_all(PrimitiveType::Edge).size() == 8);
    CHECK(child_mesh.get_all(PrimitiveType::Vertex).size() == 8);


    wmtk::components::internal::isotropic_remeshing(
        pos_handle,
        pass_through_attributes,
        0.5,
        /*lock_boundary*/ false,
        true,
        true,
        false,
        5);

    REQUIRE(mesh.is_connectivity_valid());
    mesh.multi_mesh_manager().check_map_valid(mesh);


    size_t n_boundary_edges = 0;
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(e)) {
            ++n_boundary_edges;
        }
    }
    // CHECK(n_boundary_edges > 8);

    // output
    {
        ParaviewWriter writer("remeshing_test", "vertices", mesh, true, true, true, false);
        mesh.serialize(writer);
    }
}

TEST_CASE("remeshing_preserve_topology_realmesh", "[components][isotropic_remeshing][2D][.]")
{
    using namespace wmtk::components::internal;
    using namespace operations;

    wmtk::io::Cache cache("wmtk_cache", ".");

    // input
    // TODO: What is the default attribute for "vertices". From the reader it seems to be
    // "vertices". need change "vertices" to "vertices" isotropic_remeshing.hpp
    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 2},
        {"file", (data_dir / "circle.msh").string()},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), input_component_json, cache);

    auto m = cache.read_mesh(input_component_json["name"]);
    tests::DEBUG_TriMesh& mesh = static_cast<tests::DEBUG_TriMesh&>(*m);

    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;

    auto tag_handle = mesh.register_attribute<int64_t>("is_boundary", wmtk::PrimitiveType::Edge, 1);
    auto tag_accessor = mesh.create_accessor<int64_t>(tag_handle);
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(e)) {
            tag_accessor.scalar_attribute(e) = 1;
        } else {
            tag_accessor.scalar_attribute(e) = 0;
        }
    }
    std::shared_ptr<Mesh> child_ptr =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            mesh,
            "is_boundary",
            1,
            PrimitiveType::Edge);

    REQUIRE(mesh.get_child_meshes().size() == 1);
    // mesh.multi_mesh_manager().check_map_valid(mesh);
    // const auto& child_mesh = *child_ptr;


    // IsotropicRemeshing isotropicRemeshing(mesh, 0.5, false, false, false);

    for (int i = 0; i < 25; i++) {
        wmtk::components::internal::isotropic_remeshing(
            pos_handle,
            pass_through_attributes,
            0.05,
            false,
            true,
            false,
            false,
            1);
        REQUIRE(mesh.is_connectivity_valid());
        mesh.multi_mesh_manager().check_map_valid(mesh);
    }


    auto child_vertex_handle =
        child_ptr->register_attribute<double>("vertices", wmtk::PrimitiveType::Vertex, 3);
    auto child_vertex_accessor = child_ptr->create_accessor<int64_t>(child_vertex_handle);

    auto parent_vertex_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto parent_vertex_accessor = mesh.create_accessor<int64_t>(parent_vertex_handle);

    for (const auto& v : child_ptr->get_all(PrimitiveType::Vertex)) {
        auto parent_v = child_ptr->map_to_root_tuple(Simplex(*child_ptr, PrimitiveType::Vertex, v));
        child_vertex_accessor.vector_attribute(v) =
            parent_vertex_accessor.vector_attribute(parent_v);
    }

    // output
    {
        ParaviewWriter writer(
            cache.get_cache_path() / "remeshing_test_circle_final",
            "vertices",
            mesh,
            true,
            true,
            true,
            false);
        mesh.serialize(writer);

        ParaviewWriter writer2(
            cache.get_cache_path() / "remeshing_test_circle_child_mesh_final",
            "vertices",
            *child_ptr,
            true,
            true,
            false,
            false);
        child_ptr->serialize(writer2);
    }
}

TEST_CASE("remeshing_realmesh", "[components][isotropic_remeshing][2D][.]")
{
    using namespace wmtk::components::internal;
    using namespace operations;

    wmtk::io::Cache cache("wmtk_cache", ".");

    // input
    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 2},
        {"file", (data_dir / "circle.msh").string()},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), input_component_json, cache);

    auto m = cache.read_mesh(input_component_json["name"]);
    TriMesh& mesh = static_cast<TriMesh&>(*m);

    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;

    // auto tag_handle = mesh.register_attribute<int64_t>("is_boundary", wmtk::PrimitiveType::Edge,
    // 1); auto tag_accessor = mesh.create_accessor<int64_t>(tag_handle); for (const Tuple& e :
    // mesh.get_all(PrimitiveType::Edge)) {
    //     if (mesh.is_boundary_edge(e)) {
    //         tag_accessor.scalar_attribute(e) = 1;
    //     } else {
    //         tag_accessor.scalar_attribute(e) = 0;
    //     }
    // }
    // std::shared_ptr<Mesh> child_ptr =
    //     wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
    //         mesh,
    //         "is_boundary",
    //         1,
    //         PrimitiveType::Edge);

    // REQUIRE(mesh.get_child_meshes().size() == 1);
    // mesh.multi_mesh_manager().check_map_valid(mesh);
    // const auto& child_mesh = *child_ptr;

    wmtk::components::internal::isotropic_remeshing(
        pos_handle,
        pass_through_attributes,
        0.5,
        false,
        true,
        false,
        false,
        25);

    REQUIRE(mesh.is_connectivity_valid());
    // mesh.multi_mesh_manager().check_map_valid(mesh);

    // output
    {
        ParaviewWriter writer(
            cache.get_cache_path() / "remeshing_test_circle_no_nultimesh",
            "vertices",
            mesh,
            true,
            true,
            true,
            false);
        mesh.serialize(writer);
    }
}
