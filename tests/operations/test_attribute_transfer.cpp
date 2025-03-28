#include <catch2/catch_test_macros.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/operations/AttributeTransferConfiguration.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/attribute_update/CastAttributeTransferStrategy.hpp>
#include <wmtk/utils/cast_attribute.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::invariants;
using namespace wmtk::simplex;


TEST_CASE("split_edge_attr_update", "[operations][split][2D]")
{
    using namespace operations;
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /
    //    7---8
    TriMesh mold = hex_plus_two_with_position(); // 0xa <- 0xa
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(mold);


    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto edge_length_handle = m.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);

    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };

    // strategy for updateing from pos_handle to edge_length_handle
    std::shared_ptr el_strategy =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_handle,
            pos_handle,
            compute_edge_length);

    {
        el_strategy->run_on_all();
    }

    auto pos_acc = m.create_const_accessor<double>(pos_handle);
    auto el_acc = m.create_const_accessor<double>(edge_length_handle);

    auto check_lengths = [&]() {
        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            // el_strategy->update(simplex::Simplex::edge(m,e));


            const Tuple v0 = e;
            const Tuple v1 = m.switch_vertex(e);

            const auto pos0 = pos_acc.const_vector_attribute(v0);
            const auto pos1 = pos_acc.const_vector_attribute(v1);

            const double len = (pos0 - pos1).norm();

            const double len2 = el_acc.const_scalar_attribute(e);

            CHECK(len == len2);
        }
    };


    check_lengths();

    EdgeSplit op(m);
    op.add_transfer_strategy(el_strategy);
    op.set_new_attribute_strategy(
        edge_length_handle,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::None); // the edge length attribute is updated
                                      // in the update strategy
    op.set_new_attribute_strategy(pos_handle);
    const Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
    bool success = !op(Simplex::edge(m, edge)).empty();
    CHECK(success);
    check_lengths();

    const Tuple edge2 = m.edge_tuple_with_vs_and_t(3, 0, 0);
    success = !op(Simplex::edge(m, edge2)).empty();
    CHECK(success);
    check_lengths();

    const Tuple edge3 = m.edge_tuple_with_vs_and_t(4, 7, 6);
    success = !op(Simplex::edge(m, edge3)).empty();
    CHECK(success);
    REQUIRE(m.is_connectivity_valid());
    check_lengths();

    const Tuple edge4 = m.edge_tuple_with_vs_and_t(4, 9, 8);
    success = !op(Simplex::edge(m, edge4)).empty();
    CHECK(success);
    check_lengths();

    const Tuple edge5 = m.edge_tuple_with_vs_and_t(5, 6, 4);
    success = !op(Simplex::edge(m, edge5)).empty();
    CHECK(success);
    check_lengths();
}

TEST_CASE("collapse_edge_new_attr", "[operations][collapse][2D]")
{
    using namespace operations;

    TriMesh mold = hex_plus_two_with_position(); // 0xa <- 0xa
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(mold);
    REQUIRE(m.is_connectivity_valid());

    // this handel already has default behaviors so lets leave it alone
    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    //
    //


    auto edge_length_handle = m.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);

    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };

    std::shared_ptr el_strategy =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_handle,
            pos_handle,
            compute_edge_length);


    auto pos_acc = m.create_const_accessor<double>(pos_handle);
    auto el_acc = m.create_const_accessor<double>(edge_length_handle);

    {
        // initialize
        auto edges = m.get_all(PrimitiveType::Edge);
        for (const auto& e : edges) {
            el_strategy->run(simplex::Simplex::edge(m, e));
        }
    }

    auto check_lengths = [&]() {
        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            const Tuple v0 = e;
            const Tuple v1 = m.switch_vertex(e);

            auto pos0 = pos_acc.const_vector_attribute(v0);
            auto pos1 = pos_acc.const_vector_attribute(v1);

            const double len = (pos0 - pos1).norm();

            const double len2 = el_acc.const_scalar_attribute(e);

            CHECK(len == len2);
        }
    };
    EdgeCollapse op(m);
    op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

    AttributeTransferConfiguration cfg;
    // spdlog::info("Adding transfers");
    // cfg.add(*el_strategy);
    // spdlog::info("Adding news");
    // cfg.add_collapse_new(edge_length_handle);
    // cfg.add_collapse_new(pos_handle);

    // cfg.apply(op);

    op.add_transfer_strategy(el_strategy);
    op.set_new_attribute_strategy(edge_length_handle);
    op.set_new_attribute_strategy(pos_handle);
    spdlog::info("starting to do sections");

    Tuple edge;

    SECTION("interior_edge")
    {
        edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
    }
    SECTION("edge_to_boundary")
    {
        edge = m.edge_tuple_with_vs_and_t(4, 0, 0);
    }
    SECTION("edge_from_boundary_allowed")
    {
        edge = m.edge_tuple_with_vs_and_t(0, 4, 0);
    }
    SECTION("boundary_edge")
    {
        edge = m.edge_tuple_with_vs_and_t(0, 1, 1);
    }

    const bool success = !op(Simplex::edge(m, edge)).empty();
    CHECK(success);
    check_lengths();
}

TEST_CASE("attribute_strategy_missing", "[operations][split]")
{
    using namespace operations;
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /
    //    7---8
    TriMesh mold = hex_plus_two_with_position(); // 0xa <- 0xa


    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(mold);

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    EdgeSplit op(m);

    const Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
    REQUIRE(m.is_valid(edge));

    // attributes without update strategy cause an exception in the operation
    std::vector<simplex::Simplex> ret;
    auto valid_gids = [&](const PrimitiveType& pt) -> std::vector<int64_t> {
        std::vector<int64_t> ret;
        const auto tups = m.get_all(pt);
        std::transform(tups.begin(), tups.end(), std::back_inserter(ret), [&](const Tuple& t) {
            return m.id(t, pt);
        });
        return ret;
    };
    const std::vector<int64_t> orig_tris = valid_gids(PrimitiveType::Triangle);

    CHECK_THROWS(ret = op(Simplex::edge(m, edge)));
    const std::vector<int64_t> new_tris = valid_gids(PrimitiveType::Triangle);
    REQUIRE(orig_tris == new_tris);
    logger().trace("{} {}", edge.as_string(), ret.size());
    REQUIRE(m.is_valid(edge));

    op.set_new_attribute_strategy(pos_handle);
    CHECK_NOTHROW(op(Simplex::edge(m, edge)));
}
TEST_CASE("attribute_update_multimesh", "[attribute_updates][multimesh]")
{
    using namespace operations;

    int N = 6;
    // disk mesh
    auto d_mesh = disk_to_individual_multimesh(N);
    // independent_mesh
    auto i_mesh = std::dynamic_pointer_cast<TriMesh>(d_mesh->get_child_meshes()[0]);

    auto d_mesh_debug = std::static_pointer_cast<DEBUG_TriMesh>(d_mesh);
    auto i_mesh_debug = std::static_pointer_cast<DEBUG_TriMesh>(i_mesh);

    auto d_pos_handle = d_mesh->register_attribute<double>("pos", PrimitiveType::Vertex, 2);
    auto i_pos_handle = i_mesh->register_attribute<double>("pos", PrimitiveType::Vertex, 2);

    auto& d_pos = d_mesh_debug->create_base_accessor(d_pos_handle.as<double>());
    REQUIRE(d_pos.reserved_size() == N + 1);
    int rows = N + 1;
    int cols = 2;
    std::vector<double> r(rows * cols);
    Eigen::MatrixXd::MapType pos(r.data(), rows, cols);
    {
        pos.row(0).setZero();
        double dtheta = 2. * M_PI / N;

        for (int j = 0; j < N; ++j) {
            double theta = dtheta * j;
            double c = std::cos(theta);
            double s = std::sin(theta);
            pos.row(j + 1) << c, s;
        }
        d_pos.set(r);
    }

    auto average = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd { return P.rowwise().mean(); };
    // strategy for updateing from pos_handle to edge_length_handle
    std::shared_ptr average_strategy_down =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            i_pos_handle,
            d_pos_handle,
            average);


    average_strategy_down->run_on_all();

    auto& i_pos = i_mesh_debug->create_base_accessor(i_pos_handle.as<double>());

    REQUIRE(i_pos.reserved_size() == 3 * N);
    for (int n = 0; n < N; ++n) {
        // d <- 0, n+1, n+2%N
        // i <- 3*n, 3*n+1, 3*n+2

        std::array<int, 3> d_indices{{0, n + 1, (n + 1) % (N) + 1}};
        std::array<int, 3> i_indices{{3 * n, 3 * n + 1, 3 * n + 2}};

        for (int j = 0; j < 3; ++j) {
            auto a = d_pos.vector_attribute(d_indices[j]);
            auto b = i_pos.vector_attribute(i_indices[j]);

            // spdlog::warn(
            //     "{}={} == {}={}",
            //     d_indices[j],
            //     fmt::join(a, ","),
            //     fmt::join(b, ","),
            //     i_indices[j]);
            CHECK(a == b);
        }
    }

    auto sum_plus_one = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P.array().rowwise().mean();
        // return (P.array() + 1.0).rowwise().sum();
    };
    // strategy for updateing from pos_handle to edge_length_handle
    std::shared_ptr sum_plus_one_strategy_up =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            d_pos_handle,
            i_pos_handle,
            sum_plus_one);

    // clear the data
    {
        pos.row(0).setZero();
        d_pos.set(r);
    }
    sum_plus_one_strategy_up->run_on_all();

    for (int n = 0; n < N; ++n) {
        // d <- 0, n+1, n+2%N
        // i <- 3*n, 3*n+1, 3*n+2

        std::array<int, 3> d_indices{{0, n + 1, (n + 1) % (N) + 1}};
        std::array<int, 3> i_indices{{3 * n, 3 * n + 1, 3 * n + 2}};

        for (int j = 0; j < 3; ++j) {
            Eigen::VectorXd a = d_pos.vector_attribute(d_indices[j]);
            Eigen::VectorXd b = i_pos.vector_attribute(i_indices[j]);
            // a = a.array() + 1.0;

            CHECK(a == b);
        }
    }

    auto valence = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return Eigen::VectorXd::Constant(P.rows(), double(P.cols()));
    };
    // strategy for updateing from pos_handle to edge_length_handle
    std::shared_ptr valence_strategy_up =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            d_pos_handle,
            i_pos_handle,
            valence);

    valence_strategy_up->run_on_all();
    {
        Eigen::VectorXd b = d_pos.vector_attribute(0);
        CHECK((b.array() == double(N)).all());
    }
    for (int n = 0; n < N; ++n) {
        // d <- 0, n+1, n+2%N

        std::array<int, 3> d_indices{{0, n + 1, (n + 1) % (N) + 1}};

        for (int j = 1; j < 3; ++j) {
            Eigen::VectorXd b = d_pos.vector_attribute(d_indices[j]);
            CHECK((b.array() == 2.0).all());
        }
    }
}

TEST_CASE("attr_cast", "[operations][attributes]")
{
    using namespace operations;
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /
    //    7---8
    TriMesh mold = hex_plus_two_with_position(); // 0xa <- 0xa
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(mold);


    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_rational_handle = m.register_attribute<wmtk::Rational>(
        "vertices_rational",
        PrimitiveType::Vertex,
        m.get_attribute_dimension(pos_handle.as<double>()));

    auto pos_handle2 = m.register_attribute<double>(
        "vertices2",
        PrimitiveType::Vertex,
        m.get_attribute_dimension(pos_handle.as<double>()));


    wmtk::operations::attribute_update::CastAttributeTransferStrategy<wmtk::Rational, double>
        caster(pos_rational_handle, pos_handle);
    wmtk::operations::attribute_update::CastAttributeTransferStrategy<double, double> caster2(
        pos_handle2,
        pos_handle);

    caster.run_on_all();
    caster2.run_on_all();

    auto rational_handle2 =
        wmtk::utils::cast_attribute<wmtk::Rational>(pos_handle, m, "vertices_rational2");

    REQUIRE(m.has_attribute<wmtk::Rational>("vertices_rational2", wmtk::PrimitiveType::Vertex));

    auto aa = m.create_const_accessor<double>(pos_handle);
    auto ba = m.create_const_accessor<wmtk::Rational>(pos_rational_handle);
    auto ca = m.create_const_accessor<double>(pos_handle2);
    auto da = m.create_const_accessor<wmtk::Rational>(rational_handle2);
    for (const auto& vtup : m.get_all(wmtk::PrimitiveType::Vertex)) {
        auto a = aa.vector_attribute(vtup);
        auto b = ba.vector_attribute(vtup);
        auto c = ca.vector_attribute(vtup);
        auto d = da.vector_attribute(vtup);

        CHECK(a == c);
        CHECK(a == b.cast<double>());
        CHECK(b.array().unaryExpr([](const auto& b) -> bool { return b.is_rounded(); }).all());
        REQUIRE(b.size() == d.size());
        for (int j = 0; j < b.size(); ++j) {
            CHECK(b(j) == d(j));
        }
    }
}
