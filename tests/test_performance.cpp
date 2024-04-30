#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <polysolve/Utils.hpp>

using json = nlohmann::json;

using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("Read_only", "[performance][.]")
{
    const std::filesystem::path meshfile =
        data_dir / "adaptive_tessellation_test" / "after_smooth_uv.msh";

    auto mesh_in = wmtk::read_mesh(meshfile, true);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_acc = m.create_accessor<double>(pos_handle);

    double sum = 0;

    const auto vertices = m.get_all(PrimitiveType::Vertex);
    for (int i = 0; i < 10000; ++i) {
        for (const Tuple& t : vertices) {
            sum += pos_acc.const_vector_attribute(t)[0];
        }
        for (const Tuple& t : vertices) {
            sum += pos_acc.const_vector_attribute(t)[1];
        }
    }
    std::cout << "sum = " << sum << std::endl;
}

TEST_CASE("accessor_performance", "[accessor][performance][.]")
{
    const std::filesystem::path meshfile = data_dir / "armadillo.msh";

    logger().set_level(spdlog::level::trace);

    auto mesh_in = wmtk::read_mesh(meshfile);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_acc = m.create_accessor<double>(pos_handle);

    const size_t n_repetitions = 50000;

    const auto vertices = m.get_all(PrimitiveType::Vertex);

    // create matrix of positions
    Eigen::MatrixXd positions;
    positions.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        positions.row(i) = pos_acc.const_vector_attribute(vertices[i]);
    }

    {
        POLYSOLVE_SCOPED_STOPWATCH("Direct", logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < vertices.size(); ++i) {
                sum += positions(i, 0);
            }
            for (size_t i = 0; i < vertices.size(); ++i) {
                sum += positions(i, 1);
            }
            for (size_t i = 0; i < vertices.size(); ++i) {
                sum += positions(i, 2);
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }

    PointMesh pm(vertices.size());
    auto pph = mesh_utils::set_matrix_attribute(positions, "vertices", PrimitiveType::Vertex, pm);
    auto pp_acc = pm.create_accessor<double>(pph);
    {
        const auto vv = pm.get_all(PrimitiveType::Vertex);
        POLYSOLVE_SCOPED_STOPWATCH("PointMesh wmtk::attribute::Accessors", logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[0];
            }
            for (const Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[1];
            }
            for (const Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[2];
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }

    //{
    //    POLYSOLVE_SCOPED_STOPWATCH("TriMesh wmtk::attribute::Accessors", logger());
    //    double sum = 0;
    //    for (size_t i = 0; i < n_repetitions; ++i) {
    //        for (const Tuple& t : vertices) {
    //            sum += pos_acc.const_vector_attribute(t)[0];
    //        }
    //        for (const Tuple& t : vertices) {
    //            sum += pos_acc.const_vector_attribute(t)[1];
    //        }
    //        for (const Tuple& t : vertices) {
    //            sum += pos_acc.const_vector_attribute(t)[2];
    //        }
    //    }
    //    std::cout << "sum = " << sum << std::endl;
    //}
}

TEST_CASE("accessor_write_performance", "[accessor][performance][.]")
{
    const std::filesystem::path meshfile = data_dir / "armadillo.msh";

    logger().set_level(spdlog::level::trace);

    auto mesh_in = wmtk::read_mesh(meshfile);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_acc = m.create_accessor<double>(pos_handle);

    const size_t n_repetitions = 1000;

    const auto vertices = m.get_all(PrimitiveType::Vertex);

    // create matrix of positions
    Eigen::MatrixXd positions;
    positions.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        positions.row(i) = pos_acc.const_vector_attribute(vertices[i]);
    }

    {
        POLYSOLVE_SCOPED_STOPWATCH("Direct", logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < vertices.size(); ++i) {
                positions(i, 0) = sum++;
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }

    PointMesh pm(vertices.size());
    auto pph = mesh_utils::set_matrix_attribute(positions, "vertices", PrimitiveType::Vertex, pm);
    auto pp_acc = pm.create_accessor<double>(pph);
    {
        POLYSOLVE_SCOPED_STOPWATCH("PointMesh wmtk::attribute::Accessors", logger());
        double sum = 0;
        const auto vv = pm.get_all(PrimitiveType::Vertex);
        for (size_t i = 0; i < n_repetitions; ++i) {
            {
                auto scope = pm.create_scope();

                for (const Tuple& t : vv) {
                    pp_acc.vector_attribute(t)[0] = sum++;
                }

                // scope.mark_failed();
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
}

TEST_CASE("split_with_attributes", "[performance][.]")
{
    using namespace operations;

    const std::filesystem::path meshfile = data_dir / "armadillo.msh";
    const int64_t iterations = 5;

    auto mesh_in = wmtk::read_mesh(meshfile);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);


    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    pass_through_attributes.emplace_back(pos_handle);

    std::string test_name;
    SECTION("pos")
    {
        test_name = "[pos]";
    }
    SECTION("pos_double3")
    {
        test_name = "[pos,d3]";
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr", PrimitiveType::Vertex, 3));
    }
    SECTION("many_doubles")
    {
        test_name = "[pos,d3]";
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr1", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr2", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr3", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr4", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr5", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr6", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr7", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr8", PrimitiveType::Vertex, 3));
        pass_through_attributes.emplace_back(
            m.register_attribute<double>("double_attr9", PrimitiveType::Vertex, 3));
    }

    // split
    auto op_split = std::make_shared<EdgeSplit>(m);

    for (const auto& attr : pass_through_attributes) {
        op_split->set_new_attribute_strategy(attr);
    }


    {
        logger().set_level(spdlog::level::off);
        POLYSOLVE_SCOPED_STOPWATCH(test_name, logger());
        Scheduler scheduler;
        SchedulerStats pass_stats;
        for (long i = 0; i < iterations; ++i) {
            pass_stats += scheduler.run_operation_on_all(*op_split);
        }
        logger().set_level(spdlog::level::trace);
        logger().info("#performed operations : {}", pass_stats.number_of_performed_operations());
        logger().info("#successful operations: {}", pass_stats.number_of_successful_operations());
    }
}

TEST_CASE("collapse_performance", "[performance][.]")
{
    using namespace operations;

    const std::filesystem::path meshfile = data_dir / "armadillo.msh";
    const int64_t iterations = 5;

    auto mesh_in = wmtk::read_mesh(meshfile);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);


    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    pass_through_attributes.emplace_back(pos_handle);

    auto invariant_link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(m);

    //////////////////////////////////////////
    // collapse
    auto op_collapse = std::make_shared<EdgeCollapse>(m);
    op_collapse->add_invariant(invariant_link_condition);
    op_collapse->set_new_attribute_strategy(pos_handle);

    {
        logger().set_level(spdlog::level::off);
        POLYSOLVE_SCOPED_STOPWATCH("Collapse", logger());
        Scheduler scheduler;
        SchedulerStats pass_stats;
        for (long i = 0; i < iterations; ++i) {
            pass_stats += scheduler.run_operation_on_all(*op_collapse);
        }
        logger().set_level(spdlog::level::trace);
        logger().info("#performed operations : {}", pass_stats.number_of_performed_operations());
        logger().info("#successful operations: {}", pass_stats.number_of_successful_operations());
    }
}