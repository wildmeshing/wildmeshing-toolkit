#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/wildmeshing/wildmeshing.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/orient.hpp>

#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/utils/Logger.hpp>


#include <wmtk/Scheduler.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

#include <wmtk/operations/AMIPSOptimizationSmoothing.hpp>
#include <wmtk/operations/AndOperationSequence.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/MinOperationSequence.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/OrOperationSequence.hpp>
#include <wmtk/operations/Rounding.hpp>
#include <wmtk/operations/composite/ProjectOperation.hpp>
#include <wmtk/operations/composite/TetEdgeSwap.hpp>
#include <wmtk/operations/composite/TetFaceSwap.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>


#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>

#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxFunctionInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/RoundedInvariant.hpp>
#include <wmtk/invariants/SeparateSubstructuresInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/Swap23EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap32EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap44EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap44_2EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap56EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <fstream>

using namespace wmtk::components::base;
using namespace wmtk;
using namespace simplex;
using namespace operations;
using namespace operations::tri_mesh;
using namespace operations::tet_mesh;
using namespace operations::composite;
using namespace wmtk::attribute;
using namespace function;
using namespace invariants;

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("wildmeshing", "[components][wildmeshing][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json input_component_json = {
        {"name", "mesh"},
        {"file", data_dir / "adaptive_tessellation_test" / "after_smooth_uv.msh"},
        // {"file", data_dir / "2d" / "rect1.msh"},
        {"ignore_z", true}};
    wmtk::components::input(Paths(), input_component_json, cache);


    json input =
        R"({
        "passes": 10,
        "input": "mesh",
        "target_edge_length": 0.01,
        "intermediate_output": true,
        "attributes": {"position": "vertices"},
        "pass_through": [],
        "output": "test"
        })"_json;

    CHECK_NOTHROW(wmtk::components::wildmeshing(Paths(), input, cache));
}


TEST_CASE("wildmeshing_3d", "[components][wildmeshing][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json input_component_json = {
        {"name", "mesh"},
        // {"input", data_dir / "sphere_coarse_.msh"},
        // {"input", data_dir / "tet.msh"},
        // {"file", data_dir / "sphere_coarse_005_.msh"},
        {"file", data_dir / "cube_2.msh"},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), input_component_json, cache);

    json attributes = {{"position", "vertices"}};

    json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"target_edge_length", 0.05},
        {"intermediate_output", false},
        {"output", "test_mm"},
        {"pass_through", std::vector<int64_t>()},
        {"attributes", attributes}};


    CHECK_NOTHROW(wmtk::components::wildmeshing(Paths(), input, cache));
}

TEST_CASE("wildmeshing_3d_multimesh", "[components][wildmeshing][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json input_component_json = {
        {"name", "mesh"},
        // {"input", data_dir / "sphere_coarse_.msh"},
        // {"input", data_dir / "tet.msh"},
        {"file", data_dir / "sphere_coarse_005_.msh"},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), input_component_json, cache);

    json attributes = {{"position", "vertices"}};

    json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"target_edge_length", 0.1},
        {"intermediate_output", true},
        {"output", "test_multimesh"},
        {"pass_through", std::vector<int64_t>()},
        {"attributes", attributes}};


    CHECK_NOTHROW(wmtk::components::wildmeshing(Paths(), input, cache));
}

void run_tetwild_test(
    const std::string& path,
    const std::string& name,
    std::function<std::shared_ptr<Operation>(
        Mesh&,
        std::shared_ptr<Rounding>&,
        MeshAttributeHandle&,
        MeshAttributeHandle&,
        std::vector<attribute::MeshAttributeHandle>&,
        std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&,
        std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&,
        std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>&)>
        create_op)
{
    // const double target_edge_length = 1.21271;
    const double target_max_amips = 10;
    const double min_edge_length = 0.001;
    const double target_edge_length = 1.04; // for collapse
    // const double target_edge_length = 0.59019; // for split

    std::ifstream file(data_dir / path);
    std::vector<Vector3r> vertices;
    std::vector<Vector4l> tets;
    std::string x, y, z, token;

    while (file.good()) {
        std::string line;
        std::getline(file, line);
        std::istringstream iss(line);
        iss >> token;
        if (line[0] == 'v') {
            int sx = line.find(' ', 2);
            x = line.substr(2, sx - 2);

            int sy = line.find(' ', sx + 1);
            y = line.substr(sx + 1, sy - (sx + 1));

            int sz = line.find(' ', sy + 1);
            z = line.substr(sy + 1, sz - (sy + 1));

            vertices.emplace_back(x, y, z);
        } else if (line[0] == 't') {
            Vector4l f;
            iss >> f[0] >> f[1] >> f[3] >> f[2];
            tets.push_back(f);
        }
    }

    RowVectors3r v(vertices.size(), 3);
    for (int64_t i = 0; i < vertices.size(); ++i) {
        v.row(i) = vertices[i];
    }

    RowVectors4l t(tets.size(), 4);
    for (int64_t i = 0; i < tets.size(); ++i) {
        t.row(i) = tets[i];
        const auto ori = wmtk::utils::wmtk_orient3d<Rational>(
            v.row(t(i, 0)),
            v.row(t(i, 1)),
            v.row(t(i, 2)),
            v.row(t(i, 3)));
        CHECK(ori > 0);
    }

    const Vector3d bbox_max(
        v.col(0).cast<double>().maxCoeff(),
        v.col(1).cast<double>().maxCoeff(),
        v.col(2).cast<double>().maxCoeff());
    const Vector3d bbox_min(
        v.col(0).cast<double>().minCoeff(),
        v.col(1).cast<double>().minCoeff(),
        v.col(2).cast<double>().minCoeff());

    // std::cout << bbox_max << std::endl;
    // std::cout << bbox_min << std::endl;

    double diag = (bbox_max - bbox_min).norm();
    std::cout << diag << std::endl;
    std::cout << 0.05 * diag << std::endl;


    auto mesh = std::make_shared<TetMesh>();
    mesh->initialize(t);
    mesh_utils::set_matrix_attribute(v, "vertices", PrimitiveType::Vertex, *mesh);
    logger().info("Mesh has {} vertices {} tests", vertices.size(), tets.size());


    /////////////////////////
    auto pt_attribute = mesh->get_attribute_handle<Rational>("vertices", PrimitiveType::Vertex);
    auto pt_accessor = mesh->create_accessor(pt_attribute.as<Rational>());

    auto amips_attribute =
        mesh->register_attribute<double>("wildmeshing_amips", mesh->top_simplex_type(), 1);
    auto amips_accessor = mesh->create_accessor(amips_attribute.as<double>());
    // amips update
    auto compute_amips = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorXd {
        assert(P.rows() == 2 || P.rows() == 3); // rows --> attribute dimension
        assert(P.cols() == P.rows() + 1);
        if (P.cols() == 3) {
            // triangle
            assert(P.rows() == 2);
            std::array<double, 6> pts;
            for (size_t i = 0; i < 3; ++i) {
                for (size_t j = 0; j < 2; ++j) {
                    pts[2 * i + j] = P(j, i).to_double();
                }
            }
            const double a = Tri_AMIPS_energy(pts);
            return Eigen::VectorXd::Constant(1, a);
        } else {
            // tet
            assert(P.rows() == 3);
            std::array<double, 12> pts;
            for (size_t i = 0; i < 4; ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    pts[3 * i + j] = P(j, i).to_double();
                }
            }
            const double a = Tet_AMIPS_energy(pts);
            return Eigen::VectorXd::Constant(1, a);
        }
    };
    auto amips_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>(
            amips_attribute,
            pt_attribute,
            compute_amips);
    amips_update->run_on_all();

    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh->create_accessor(edge_length_attribute.as<double>());
    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, sqrt((P.col(0) - P.col(1)).squaredNorm().to_double()));
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>(
            edge_length_attribute,
            pt_attribute,
            compute_edge_length);
    edge_length_update->run_on_all();

    auto visited_edge_flag =
        mesh->register_attribute<char>("visited_edge", PrimitiveType::Edge, 1, false, char(1));

    auto target_edge_length_attribute = mesh->register_attribute<double>(
        "wildmeshing_target_edge_length",
        PrimitiveType::Edge,
        1,
        false,
        target_edge_length); // defaults to target edge length


    auto compute_target_edge_length =
        [target_edge_length,
         target_max_amips,
         min_edge_length,
         target_edge_length_attribute,
         &mesh](const Eigen::MatrixXd& P, const std::vector<Tuple>& neighs) -> Eigen::VectorXd {
        auto target_edge_length_accessor =
            mesh->create_accessor(target_edge_length_attribute.as<double>());

        assert(P.rows() == 1); // rows --> attribute dimension
        assert(!neighs.empty());
        assert(P.cols() == neighs.size());
        const double current_target_edge_length =
            target_edge_length_accessor.const_scalar_attribute(neighs[0]);
        const double max_amips = P.maxCoeff();

        double new_target_edge_length = current_target_edge_length;
        if (max_amips > target_max_amips) {
            new_target_edge_length *= 0.5;
        } else {
            new_target_edge_length *= 1.5;
        }
        new_target_edge_length =
            std::min(new_target_edge_length, target_edge_length); // upper bound
        new_target_edge_length = std::max(new_target_edge_length, min_edge_length); // lower bound

        return Eigen::VectorXd::Constant(1, new_target_edge_length);
    };
    auto target_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            target_edge_length_attribute,
            amips_attribute,
            compute_target_edge_length);
    auto update_flag_func = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<char> {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto flag_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>(
            visited_edge_flag,
            pt_attribute,
            update_flag_func);


    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    pass_through_attributes.push_back(edge_length_attribute);
    pass_through_attributes.push_back(amips_attribute);
    pass_through_attributes.push_back(target_edge_length_attribute);

    // ////////////////////////////////////////////////////

    auto inversion_invariant =
        std::make_shared<SimplexInversionInvariant<Rational>>(*mesh, pt_attribute.as<Rational>());

    ///////////////////////////

    auto rounding_pt_attribute =
        mesh->get_attribute_handle_typed<Rational>("vertices", PrimitiveType::Vertex);
    auto rounding = std::make_shared<Rounding>(*mesh, rounding_pt_attribute);
    rounding->add_invariant(
        std::make_shared<RoundedInvariant>(*mesh, pt_attribute.as<Rational>(), true));
    rounding->add_invariant(inversion_invariant);
    ////////////////////////////////


    Scheduler scheduler;
    int success = 10;
    while (success > 0) {
        auto stats = scheduler.run_operation_on_all(*rounding);
        logger().info(
            "Rounding, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
            "executing: {}. Total {}",
            stats.number_of_performed_operations(),
            stats.number_of_successful_operations(),
            stats.number_of_failed_operations(),
            stats.collecting_time,
            stats.sorting_time,
            stats.executing_time,
            stats.total_time());

        success = stats.number_of_successful_operations();

        int64_t unrounded = 0;
        for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
            const auto p = pt_accessor.vector_attribute(v);
            for (int64_t d = 0; d < 3; ++d) {
                if (!p[d].is_rounded()) {
                    ++unrounded;
                    break;
                }
            }
        }

        logger().info("Mesh has {} unrounded vertices", unrounded);
    }

    auto op = create_op(
        *mesh,
        rounding,
        pt_attribute,
        visited_edge_flag,
        pass_through_attributes,
        amips_update,
        edge_length_update,
        flag_update);

    auto stats = scheduler.run_operation_on_all(*op, visited_edge_flag.as<char>());
    logger().info(
        "{}, {} ops (S/F) {}/{}. Passes: {}. Time collecting: {}, sorting: {}, "
        "executing: {}; total {}. Passes (avg) collecting: {}, sorting: {}, executing: {}",
        name,
        stats.number_of_performed_operations(),
        stats.number_of_successful_operations(),
        stats.number_of_failed_operations(),
        stats.sub_stats.size(),
        stats.collecting_time,
        stats.sorting_time,
        stats.executing_time,
        stats.total_time(),
        stats.avg_sub_collecting_time(),
        stats.avg_sub_sorting_time(),
        stats.avg_sub_executing_time());

    int64_t unrounded = 0;
    for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < 3; ++d) {
            if (!p[d].is_rounded()) {
                ++unrounded;
                break;
            }
        }
    }

    logger().info("Mesh has {} unrounded vertices", unrounded);

    double max_energy = std::numeric_limits<double>::lowest();
    double min_energy = std::numeric_limits<double>::max();
    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
        double e = amips_accessor.scalar_attribute(t);
        max_energy = std::max(max_energy, e);
        min_energy = std::min(min_energy, e);
    }

    logger().info("Max AMIPS Energy: {}, Min AMIPS Energy: {}", max_energy, min_energy);
    logger().info(
        "{} vertices, {} edges, {} tets",
        mesh->get_all(PrimitiveType::Vertex).size(),
        mesh->get_all(PrimitiveType::Edge).size(),
        mesh->get_all(PrimitiveType::Tetrahedron).size());
}


TEST_CASE("tetwild-split", "[components][wildmeshing][.]")
{
    logger().set_level(spdlog::level::debug);

    run_tetwild_test(
        "split_initial_state_test.obj",
        "Split",
        [](Mesh& mesh,
           std::shared_ptr<Rounding>& rounding,
           MeshAttributeHandle& pt_attribute,
           MeshAttributeHandle& visited_edge_flag,
           std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               amips_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               edge_length_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>&
               tag_update) {
            auto edge_length_attribute =
                mesh.get_attribute_handle<double>("edge_length", PrimitiveType::Edge);
            auto target_edge_length_attribute = mesh.get_attribute_handle<double>(
                "wildmeshing_target_edge_length",
                PrimitiveType::Edge);
            auto edge_length_accessor = mesh.create_accessor(edge_length_attribute.as<double>());

            auto long_edges_first = [&](const simplex::Simplex& s) {
                assert(s.primitive_type() == PrimitiveType::Edge);
                return -edge_length_accessor.scalar_attribute(s);
            };

            auto todo_larger = std::make_shared<TodoLargerInvariant>(
                mesh,
                edge_length_attribute.as<double>(),
                target_edge_length_attribute.as<double>(),
                4.0 / 3.0);

            auto split = std::make_shared<EdgeSplit>(mesh);
            split->set_priority(long_edges_first);

            split->add_invariant(todo_larger);

            split->set_new_attribute_strategy(pt_attribute);
            split->set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            for (const auto& attr : pass_through_attributes) {
                split->set_new_attribute_strategy(attr);
            }

            split->add_transfer_strategy(amips_update);
            split->add_transfer_strategy(edge_length_update);
            split->add_transfer_strategy(tag_update);

            auto split_then_round = std::make_shared<AndOperationSequence>(mesh);
            split_then_round->add_operation(split);
            split_then_round->add_operation(rounding);

            return split_then_round;
        });
}

TEST_CASE("tetwild-collapse", "[components][wildmeshing][.]")
{
    // logger().set_level(spdlog::level::trace);
    run_tetwild_test(
        "collapse_initial_state_141017.obj",
        "Collapse",
        [](Mesh& mesh,
           std::shared_ptr<Rounding>& rounding,
           MeshAttributeHandle& pt_attribute,
           MeshAttributeHandle& visited_edge_flag,
           std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               amips_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               edge_length_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>&
               tag_update) {
            auto edge_length_attribute =
                mesh.get_attribute_handle<double>("edge_length", PrimitiveType::Edge);
            auto target_edge_length_attribute = mesh.get_attribute_handle<double>(
                "wildmeshing_target_edge_length",
                PrimitiveType::Edge);
            auto edge_length_accessor = mesh.create_accessor(edge_length_attribute.as<double>());


            auto clps_strat =
                std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
            clps_strat->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            // clps_strat->set_strategy(CollapseBasicStrategy::Default);
            clps_strat->set_strategy(CollapseBasicStrategy::CopyOther);


            auto inversion_invariant = std::make_shared<SimplexInversionInvariant<Rational>>(
                mesh,
                pt_attribute.as<Rational>());
            auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(mesh);
            auto invariant_separate_substructures =
                std::make_shared<invariants::SeparateSubstructuresInvariant>(mesh);

            auto short_edges_first = [&](const simplex::Simplex& s) {
                assert(s.primitive_type() == PrimitiveType::Edge);
                return edge_length_accessor.scalar_attribute(s);
            };

            auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
                mesh,
                edge_length_attribute.as<double>(),
                target_edge_length_attribute.as<double>(),
                4.0 / 5.0);

            std::shared_ptr<function::PerSimplexFunction> amips =
                std::make_shared<AMIPS>(mesh, pt_attribute);

            auto function_invariant = std::make_shared<MaxFunctionInvariant>(
                mesh.top_simplex_type(),
                amips,
                pt_attribute.as<Rational>());

            //////////////////////////////////
            // 2) EdgeCollapse
            //////////////////////////////////
            auto collapse = std::make_shared<EdgeCollapse>(mesh);
            // THis triggers a segfault in release
            // collapse->set_priority(short_edges_first);

            collapse->add_invariant(todo_smaller);
            collapse->add_invariant(invariant_separate_substructures);
            collapse->add_invariant(link_condition);
            collapse->add_invariant(inversion_invariant);
            collapse->add_invariant(function_invariant);
            // collapse->add_invariant(envelope_invariant);

            collapse->set_new_attribute_strategy(pt_attribute, clps_strat);
            collapse->add_transfer_strategy(tag_update);
            collapse->set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::CollapseBasicStrategy::None);
            for (const auto& attr : pass_through_attributes) {
                collapse->set_new_attribute_strategy(attr);
            }

            collapse->add_transfer_strategy(amips_update);
            collapse->add_transfer_strategy(edge_length_update);
            // proj_collapse->add_transfer_strategy(target_edge_length_update);

            auto collapse_then_round = std::make_shared<AndOperationSequence>(mesh);
            collapse_then_round->add_operation(collapse);
            collapse_then_round->add_operation(rounding);

            return collapse_then_round;
        });
}

TEST_CASE("tetwild-collapse-twoway", "[components][wildmeshing][.]")
{
    logger().set_level(spdlog::level::trace);
    run_tetwild_test(
        "collapse_initial_state_141017.obj",
        "Collapse-Two",
        [](Mesh& mesh,
           std::shared_ptr<Rounding>& rounding,
           MeshAttributeHandle& pt_attribute,
           MeshAttributeHandle& visited_edge_flag,
           std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               amips_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               edge_length_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>&
               tag_update) {
            auto edge_length_attribute =
                mesh.get_attribute_handle<double>("edge_length", PrimitiveType::Edge);
            auto target_edge_length_attribute = mesh.get_attribute_handle<double>(
                "wildmeshing_target_edge_length",
                PrimitiveType::Edge);
            auto edge_length_accessor = mesh.create_accessor(edge_length_attribute.as<double>());


            auto clps_strat =
                std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
            clps_strat->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            // clps_strat->set_strategy(CollapseBasicStrategy::Default);
            clps_strat->set_strategy(CollapseBasicStrategy::CopyOther);


            auto inversion_invariant = std::make_shared<SimplexInversionInvariant<Rational>>(
                mesh,
                pt_attribute.as<Rational>());
            auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(mesh);
            auto invariant_separate_substructures =
                std::make_shared<invariants::SeparateSubstructuresInvariant>(mesh);

            auto short_edges_first = [&](const simplex::Simplex& s) {
                assert(s.primitive_type() == PrimitiveType::Edge);
                return edge_length_accessor.scalar_attribute(s);
            };

            auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
                mesh,
                edge_length_attribute.as<double>(),
                target_edge_length_attribute.as<double>(),
                4.0 / 5.0);

            std::shared_ptr<function::PerSimplexFunction> amips =
                std::make_shared<AMIPS>(mesh, pt_attribute);

            auto function_invariant = std::make_shared<MaxFunctionInvariant>(
                mesh.top_simplex_type(),
                amips,
                pt_attribute.as<Rational>());

            //////////////////////////////////
            // 2) EdgeCollapse
            //////////////////////////////////
            auto clps_strat1 =
                std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
            clps_strat1->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            // clps_strat1->set_strategy(CollapseBasicStrategy::Default);
            clps_strat1->set_strategy(CollapseBasicStrategy::CopyOther);

            auto clps_strat2 =
                std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
            clps_strat2->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            // clps_strat2->set_strategy(CollapseBasicStrategy::Default);
            clps_strat2->set_strategy(CollapseBasicStrategy::CopyTuple);

            auto setup_collapse = [&](std::shared_ptr<EdgeCollapse>& collapse) {
                collapse->add_invariant(invariant_separate_substructures);
                collapse->add_invariant(link_condition);
                collapse->add_invariant(inversion_invariant);
                collapse->add_invariant(function_invariant);


                collapse->set_new_attribute_strategy(
                    visited_edge_flag,
                    wmtk::operations::CollapseBasicStrategy::None);

                collapse->add_transfer_strategy(tag_update);
                for (const auto& attr : pass_through_attributes) {
                    collapse->set_new_attribute_strategy(attr);
                }
                // THis triggers a segfault in release
                // collapse->set_priority(short_edges_first);

                // collapse->add_invariant(envelope_invariant);

                collapse->add_transfer_strategy(amips_update);
                collapse->add_transfer_strategy(edge_length_update);
                // proj_collapse->add_transfer_strategy(target_edge_length_update);
            };

            auto collapse1 = std::make_shared<EdgeCollapse>(mesh);
            collapse1->set_new_attribute_strategy(pt_attribute, clps_strat1);
            setup_collapse(collapse1);

            auto collapse2 = std::make_shared<EdgeCollapse>(mesh);
            collapse2->set_new_attribute_strategy(pt_attribute, clps_strat2);
            setup_collapse(collapse2);

            auto collapse = std::make_shared<OrOperationSequence>(mesh);
            collapse->add_operation(collapse1);
            collapse->add_operation(collapse2);
            collapse->add_invariant(todo_smaller);

            auto collapse_then_round = std::make_shared<AndOperationSequence>(mesh);
            collapse_then_round->add_operation(collapse);
            collapse_then_round->add_operation(rounding);

            return collapse_then_round;
        });
}

TEST_CASE("tetwild-swap", "[components][wildmeshing][.]")
{
    // logger().set_level(spdlog::level::trace);
    run_tetwild_test(
        "swap_initial_state.obj",
        "Swap",
        [](Mesh& mesh,
           std::shared_ptr<Rounding>& rounding,
           MeshAttributeHandle& pt_attribute,
           MeshAttributeHandle& visited_edge_flag,
           std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               amips_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>&
               edge_length_update,
           std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>&
               tag_update) {
            auto edge_length_attribute =
                mesh.get_attribute_handle<double>("edge_length", PrimitiveType::Edge);
            auto target_edge_length_attribute = mesh.get_attribute_handle<double>(
                "wildmeshing_target_edge_length",
                PrimitiveType::Edge);
            auto edge_length_accessor = mesh.create_accessor(edge_length_attribute.as<double>());


            auto clps_strat =
                std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
            clps_strat->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            // clps_strat->set_strategy(CollapseBasicStrategy::Default);
            clps_strat->set_strategy(CollapseBasicStrategy::CopyOther);


            auto inversion_invariant = std::make_shared<SimplexInversionInvariant<Rational>>(
                mesh,
                pt_attribute.as<Rational>());
            auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(mesh);
            auto invariant_separate_substructures =
                std::make_shared<invariants::SeparateSubstructuresInvariant>(mesh);

            auto long_edges_first = [&](const simplex::Simplex& s) {
                assert(s.primitive_type() == PrimitiveType::Edge);
                return -edge_length_accessor.scalar_attribute(s);
            };

            std::shared_ptr<function::PerSimplexFunction> amips =
                std::make_shared<AMIPS>(mesh, pt_attribute);

            //////////////////////////////////
            // 3) Swap
            //////////////////////////////////

            // swap56

            auto swap56 = std::make_shared<MinOperationSequence>(mesh);
            for (int i = 0; i < 5; ++i) {
                auto swap = std::make_shared<TetEdgeSwap>(mesh, i);
                swap->collapse().add_invariant(invariant_separate_substructures);
                swap->collapse().add_invariant(link_condition);
                swap->collapse().set_new_attribute_strategy(
                    pt_attribute,
                    CollapseBasicStrategy::CopyOther);
                swap->split().set_new_attribute_strategy(pt_attribute);
                swap->split().set_new_attribute_strategy(
                    visited_edge_flag,
                    wmtk::operations::SplitBasicStrategy::None,
                    wmtk::operations::SplitRibBasicStrategy::None);
                swap->collapse().set_new_attribute_strategy(
                    visited_edge_flag,
                    wmtk::operations::CollapseBasicStrategy::None);

                swap->add_invariant(std::make_shared<Swap56EnergyBeforeInvariant>(
                    mesh,
                    pt_attribute.as<Rational>(),
                    i));

                for (const auto& attr : pass_through_attributes) {
                    swap->split().set_new_attribute_strategy(attr);
                    swap->collapse().set_new_attribute_strategy(attr);
                }

                swap56->add_operation(swap);
            }

            auto swap56_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
                constexpr static PrimitiveType PV = PrimitiveType::Vertex;
                constexpr static PrimitiveType PE = PrimitiveType::Edge;
                constexpr static PrimitiveType PF = PrimitiveType::Triangle;
                constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

                auto accessor = mesh.create_const_accessor(pt_attribute.as<Rational>());

                const Tuple e0 = t.tuple();
                const Tuple e1 = mesh.switch_tuple(e0, PV);

                std::array<Tuple, 5> v;
                auto iter_tuple = e0;
                for (int64_t i = 0; i < 5; ++i) {
                    v[i] = mesh.switch_tuples(iter_tuple, {PE, PV});
                    iter_tuple = mesh.switch_tuples(iter_tuple, {PF, PT});
                }
                if (iter_tuple != e0) return 0;
                assert(iter_tuple == e0);

                // five iterable vertices remap to 0-4 by m_collapse_index, 0: m_collapse_index, 5:
                // e0, 6: e1
                std::array<Eigen::Vector3<Rational>, 7> positions = {
                    {accessor.const_vector_attribute(v[(idx + 0) % 5]),
                     accessor.const_vector_attribute(v[(idx + 1) % 5]),
                     accessor.const_vector_attribute(v[(idx + 2) % 5]),
                     accessor.const_vector_attribute(v[(idx + 3) % 5]),
                     accessor.const_vector_attribute(v[(idx + 4) % 5]),
                     accessor.const_vector_attribute(e0),
                     accessor.const_vector_attribute(e1)}};

                std::array<Eigen::Vector3d, 7> positions_double = {
                    {positions[0].cast<double>(),
                     positions[1].cast<double>(),
                     positions[2].cast<double>(),
                     positions[3].cast<double>(),
                     positions[4].cast<double>(),
                     positions[5].cast<double>(),
                     positions[6].cast<double>()}};

                std::array<std::array<int, 4>, 6> new_tets = {
                    {{{0, 1, 2, 5}},
                     {{0, 2, 3, 5}},
                     {{0, 3, 4, 5}},
                     {{0, 1, 2, 6}},
                     {{0, 2, 3, 6}},
                     {{0, 3, 4, 6}}}};

                double new_energy_max = std::numeric_limits<double>::lowest();

                for (int i = 0; i < 6; ++i) {
                    if (wmtk::utils::wmtk_orient3d(
                            positions[new_tets[i][0]],
                            positions[new_tets[i][1]],
                            positions[new_tets[i][2]],
                            positions[new_tets[i][3]]) > 0) {
                        auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                            positions_double[new_tets[i][0]][0],
                            positions_double[new_tets[i][0]][1],
                            positions_double[new_tets[i][0]][2],
                            positions_double[new_tets[i][1]][0],
                            positions_double[new_tets[i][1]][1],
                            positions_double[new_tets[i][1]][2],
                            positions_double[new_tets[i][2]][0],
                            positions_double[new_tets[i][2]][1],
                            positions_double[new_tets[i][2]][2],
                            positions_double[new_tets[i][3]][0],
                            positions_double[new_tets[i][3]][1],
                            positions_double[new_tets[i][3]][2],
                        }});


                        if (energy > new_energy_max) new_energy_max = energy;
                    } else {
                        auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                            positions_double[new_tets[i][1]][0],
                            positions_double[new_tets[i][1]][1],
                            positions_double[new_tets[i][1]][2],
                            positions_double[new_tets[i][0]][0],
                            positions_double[new_tets[i][0]][1],
                            positions_double[new_tets[i][0]][2],
                            positions_double[new_tets[i][2]][0],
                            positions_double[new_tets[i][2]][1],
                            positions_double[new_tets[i][2]][2],
                            positions_double[new_tets[i][3]][0],
                            positions_double[new_tets[i][3]][1],
                            positions_double[new_tets[i][3]][2],
                        }});


                        if (energy > new_energy_max) new_energy_max = energy;
                    }
                }

                return new_energy_max;
            };

            swap56->set_value_function(swap56_energy_check);
            swap56->add_invariant(std::make_shared<EdgeValenceInvariant>(mesh, 5));

            // swap44

            auto swap44 = std::make_shared<MinOperationSequence>(mesh);
            for (int i = 0; i < 2; ++i) {
                auto swap = std::make_shared<TetEdgeSwap>(mesh, i);
                swap->collapse().add_invariant(invariant_separate_substructures);
                swap->collapse().add_invariant(link_condition);
                swap->collapse().set_new_attribute_strategy(
                    pt_attribute,
                    CollapseBasicStrategy::CopyOther);
                swap->split().set_new_attribute_strategy(pt_attribute);
                swap->split().set_new_attribute_strategy(
                    visited_edge_flag,
                    wmtk::operations::SplitBasicStrategy::None,
                    wmtk::operations::SplitRibBasicStrategy::None);
                swap->collapse().set_new_attribute_strategy(
                    visited_edge_flag,
                    wmtk::operations::CollapseBasicStrategy::None);

                swap->add_invariant(std::make_shared<Swap44EnergyBeforeInvariant>(
                    mesh,
                    pt_attribute.as<Rational>(),
                    i));

                for (const auto& attr : pass_through_attributes) {
                    swap->split().set_new_attribute_strategy(attr);
                    swap->collapse().set_new_attribute_strategy(attr);
                }

                swap44->add_operation(swap);
            }

            auto swap44_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
                constexpr static PrimitiveType PV = PrimitiveType::Vertex;
                constexpr static PrimitiveType PE = PrimitiveType::Edge;
                constexpr static PrimitiveType PF = PrimitiveType::Triangle;
                constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

                auto accessor = mesh.create_const_accessor(pt_attribute.as<Rational>());

                // get the coords of the vertices
                // input edge end points
                const Tuple e0 = t.tuple();
                const Tuple e1 = mesh.switch_tuple(e0, PV);
                // other four vertices
                std::array<Tuple, 4> v;
                auto iter_tuple = e0;
                for (int64_t i = 0; i < 4; ++i) {
                    v[i] = mesh.switch_tuples(iter_tuple, {PE, PV});
                    iter_tuple = mesh.switch_tuples(iter_tuple, {PF, PT});
                }

                if (iter_tuple != e0) return 0;
                assert(iter_tuple == e0);

                std::array<Eigen::Vector3<Rational>, 6> positions = {
                    {accessor.const_vector_attribute(v[(idx + 0) % 4]),
                     accessor.const_vector_attribute(v[(idx + 1) % 4]),
                     accessor.const_vector_attribute(v[(idx + 2) % 4]),
                     accessor.const_vector_attribute(v[(idx + 3) % 4]),
                     accessor.const_vector_attribute(e0),
                     accessor.const_vector_attribute(e1)}};
                std::array<Eigen::Vector3d, 6> positions_double = {
                    {positions[0].cast<double>(),
                     positions[1].cast<double>(),
                     positions[2].cast<double>(),
                     positions[3].cast<double>(),
                     positions[4].cast<double>(),
                     positions[5].cast<double>()}};

                std::array<std::array<int, 4>, 4> new_tets = {
                    {{{0, 1, 2, 4}}, {{0, 2, 3, 4}}, {{0, 1, 2, 5}}, {{0, 2, 3, 5}}}};

                double new_energy_max = std::numeric_limits<double>::lowest();

                for (int i = 0; i < 4; ++i) {
                    if (wmtk::utils::wmtk_orient3d(
                            positions[new_tets[i][0]],
                            positions[new_tets[i][1]],
                            positions[new_tets[i][2]],
                            positions[new_tets[i][3]]) > 0) {
                        auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                            positions_double[new_tets[i][0]][0],
                            positions_double[new_tets[i][0]][1],
                            positions_double[new_tets[i][0]][2],
                            positions_double[new_tets[i][1]][0],
                            positions_double[new_tets[i][1]][1],
                            positions_double[new_tets[i][1]][2],
                            positions_double[new_tets[i][2]][0],
                            positions_double[new_tets[i][2]][1],
                            positions_double[new_tets[i][2]][2],
                            positions_double[new_tets[i][3]][0],
                            positions_double[new_tets[i][3]][1],
                            positions_double[new_tets[i][3]][2],
                        }});

                        if (energy > new_energy_max) new_energy_max = energy;
                    } else {
                        auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                            positions_double[new_tets[i][1]][0],
                            positions_double[new_tets[i][1]][1],
                            positions_double[new_tets[i][1]][2],
                            positions_double[new_tets[i][0]][0],
                            positions_double[new_tets[i][0]][1],
                            positions_double[new_tets[i][0]][2],
                            positions_double[new_tets[i][2]][0],
                            positions_double[new_tets[i][2]][1],
                            positions_double[new_tets[i][2]][2],
                            positions_double[new_tets[i][3]][0],
                            positions_double[new_tets[i][3]][1],
                            positions_double[new_tets[i][3]][2],
                        }});

                        if (energy > new_energy_max) new_energy_max = energy;
                    }
                }

                return new_energy_max;
            };

            swap44->set_value_function(swap44_energy_check);
            swap44->add_invariant(std::make_shared<EdgeValenceInvariant>(mesh, 4));

            // swap 32
            auto swap32 = std::make_shared<TetEdgeSwap>(mesh, 0);
            swap32->add_invariant(std::make_shared<EdgeValenceInvariant>(mesh, 3));
            swap32->add_invariant(
                std::make_shared<Swap32EnergyBeforeInvariant>(mesh, pt_attribute.as<Rational>()));

            swap32->collapse().add_invariant(invariant_separate_substructures);
            swap32->collapse().add_invariant(link_condition);
            swap32->collapse().set_new_attribute_strategy(
                pt_attribute,
                CollapseBasicStrategy::CopyOther);
            swap32->split().set_new_attribute_strategy(pt_attribute);
            swap32->split().set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            swap32->collapse().set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::CollapseBasicStrategy::None);

            for (const auto& attr : pass_through_attributes) {
                swap32->split().set_new_attribute_strategy(attr);
                swap32->collapse().set_new_attribute_strategy(attr);
            }

            // all swaps

            auto swap_all = std::make_shared<OrOperationSequence>(mesh);
            swap_all->add_invariant(std::make_shared<InteriorEdgeInvariant>(mesh));
            swap_all->add_operation(swap32);
            swap_all->add_operation(swap44);
            swap_all->add_operation(swap56);

            auto swap_then_round = std::make_shared<AndOperationSequence>(mesh);
            swap_then_round->add_operation(swap_all);
            swap_then_round->add_operation(rounding);


            return swap_then_round;
        });
}