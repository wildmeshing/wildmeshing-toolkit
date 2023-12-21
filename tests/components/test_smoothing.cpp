#include <catch2/catch_test_macros.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/function/simplex/TetrahedronAMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::operations;
using namespace wmtk::tests_3d;

// this is a test energy class for mapping one triangle to another triangle
namespace wmtk::function {
class SquareDistance : public PerSimplexAutodiffFunction
{
public:
    SquareDistance(
        const TriMesh& mesh,
        const attribute::MeshAttributeHandle<double>& attribute_handle,
        const attribute::MeshAttributeHandle<double>& target_attribute_handle)
        : PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, attribute_handle)
        , m_target_attribute_accessor(mesh.create_const_accessor(target_attribute_handle))
    {}
    ~SquareDistance() override = default;
    using DScalar = PerSimplexAutodiffFunction::DScalar;
    using DSVec = Eigen::VectorX<DScalar>;

protected:
    DScalar eval(const Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override
    {
        auto target_coordinates = get_coordinates(m_target_attribute_accessor, domain_simplex);
        DScalar r;
        for (size_t j = 0; j < 3; ++j) {
            auto c = coordinates[j];
            auto t = target_coordinates[j];
            r += (c - t).squaredNorm();
        }
        return r;
    }

    ConstAccessor<double> m_target_attribute_accessor;
};
} // namespace wmtk::function
TEST_CASE("smoothing_Newton_Method")
{
    DEBUG_TriMesh mesh = single_2d_nonequilateral_triangle_with_positions();
    OperationSettings<OptimizationSmoothing> op_settings(mesh);
    op_settings.coordinate_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    function::TriangleAMIPS per_tri_amips(
        mesh,
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));
    op_settings.energy = std::make_unique<function::LocalNeighborsSumFunction>(
        mesh,
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex),
        per_tri_amips);

    Scheduler scheduler(mesh);

    auto& factory = scheduler.add_operation_type<operations::OptimizationSmoothing>(
        "optimize_vertices",
        std::move(op_settings));
    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &factory]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            double grad_norm = factory.settings()
                                   .energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple))
                                   .norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    while (get_min_grad_norm() > 1e-10) {
        scheduler.run_operation_on_all(PrimitiveType::Vertex, "optimize_vertices");
        REQUIRE(scheduler.number_of_successful_operations() > 0);
    }
    ConstAccessor<double> pos = mesh.create_const_accessor(op_settings.coordinate_handle);
    Tuple tuple = mesh.tuple_from_face_id(0);
    Eigen::Vector2d uv0 = pos.const_vector_attribute(tuple);
    Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh.switch_vertex(tuple));
    Eigen::Vector2d uv2 = pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(tuple)));

    CHECK((uv0 - uv1).norm() - (uv1 - uv2).norm() < 1e-6);
    CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() < 1e-6);
    CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
}

TEST_CASE("smoothing_tet_amips")
{
    TetMesh mesh = three_incident_tets_with_positions();
    OperationSettings<OptimizationSmoothing> op_settings(mesh);
    op_settings.coordinate_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    function::TetrahedronAMIPS amips(
        mesh,
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));
    op_settings.energy = std::make_unique<function::LocalNeighborsSumFunction>(
        mesh,
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex),
        amips);

    Scheduler scheduler(mesh);

    auto& factory = scheduler.add_operation_type<operations::OptimizationSmoothing>(
        "optimize_vertices",
        std::move(op_settings));
    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &factory]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            double grad_norm = factory.settings()
                                   .energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple))
                                   .norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    while (get_min_grad_norm() > 1e-10) {
        scheduler.run_operation_on_all(PrimitiveType::Vertex, "optimize_vertices");
        REQUIRE(scheduler.number_of_successful_operations() > 0);
    }
}


TEST_CASE("smoothing_Gradient_Descent")
{
    DEBUG_TriMesh mesh = single_2d_nonequilateral_triangle_with_positions();
    OperationSettings<OptimizationSmoothing> op_settings(mesh);
    op_settings.coordinate_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto target_coordinate_handle =
        mesh.register_attribute<double>("target_coordinate", PrimitiveType::Vertex, 2);

    auto target_acc = mesh.create_accessor(target_coordinate_handle);

    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 0)) << 0, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 1)) << 1, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 2)) << 0, 1;

    function::SquareDistance squared_dist(
        mesh,
        op_settings.coordinate_handle,
        target_coordinate_handle);
    op_settings.energy = std::make_unique<function::LocalNeighborsSumFunction>(
        mesh,
        op_settings.coordinate_handle,
        squared_dist);

    Scheduler scheduler(mesh);

    auto& factory = scheduler.add_operation_type<operations::OptimizationSmoothing>(
        "optimize_vertices",
        std::move(op_settings));
    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &factory]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            Eigen::Vector2d grad =
                factory.settings().energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple));
            double grad_norm = grad.norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    do {
        scheduler.run_operation_on_all(PrimitiveType::Vertex, "optimize_vertices");
    } while (get_min_grad_norm() > 1e-3 && scheduler.number_of_successful_operations() > 0);
    ConstAccessor<double> pos = mesh.create_const_accessor(op_settings.coordinate_handle);
    Tuple tuple = mesh.tuple_from_face_id(0);
    Eigen::Vector2d uv0 = pos.const_vector_attribute(tuple);
    Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh.switch_vertex(tuple));
    Eigen::Vector2d uv2 = pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(tuple)));


    // CHECK((uv0 - uv1).norm() - (uv1 - uv2).norm() < 1e-6);
    // CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() < 1e-6);
    // CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
}
