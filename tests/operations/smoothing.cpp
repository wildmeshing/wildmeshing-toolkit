#include <catch2/catch_test_macros.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/SYMDIR.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;
using namespace wmtk::operations;
using namespace wmtk::tests_3d;

// this is a test energy class for mapping one triangle to another triangle
namespace wmtk::function {
class SquareDistance : public PerSimplexAutodiffFunction
{
public:
    SquareDistance(
        const TriMesh& mesh,
        const attribute::TypedAttributeHandle<double>& attribute_handle,
        const attribute::TypedAttributeHandle<double>& target_attribute_handle)
        : PerSimplexAutodiffFunction(
              mesh,
              PrimitiveType::Vertex,
              attribute::MeshAttributeHandle(const_cast<TriMesh&>(mesh), attribute_handle))
        , m_target_attribute_accessor(mesh.create_const_accessor<double>(target_attribute_handle))
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
    auto handler = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    function::SYMDIR per_tri_energy(mesh, handler);
    auto energy =
        std::make_shared<function::LocalNeighborsSumFunction>(mesh, handler, per_tri_energy);

    OptimizationSmoothing op(mesh, energy);
    op.add_invariant(std::make_shared<SimplexInversionInvariant>(mesh, handler.as<double>()));
    Scheduler scheduler;

    {
        for (const auto& tuple : mesh.get_all(PrimitiveType::Vertex)) {
            auto value = energy->get_value(Simplex(PrimitiveType::Vertex, tuple));
            auto grad = energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple));
            auto hess = energy->get_hessian(Simplex(PrimitiveType::Vertex, tuple));
            std::cout << "value = " << value << std::endl;
            std::cout << "grad = \n" << grad << std::endl;
            std::cout << "grad.norm() = " << grad.norm() << std::endl;
            std::cout << std::endl;
        }
    }

    // iterate all the vertices and find max gradnorm
    auto get_max_grad_norm = [&mesh, &energy]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double max_grad_norm = 0;
        for (const Tuple& tuple : tuples) {
            double grad_norm = energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple)).norm();
            if (grad_norm > max_grad_norm) {
                max_grad_norm = grad_norm;
            }
        }
        return max_grad_norm;
    };

    while (get_max_grad_norm() > 1e-10) {
        auto stats = scheduler.run_operation_on_all(op);
        REQUIRE(stats.number_of_successful_operations() > 0);
    }

    {
        for (const auto& tuple : mesh.get_all(PrimitiveType::Vertex)) {
            auto value = energy->get_value(Simplex(PrimitiveType::Vertex, tuple));
            auto grad = energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple));
            auto hess = energy->get_hessian(Simplex(PrimitiveType::Vertex, tuple));
            std::cout << "value = " << value << std::endl;
            std::cout << "grad = \n" << grad << std::endl;
            std::cout << "grad.norm() = " << grad.norm() << std::endl;
            std::cout << std::endl;
        }
    }

    ConstAccessor<double> pos = mesh.create_const_accessor<double>(handler);
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
    auto handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    function::AMIPS amips(mesh, handle);
    auto energy = std::make_shared<function::LocalNeighborsSumFunction>(mesh, handle, amips);
    OptimizationSmoothing op(mesh, energy);

    Scheduler scheduler;

    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &energy]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            double grad_norm = energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple)).norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    while (get_min_grad_norm() > 1e-10) {
        auto stats = scheduler.run_operation_on_all(op);
        REQUIRE(stats.number_of_successful_operations() > 0);
    }
}


TEST_CASE("smoothing_Gradient_Descent")
{
    DEBUG_TriMesh mesh = single_2d_nonequilateral_triangle_with_positions();
    auto handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto target_coordinate_handle =
        mesh.register_attribute<double>("target_coordinate", PrimitiveType::Vertex, 2);

    auto target_acc = mesh.create_accessor<double>(target_coordinate_handle);

    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 0)) << 0, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 1)) << 1, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 2)) << 0, 1;

    function::SquareDistance squared_dist(
        mesh,
        handle.as<double>(),
        target_coordinate_handle.as<double>());
    auto energy = std::make_shared<function::LocalNeighborsSumFunction>(mesh, handle, squared_dist);
    OptimizationSmoothing op(mesh, energy);

    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &energy]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            Eigen::Vector2d grad = energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple));
            double grad_norm = grad.norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    SchedulerStats stats;
    Scheduler scheduler;

    do {
        stats = scheduler.run_operation_on_all(op);
    } while (get_min_grad_norm() > 1e-3 && stats.number_of_successful_operations() > 0);
    ConstAccessor<double> pos = mesh.create_const_accessor<double>(handle);
    Tuple tuple = mesh.tuple_from_face_id(0);
    Eigen::Vector2d uv0 = pos.const_vector_attribute(tuple);
    Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh.switch_vertex(tuple));
    Eigen::Vector2d uv2 = pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(tuple)));


    // CHECK((uv0 - uv1).norm() - (uv1 - uv2).norm() < 1e-6);
    // CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() < 1e-6);
    // CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
}
