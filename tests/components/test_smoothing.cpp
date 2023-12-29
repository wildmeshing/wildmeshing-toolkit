#include <catch2/catch_test_macros.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/function/AMIPS.hpp>
#include <wmtk/function/LocalDifferentiableFunction.hpp>
#include <wmtk/function/PerSimplexDifferentiableFunction.hpp>
#include <wmtk/function/SYMDIR.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::operations;

// this is a test energy class for mapping one triangle to another triangle
namespace wmtk::function {
class SquareDistance : public TriangleAutodiffFunction
{
public:
    SquareDistance(
        const TriMesh& mesh,
        const attribute::MeshAttributeHandle<double>& attribute_handle,
        const attribute::MeshAttributeHandle<double>& target_attribute_handle)
        : TriangleAutodiffFunction(mesh, attribute_handle)
        , m_target_attribute_accessor(mesh.create_const_accessor(target_attribute_handle))
    {}
    ~SquareDistance() override = default;
    using DScalar = AutodiffFunction::DScalar;
    using DSVec = Eigen::VectorX<DScalar>;

protected:
    DScalar eval(const simplex::Simplex& domain_simplex, const std::array<DSVec, 3>& coordinates)
        const override
    {
        auto target_coordinates =
            get_coordinates(m_target_attribute_accessor, domain_simplex.tuple());
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
    OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy> op_settings(mesh);
    op_settings.coordinate_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    op_settings.smooth_boundary = true;
    op_settings.second_order = true;
    op_settings.line_search = false;
    op_settings.step_size = 1;
    std::shared_ptr<function::SYMDIR> per_tri_energy = std::make_shared<function::SYMDIR>(
        mesh,
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));
    op_settings.energy = std::make_unique<function::LocalDifferentiableFunction>(per_tri_energy);

    Scheduler scheduler(mesh);

    auto& factory =
        scheduler.add_operation_type<operations::tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
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

    std::cout << (uv0 - uv1).norm() << std::endl;
    std::cout << (uv0 - uv2).norm() << std::endl;
    std::cout << (uv1 - uv2).norm() << std::endl;

    std::cout << per_tri_energy->get_energy_avg() << std::endl;
    std::cout << per_tri_energy->get_energy_max() << std::endl;
    CHECK((uv0 - uv1).norm() - (uv1 - uv2).norm() < 1e-6);
    CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() < 1e-6);
    CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
}


TEST_CASE("smoothing_Gradient_Descent", "[.][slow]")
{
    DEBUG_TriMesh mesh = single_2d_nonequilateral_triangle_with_positions();
    OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy> op_settings(mesh);
    op_settings.coordinate_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto target_coordinate_handle =
        mesh.register_attribute<double>("target_coordinate", PrimitiveType::Vertex, 2);

    auto target_acc = mesh.create_accessor(target_coordinate_handle);

    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 0)) << 0, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 1)) << 1, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 2)) << 0, 1;

    op_settings.smooth_boundary = true;
    op_settings.second_order = false;
    op_settings.line_search = false;
    op_settings.step_size = 1e-1;
    std::shared_ptr<function::SquareDistance> per_tri_amips =
        std::make_shared<function::SquareDistance>(
            mesh,
            op_settings.coordinate_handle,
            target_coordinate_handle);
    op_settings.energy = std::make_unique<function::LocalDifferentiableFunction>(per_tri_amips);

    Scheduler scheduler(mesh);

    auto& factory =
        scheduler.add_operation_type<operations::tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
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
