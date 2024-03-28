#include <cassert>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/DEBUG_Tuple.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureMapAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/utils/Logger.hpp>

namespace image = wmtk::components::image;
namespace ATfunction = wmtk::components::function;
namespace tests = wmtk::tests;
using namespace Eigen;

namespace wmtk {
TEST_CASE("sinxcosy over unit square")
{
    tests::DEBUG_TriMesh mesh = tests::unit_squre();
    std::array<std::shared_ptr<image::Image>, 3> images = {
        {std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500)}};
    auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        // return u * u + v * v;
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0]->set(u);
    images[1]->set(v);
    images[2]->set(height_function);
    ATfunction::utils::ThreeChannelPositionMapEvaluator evaluator(images);
    ATfunction::utils::TextureMapAvgDistanceToLimit texture_integral(evaluator);

    std::array<Tuple, 2> triangles = {mesh.tuple_from_face_id(0), mesh.tuple_from_face_id(1)};

    wmtk::attribute::MeshAttributeHandle uv_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    ConstAccessor<double> uv = mesh.create_const_accessor(uv_handle.as<double>());


    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;
    double error = 0.0;
    for (const auto& triangle : triangles) {
        // get coordinates of triangle 3 vertices
        Vector2d uv0 = uv.const_vector_attribute(triangle);
        Vector2d uv1 = uv.const_vector_attribute(mesh.switch_tuples(triangle, {PV}));
        Vector2d uv2 = uv.const_vector_attribute(mesh.switch_tuples(triangle, {PE, PV}));

        double tri_error = texture_integral.distance(uv0, uv1, uv2);
        error += tri_error;
    }

    // the analytical solve for
    // (sin(2 * M_PI * u) * cos(2 * M_PI * v))^2 with x in (0,1), y in (0,1) = 0.25
    REQUIRE(pow(error - 0.25, 2) < 1e-4);
}
TEST_CASE("analytical quadrature")
{
    tests::DEBUG_TriMesh mesh = tests::unit_squre();

    std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs = {
        {std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             1,
             0,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             0,
             1,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Periodic,
             2,
             2,
             1.)}

    };

    ATfunction::utils::ThreeChannelPositionMapEvaluator evaluator(funcs);
    ATfunction::utils::AnalyticalFunctionAvgDistanceToLimit analy_quad(evaluator);

    std::array<Tuple, 2> triangles = {mesh.tuple_from_face_id(0), mesh.tuple_from_face_id(1)};

    wmtk::attribute::MeshAttributeHandle uv_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    ConstAccessor<double> uv = mesh.create_const_accessor(uv_handle.as<double>());


    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;
    double error = 0.0;
    for (const auto& triangle : triangles) {
        // get coordinates of triangle 3 vertices
        Vector2d uv0 = uv.const_vector_attribute(triangle);
        Vector2d uv1 = uv.const_vector_attribute(mesh.switch_tuples(triangle, {PV}));
        Vector2d uv2 = uv.const_vector_attribute(mesh.switch_tuples(triangle, {PE, PV}));

        double tri_error = analy_quad.distance(uv0, uv1, uv2);
        std::cout << "tri_error " << tri_error << std::endl;
        error += tri_error;
    }
    std::cout << "error " << error << std::endl;
    // the analytical solve for
    // (sin(2 * M_PI * u) * cos(2 * M_PI * v))^2 with x in (0,1), y in (0,1) = 0.25
    REQUIRE(pow(error - 0.25, 2) < 1e-4);
}
} // namespace wmtk