#include <cassert>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/DEBUG_Tuple.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/function/utils/TextureIntegral.hpp>
#include <wmtk/utils/Logger.hpp>

namespace image = wmtk::components::adaptive_tessellation::image;
namespace function = wmtk::function;
namespace tests = wmtk::tests;
using namespace Eigen;
namespace wmtk {
TEST_CASE("sinxcosy over unit square")
{
    tests::DEBUG_TriMesh mesh = tests::unit_squre();
    std::array<image::Image, 3> images = {
        image::Image(500, 500),
        image::Image(500, 500),
        image::Image(500, 500)};
    auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        // return u * u + v * v;
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0].set(u);
    images[1].set(v);
    images[2].set(height_function);
    function::utils::ThreeChannelPositionMapEvaluator evaluator(images);
    function::utils::TextureIntegral texture_integral(evaluator);

    std::array<Tuple, 2> triangles = {mesh.tuple_from_face_id(0), mesh.tuple_from_face_id(1)};

    MeshAttributeHandle<double> uv_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    ConstAccessor<double> uv = mesh.create_const_accessor(uv_handle);


    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;
    double error = 0.0;
    for (const auto& triangle : triangles) {
        // get coordinates of triangle 3 vertices
        Vector2d uv0 = uv.const_vector_attribute(triangle);
        Vector2d uv1 = uv.const_vector_attribute(mesh.switch_tuples(triangle, {PV}));
        Vector2d uv2 = uv.const_vector_attribute(mesh.switch_tuples(triangle, {PE, PV}));

        double tri_error = texture_integral.get_error_one_triangle_exact<double>(uv0, uv1, uv2);
        error += tri_error;
    }

    // the analytical solve for
    // (sin(2 * M_PI * u) * cos(2 * M_PI * v))^2 with x in (0,1), y in (0,1) = 0.25
    REQUIRE(pow(error - 0.25, 2) < 1e-4);
}
} // namespace wmtk