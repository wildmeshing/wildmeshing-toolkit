#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/isotropic_remeshing/IsotropicRemeshingOptions.hpp>
#include <spdlog/spdlog.h>
#include <tools/TriMesh_examples.hpp>
TEST_CASE("component_isotropic_remeshing", "[components][isotropic_remeshing]")
{

    auto mptr = std::make_shared<wmtk::TriMesh>(wmtk::tests::ten_triangles_with_position(3));


    wmtk::components::isotropic_remeshing::IsotropicRemeshingOptions opts;
    opts.position_attribute = mptr->get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);

    opts.edge_swap_mode = wmtk::components::isotropic_remeshing::EdgeSwapMode::Valence;

    opts.envelope_size = 1e-3;
    opts.length_rel = 1e-1;
    opts.fix_uv_seam = false;

    wmtk::components::isotropic_remeshing::isotropic_remeshing(opts);
    spdlog::info("hi");
}
