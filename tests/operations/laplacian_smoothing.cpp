#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>
#include "tools/TriMesh_examples.hpp"

TEST_CASE("test_hybrid_laplacian_smoothing", "[scheduler][2D]")
{
    // note that we return the trimesh because even though teh meshattributehandle holds a ref to
    // the mesh it's not an owning ref so we need to make sure it stays in scope
    auto make_disk = [](int64_t size, bool use_rational)
        -> std::tuple<std::shared_ptr<wmtk::TriMesh>, wmtk::attribute::MeshAttributeHandle> {
        auto disk = wmtk::tests::disk(size);
        auto double_vertex_handle =
            disk->register_attribute<double>("vertices", wmtk::PrimitiveType::Vertex, 2);

        int64_t ring_size = size - 1;

        auto vertices = disk->get_all(wmtk::PrimitiveType::Vertex);
        auto acc = disk->create_accessor<double, 2>(double_vertex_handle);
        acc.vector_attribute(vertices[0]).setConstant(0);

        const double dt = 2 * M_PI / ring_size;
        for (int j = 0; j < ring_size; ++j) {
            double theta = j * dt;
            auto v = acc.vector_attribute(vertices[j + 1]);

            v.x() = std::cos(theta);
            v.y() = std::sin(theta);
        }


        if (!use_rational) {
            return std::make_tuple(disk, double_vertex_handle);
        } else {
            auto hybrid_handle =
                wmtk::attribute::utils::HybridRationalAttribute<>::register_attribute_from_double(
                    *disk,
                    double_vertex_handle.as<double>(),
                    {});
            auto mesh_handle = wmtk::attribute::MeshAttributeHandle(*disk, hybrid_handle);
            return std::make_tuple(disk, mesh_handle);
        }
    };


    constexpr static int DISK_SIZE = 5;

    {
        auto [disk, hybrid_handle] = make_disk(DISK_SIZE, true);

        wmtk::attribute::utils::HybridRationalAccessor<-1, wmtk::Mesh> acc(hybrid_handle);
        // wmtk::attribute::utils::HybridRationalAccessor acc(hybrid_handle);
        {
            auto vertices = disk->get_all(wmtk::PrimitiveType::Vertex);
            for (const auto& vtup : vertices) {
                auto [c, r, d] = acc.value(vtup);
                REQUIRE(c.size() == 2);
                REQUIRE(r.size() == 2);
                REQUIRE(d.size() == 2);
                CHECK((c.array() == 1).all());
                for (int j = 0; j < 2; ++j) {
                    auto dv = d(j);
                    auto rv = r(j);
                    CHECK(dv == rv.to_double());
                }
            }
        }
        // lets just mmake sure the tuples returned by get_all are consistent is consistent at this
        // point
        auto [disk_double, double_handle] = make_disk(DISK_SIZE, false);
        REQUIRE(
            disk->get_all(wmtk::PrimitiveType::Vertex) ==
            disk_double->get_all(wmtk::PrimitiveType::Vertex));
        // this should be the arg used to create teh disk mesh
        REQUIRE(disk->get_all(wmtk::PrimitiveType::Vertex).size() == DISK_SIZE + 1);
    }
    { // test that smoothing a disk and double do the same thing up to flop
        auto [disk, hybrid_handle] = make_disk(DISK_SIZE, true);
        auto [disk_double, double_handle] = make_disk(DISK_SIZE, false);

        wmtk::operations::VertexLaplacianSmooth double_vsmooth(double_handle);

        wmtk::operations::VertexLaplacianSmooth hybrid_vsmooth(hybrid_handle);

        // can use this tuple on both cuz of aforemention checking on tuples
        const wmtk::Tuple first_vertex_tuple = disk->get_all(wmtk::PrimitiveType::Vertex)[0];
        const auto first_vertex = wmtk::simplex::Simplex::vertex(*disk, first_vertex_tuple);

        double_vsmooth(*disk_double, first_vertex);
        hybrid_vsmooth(*disk, first_vertex);

        {
            auto double_acc = disk_double->create_accessor<double, 2>(double_handle);
            wmtk::attribute::utils::HybridRationalAccessor acc(hybrid_handle);
            auto vertices = disk->get_all(wmtk::PrimitiveType::Vertex);
            for (const auto& vtup : vertices) {
                acc.round(vtup);
                auto [c, r, d] = acc.const_value(vtup);
                auto dd = double_acc.const_vector_attribute(vtup);
                REQUIRE(c.size() == 2);
                REQUIRE(r.size() == 2);
                REQUIRE(d.size() == 2);
                CHECK((c.array() == 1).all());
                for (int j = 0; j < 2; ++j) {
                    auto dv = d(j);
                    auto rv = r(j);
                    CHECK(dv == rv.to_double());
                }
                CHECK((dd - d).norm() < 1e-10);
            }
        }
    }

    // auto hybrid_handle = wmtk::attribute::utils::HybridRationalAttribute<>::register_attribute(
    //     *disk,
    //     "scalar",
    //     wmtk::PrimitiveType::Vertex);
}
