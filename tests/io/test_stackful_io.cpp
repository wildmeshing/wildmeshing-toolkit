#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include "../tools/DEBUG_PointMesh.hpp"

#include <catch2/catch_test_macros.hpp>


using namespace wmtk;
using namespace wmtk::tests;

namespace fs = std::filesystem;


constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;


TEST_CASE("hdf5_in_transit", "[io]")
{
    int64_t size = 20;
    DEBUG_PointMesh mesh(size);
    REQUIRE(size == mesh.capacity(wmtk::PrimitiveType::Vertex));

    auto double_handle = mesh.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3).as<double>();

    HDF5Writer writer("hdf5_in_transit_begin.hdf5");
    mesh.serialize(writer);

    auto& double_acc = mesh.create_index_accessor(double_handle);
    {
        auto scope = mesh.create_scope();

        {
            for (int j = 0; j < 20; ++j) {
                double_acc.vector_attribute(j).setConstant(j);
            }
            {
                HDF5Writer writer("hdf5_in_transit_mid.hdf5");
                mesh.serialize(writer);
            }
            auto m2ptr = read_mesh("hdf5_in_transit_mid.hdf5");
            DEBUG_PointMesh& m2 = reinterpret_cast<DEBUG_PointMesh&>(*std::dynamic_pointer_cast<PointMesh>(m2ptr));


            auto double_handle2 =
                m2.get_attribute_handle<double>("double", wmtk::PrimitiveType::Vertex).as<double>();
            auto& double_acc2 = m2.create_index_accessor(double_handle2);
            for (int j = 0; j < 20; ++j) {
                CHECK(double_acc2.vector_attribute(j) == double_acc.vector_attribute(j));
            }
            auto minitptr = read_mesh("hdf5_in_transit_begin.hdf5");
            DEBUG_PointMesh& minit= reinterpret_cast<DEBUG_PointMesh&>(*std::dynamic_pointer_cast<PointMesh>(minitptr));
            CHECK(!(minit == m2));
        }
    }
    {
        HDF5Writer writer("hdf5_in_transit_end.hdf5");
        mesh.serialize(writer);
    }


    auto mesh1 = read_mesh("hdf5_in_transit_mid.hdf5");
    auto mesh2 = read_mesh("hdf5_in_transit_end.hdf5");

    CHECK(*mesh1 == *mesh2);
}
