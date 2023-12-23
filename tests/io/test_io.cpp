#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/operations/EdgeSplit.hpp>

#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

#include <catch2/catch_test_macros.hpp>
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>


using namespace wmtk;
using namespace wmtk::tests;

namespace fs = std::filesystem;


constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;

TEST_CASE("hdf5_2d", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};

    TriMesh mesh;
    mesh.initialize(tris);

    HDF5Writer writer("hdf5_2d.hdf5");
    mesh.serialize(writer);
}

TEST_CASE("hdf5_2d_read", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};

    TriMesh mesh;
    mesh.initialize(tris);

    HDF5Writer writer("hdf5_2d_read.hdf5");
    mesh.serialize(writer);

    auto mesh1 = read_mesh("hdf5_2d_read.hdf5");

    CHECK(*mesh1 == mesh);
}

TEST_CASE("hdf5_rational", "[io]")
{
    Eigen::Matrix<long, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;
    TetMesh mesh;
    mesh.initialize(T);
    Eigen::Matrix<Rational, 8, 3> V;
    for (size_t i = 0; i < 8; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            V(i, j) = Rational(std::rand()) / Rational(std::rand());
        }
    }

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, mesh);

    HDF5Writer writer("hdf5_rational.hdf5");
    mesh.serialize(writer);

    auto mesh1 = read_mesh("hdf5_rational.hdf5");

    CHECK(*mesh1 == mesh);
}

TEST_CASE("paraview_2d", "[io]")
{
    auto mesh = read_mesh(WMTK_DATA_DIR "/fan.msh");

    ParaviewWriter writer("paraview_2d", "vertices", *mesh, true, true, true, false);
    mesh->serialize(writer);
}

TEST_CASE("hdf5_3d", "[io]")
{
    Eigen::Matrix<long, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;
    TetMesh mesh;
    mesh.initialize(T);

    HDF5Writer writer("hdf5_3d.hdf5");
    mesh.serialize(writer);
}

TEST_CASE("paraview_3d", "[io]")
{
    Eigen::Matrix<long, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;
    TetMesh mesh;
    mesh.initialize(T);
    Eigen::MatrixXd V(8, 3);
    V.setRandom();
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, mesh);

    ParaviewWriter writer("paraview_3d", "vertices", mesh, true, true, true, true);
    mesh.serialize(writer);
}

TEST_CASE("msh_3d", "[io]")
{
    auto mesh = read_mesh(WMTK_DATA_DIR "/sphere_delaunay.msh");
}

TEST_CASE("attribute_after_split", "[io][.]")
{
    DEBUG_TriMesh m = single_equilateral_triangle();
    auto attribute_handle = m.register_attribute<long>(std::string("test_attribute"), PE, 1);

    {
        // get the casted attribute types
        auto& split_strat = attribute_handle.trimesh_standard_split_strategy();
        auto& collapse_strat = attribute_handle.trimesh_standard_collapse_strategy();

        // set the strategies
        split_strat.set_standard_split_strategy(
            wmtk::operations::NewAttributeStrategy::SplitBasicStrategy::Copy);
        split_strat.set_standard_split_rib_strategy(
            wmtk::operations::NewAttributeStrategy::SplitRibBasicStrategy::CopyTuple);
        collapse_strat.set_standard_collapse_strategy(
            wmtk::operations::NewAttributeStrategy::CollapseBasicStrategy::CopyTuple);
    }

    wmtk::MeshAttributeHandle<double> pos_handle =
        m.get_attribute_handle<double>(std::string("vertices"), PV);

    {
        Accessor<long> acc_attribute = m.create_accessor<long>(attribute_handle);
        Accessor<double> acc_pos = m.create_accessor<double>(pos_handle);

        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 0);

        Eigen::Vector3d p_mid;
        {
            const Eigen::Vector3d p0 = acc_pos.vector_attribute(edge);
            const Eigen::Vector3d p1 = acc_pos.vector_attribute(m.switch_vertex(edge));
            p_mid = 0.5 * (p0 + p1);
        }

        {
            // set edge(0,1)'s tag as 1
            acc_attribute.scalar_attribute(edge) = 1;

            // all edges hold 0 besides "edge"
            for (const Tuple& t : m.get_all(PE)) {
                if (simplex::utils::SimplexComparisons::equal(
                        m,
                        Simplex::edge(edge),
                        Simplex::edge(t))) {
                    CHECK(acc_attribute.scalar_attribute(t) == 1);
                } else {
                    CHECK(acc_attribute.scalar_attribute(t) == 0);
                }
            }

            operations::EdgeSplit op(m);
            auto tmp = op(Simplex::edge(edge));
            REQUIRE(!tmp.empty());

            // set new vertex position
            acc_pos.vector_attribute(tmp.front().tuple()) = p_mid;
        }

        // since the default value is 0, there should be no other value in this triangle
        for (const Tuple& t : m.get_all(PE)) {
            CHECK(acc_attribute.scalar_attribute(t) == 0);
        }
    } // end of scope for the accessors

    {
        Accessor<long> acc_attribute = m.create_accessor<long>(attribute_handle);
        for (const Tuple& t : m.get_all(PE)) {
            CHECK(acc_attribute.scalar_attribute(t) == 0);
        }
    }

    // attribute_after_split_edges.hdf contains a 1 in the "test_attribute"
    ParaviewWriter writer("attribute_after_split", "vertices", m, true, true, true, false);
    m.serialize(writer);
}
