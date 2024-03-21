#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/MshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/operations/EdgeSplit.hpp>

#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

#include <catch2/catch_test_macros.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>


using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::tests;

namespace fs = std::filesystem;

const fs::path wmtk_data_dir = WMTK_DATA_DIR;


constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;

TEST_CASE("hdf5_2d", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};

    TriMesh mesh;
    mesh.initialize(tris);

    HDF5Writer writer("hdf5_2d.hdf5");
    mesh.serialize(writer);
}

TEST_CASE("hdf5_2d_read", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};

    TriMesh mesh;
    mesh.initialize(tris);

    HDF5Writer writer("hdf5_2d_read.hdf5");
    mesh.serialize(writer);

    auto mesh1 = read_mesh("hdf5_2d_read.hdf5");

    CHECK(*mesh1 == mesh);
}

TEST_CASE("hdf5_rational", "[io]")
{
    Eigen::Matrix<int64_t, 2, 4> T;
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
    Eigen::Matrix<int64_t, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;
    TetMesh mesh;
    mesh.initialize(T);

    HDF5Writer writer("hdf5_3d.hdf5");
    mesh.serialize(writer);
}

TEST_CASE("hdf5_multimesh", "[io]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::shared_ptr<DEBUG_TriMesh> child00_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;
    auto& child2 = *child2_ptr;
    auto& child00 = *child00_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});
    auto child2_map = multimesh::same_simplex_dimension_surjection(parent, child2, {0, 1, 2});
    auto child00_map = multimesh::same_simplex_dimension_surjection(child0, child00, {0});

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);
    parent.register_child_mesh(child2_ptr, child2_map);

    child0.register_child_mesh(child00_ptr, child00_map);

    HDF5Writer writer("hdf5_multimesh.hdf5");
    parent.serialize(writer);

    auto mesh = read_mesh("hdf5_multimesh.hdf5");

    CHECK(*mesh == parent);
}

TEST_CASE("paraview_3d", "[io]")
{
    Eigen::Matrix<int64_t, 2, 4> T;
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
    CHECK_NOTHROW(read_mesh(wmtk_data_dir / "sphere_delaunay.msh"));
}

TEST_CASE("msh_3d_with_tet_attribute", "[io]")
{
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    MshReader reader;
    std::vector<std::string> attrs = {"in/out", "min_dihedral_angle", "energy"};
    std::shared_ptr<Mesh> mesh = reader.read(wmtk_data_dir / "tetwild_fig8_mid.msh", false, attrs);

    CHECK(mesh != nullptr);
    CHECK(mesh->top_simplex_type() == PT);

    auto in_out_handle = mesh->get_attribute_handle<double>("in/out", PT);
    auto min_dih_handle = mesh->get_attribute_handle<double>("min_dihedral_angle", PT);
    auto energy_handle = mesh->get_attribute_handle<double>("energy", PT);
    CHECK(in_out_handle.is_valid());
    CHECK(min_dih_handle.is_valid());
    CHECK(energy_handle.is_valid());

    if (false) {
        ParaviewWriter
            writer("middle_mesh_with_in_out_attr", "vertices", *mesh, true, true, true, true);
        mesh->serialize(writer);
    }
}

TEST_CASE("msh_3d_convert_tetwild_to_wmtk", "[io][.]")
{
    // This is an example showing how to convert a tetwild msh file into our wmtk format and transfering the in/out attribute into a tag attribute.

    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    const std::string mesh_name = "tetwild_fig8_mid";

    std::shared_ptr<Mesh> mesh = read_mesh(wmtk_data_dir / (mesh_name + ".msh"), false, {"in/out"});

    CHECK(mesh != nullptr);
    CHECK(mesh->top_simplex_type() == PT);

    {
        auto in_out_handle = mesh->get_attribute_handle<double>("in/out", PT);
        auto in_out_acc = mesh->create_accessor<double>(in_out_handle);

        auto tag_handle = mesh->register_attribute<int64_t>("tag", PT, 1);
        auto tag_acc = mesh->create_accessor<int64_t>(tag_handle);

        for (const Tuple& t : mesh->get_all(PT)) {
            if (in_out_acc.const_scalar_attribute(t) < 0.5) {
                tag_acc.scalar_attribute(t) = 1;
            }
        }

        auto pos_handle = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        mesh->clear_attributes({tag_handle, pos_handle});
    }

    io::Cache cache("wmtk_cache", ".");
    cache.write_mesh(*mesh, mesh_name);
    cache.export_cache(mesh_name + "_converted_from_tetwild");

    if (true) {
        ParaviewWriter writer(mesh_name, "vertices", *mesh, true, true, true, true);
        mesh->serialize(writer);
    }
}

TEST_CASE("attribute_after_split", "[io][.]")
{
    DEBUG_TriMesh m = single_equilateral_triangle();
    auto attribute_handle =
        m.register_attribute<int64_t>(std::string("test_attribute"), PE, 1).as<int64_t>();

    wmtk::attribute::TypedAttributeHandle<double> pos_handle =
        m.get_attribute_handle<double>(std::string("vertices"), PV).as<double>();

    {
        wmtk::attribute::Accessor<int64_t> acc_attribute =
            m.create_accessor<int64_t>(attribute_handle);
        wmtk::attribute::Accessor<double> acc_pos = m.create_accessor<double>(pos_handle);

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

            {
                // set the strategies
                op.set_new_attribute_strategy(
                    wmtk::attribute::MeshAttributeHandle(m, attribute_handle),
                    wmtk::operations::SplitBasicStrategy::Copy,
                    wmtk::operations::SplitRibBasicStrategy::CopyTuple);
            }

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
        wmtk::attribute::Accessor<int64_t> acc_attribute =
            m.create_accessor<int64_t>(attribute_handle);
        for (const Tuple& t : m.get_all(PE)) {
            CHECK(acc_attribute.scalar_attribute(t) == 0);
        }
    }

    // attribute_after_split_edges.hdf contains a 1 in the "test_attribute"
    ParaviewWriter writer("attribute_after_split", "vertices", m, true, true, true, false);
    m.serialize(writer);
}
