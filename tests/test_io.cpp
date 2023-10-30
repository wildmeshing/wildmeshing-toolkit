#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <catch2/catch_test_macros.hpp>


using namespace wmtk;

TEST_CASE("hdf5_2d", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};

    TriMesh mesh;
    mesh.initialize(tris);

    HDF5Writer writer("test.hdf5");
    mesh.serialize(writer);
}

TEST_CASE("hdf5_2d_read", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};

    TriMesh mesh;
    mesh.initialize(tris);

    HDF5Writer writer("test.hdf5");
    mesh.serialize(writer);

    auto mesh1 = MeshReader::read("test.hdf5");

    CHECK(*mesh1 == mesh);
}

TEST_CASE("paraview_2d", "[io]")
{
    auto mesh = MeshReader::read(WMTK_DATA_DIR "/fan.msh");

    ParaviewWriter writer("paraview", "vertices", *mesh, true, true, true, false);
    mesh->serialize(writer);
}

TEST_CASE("hdf5_3d", "[io]")
{
    Eigen::Matrix<long, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;
    TetMesh mesh;
    mesh.initialize(T);

    HDF5Writer writer("test.hdf5");
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

    ParaviewWriter writer("paraview", "vertices", mesh, true, true, true, true);
    mesh.serialize(writer);
}

TEST_CASE("msh_3d", "[io]")
{
    auto mesh = MeshReader::read(WMTK_DATA_DIR "/sphere_delaunay.msh");
}
