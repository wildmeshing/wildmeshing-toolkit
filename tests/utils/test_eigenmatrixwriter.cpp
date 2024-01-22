#include <catch2/catch_test_macros.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/EigenMatrixWriter.cpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

const std::filesystem::path data_dir = WMTK_DATA_DIR;

using namespace wmtk::utils;
using namespace wmtk::tests;
using namespace wmtk::tests_3d;
using namespace wmtk;

TEST_CASE("test_eigenmatrixwriter", "[eigenmatrixwriter][.]")
{
    SECTION("TriMesh")
    {
        DEBUG_TriMesh m = hex_plus_two_with_position();
        EigenMatrixWriter writer;
        m.serialize(writer);

        MatrixX<double> V;
        writer.get_double_matrix("vertices", PrimitiveType::Vertex, V);

        // std::cout << V << std::endl;
        CHECK(V.rows() == 9);
        CHECK(V.cols() == 3);

        MatrixX<int64_t> FV;
        writer.get_int64_t_matrix("m_fv", PrimitiveType::Face, FV);

        // std::cout << FV << std::endl;
        CHECK(FV.rows() == 8);
        CHECK(FV.cols() == 3);

        MatrixX<double> Vs;
        writer.get_position_matrix(Vs);
        // std::cout << Vs << std::endl;
        CHECK(Vs.rows() == 9);
        CHECK(Vs.cols() == 3);

        MatrixX<int64_t> FVs;
        writer.get_FV_matrix(FVs);
        // std::cout << FVs << std::endl;
        CHECK(FVs.rows() == 8);
        CHECK(FVs.cols() == 3);
    }
    SECTION("TetMesh")
    {
        DEBUG_TetMesh m = three_incident_tets_with_positions();
        EigenMatrixWriter writer;
        m.serialize(writer);

        MatrixX<double> V;
        writer.get_double_matrix("vertices", PrimitiveType::Vertex, V);

        // std::cout << V << std::endl;
        CHECK(V.rows() == 6);
        CHECK(V.cols() == 3);

        MatrixX<int64_t> TV;
        writer.get_int64_t_matrix("m_tv", PrimitiveType::Tetrahedron, TV);

        // std::cout << TV << std::endl;
        CHECK(TV.rows() == 3);
        CHECK(TV.cols() == 4);

        MatrixX<double> Vs;
        writer.get_position_matrix(Vs);
        // std::cout << Vs << std::endl;
        CHECK(Vs.rows() == 6);
        CHECK(Vs.cols() == 3);

        MatrixX<int64_t> TVs;
        writer.get_TV_matrix(TVs);
        // std::cout << TVs << std::endl;
        CHECK(TVs.rows() == 3);
        CHECK(TVs.cols() == 4);
    }
}