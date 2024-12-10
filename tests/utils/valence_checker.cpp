#include <catch2/catch_test_macros.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/utils/verify_simplex_index_valences.hpp>

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
using namespace wmtk;

TEST_CASE("simplex_valences", "[simplex_valences]")
{
    {
        // single tri
        wmtk::TriMesh t;
        RowVectors3l tris;
        tris.resize(1, 3);
        tris.row(0) << 0, 1, 2;

        t.initialize(tris);
        CHECK(wmtk::utils::verify_simplex_index_valences(t));
    }
    {
        // manifold edg emeet
        wmtk::TriMesh t;
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) << 0, 1, 2;
        tris.row(1) << 0, 2, 3;

        t.initialize(tris);
        CHECK(wmtk::utils::verify_simplex_index_valences(t));
    }
    {
        // duplicated simplex
        wmtk::TriMesh t;
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) << 0, 1, 2;
        tris.row(1) << 0, 2, 1;

        t.initialize(tris);
        CHECK(!wmtk::utils::verify_simplex_index_valences(t));
    }
    {
        // nonmanifold vertex
        wmtk::TriMesh t;
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) << 0, 1, 2;
        tris.row(1) << 0, 3, 4;
        t.initialize(tris);
        CHECK(!wmtk::utils::verify_simplex_index_valences(t));
    }

    {
        // single tet
        wmtk::TetMesh t;
        RowVectors4l tets;
        tets.resize(1, 4);
        tets.row(0) << 0, 1, 2, 3;
        t.initialize(tets);

        CHECK(wmtk::utils::verify_simplex_index_valences(t));
    }
    {
        // duplicated simplex
        wmtk::TetMesh t;
        RowVectors4l tets;
        tets.resize(2, 4);
        tets.row(0) << 0, 1, 2, 3;
        tets.row(1) << 0, 1, 3, 2;
        t.initialize(tets);

        CHECK(!wmtk::utils::verify_simplex_index_valences(t));
    }
    {
        // manifold face meet
        wmtk::TetMesh t;
        RowVectors4l tets;
        tets.resize(2, 4);
        tets.row(0) << 0, 1, 2, 3;
        tets.row(1) << 0, 1, 2, 4;
        t.initialize(tets);

        CHECK(wmtk::utils::verify_simplex_index_valences(t));
    }
    {
        // edge nonmanifold
        wmtk::TetMesh t;
        RowVectors4l tets;
        tets.resize(2, 4);
        tets.row(0) << 0, 1, 2, 3;
        tets.row(1) << 0, 1, 5, 4;
        t.initialize(tets);

        CHECK(!wmtk::utils::verify_simplex_index_valences(t));
    }
    {
        // vertex nonmanifolld
        wmtk::TetMesh t;
        RowVectors4l tets;
        tets.resize(2, 4);
        tets.row(0) << 0, 1, 2, 3;
        tets.row(1) << 0, 5, 6, 4;
        t.initialize(tets);

        CHECK(!wmtk::utils::verify_simplex_index_valences(t));
    }
}
