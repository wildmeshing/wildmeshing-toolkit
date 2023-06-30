#include <catch2/catch.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/TriMeshOperations.hpp>

using namespace wmtk;


TEST_CASE("get per face data") {}
TEST_CASE("delete simplices") {}
TEST_CASE("operation state") {}
TEST_CASE("glue ear to face") {}

//////////// SPLIT TESTS ////////////
TEST_CASE("glue new faces across AB")
{
    // test the assumption of correct orientation
    // new face correspondance accross AB
}

TEST_CASE("glue new triangle topology") {}

TEST_CASE("simplices to delete for split") {}

//////////// COLLAPSE TESTS ////////////
TEST_CASE("2D link condition for collapse") {}