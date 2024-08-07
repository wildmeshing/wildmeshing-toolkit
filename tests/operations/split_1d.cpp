
#include <catch2/catch_test_macros.hpp>
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/is_free.hpp"

using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::tests;
using namespace operations;


TEST_CASE("split_no_topology_edgemesh", "[operations][split]")
{
    const int64_t initial_size = 20;
    DEBUG_EdgeMesh m = [](int64_t size) {
        EdgeMesh m;
        m.initialize_free(size);
        return m;
    }(initial_size);
    int64_t size = initial_size;
    for (Tuple edge : m.get_all(PrimitiveType::Edge)) {
        EdgeSplit op(m);
        REQUIRE(!op(simplex::Simplex(m, PrimitiveType::Edge, edge)).empty());
        size++;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(is_free(m));
        CHECK(m.get_all(PrimitiveType::Edge).size() == size);
    }
}
