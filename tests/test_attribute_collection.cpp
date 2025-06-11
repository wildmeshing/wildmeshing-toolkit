#include <catch2/catch_test_macros.hpp>
#include <wmtk/AttributeCollection.hpp>
TEST_CASE("attribute_collection", "[attribute_collection]")
{
    // try writing
    // try reading
    // try protecting -> writing -> flushing
    // try protecting -> writing -> rolling back
}

TEST_CASE("attribute_collection_raii", "[attribute_collection]")
{
    // try protecting -> writing -> flushing
    // try protecting -> writing -> rolling back
    // try transferring ownership using constructor
    // try transferring ownership using assignment (empty)
    // try transferring ownership using assignment (already holding something)
}
