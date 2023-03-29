#include <catch2/catch.hpp>
#include <wmtk/utils/Manifold-extraction.hpp>

TEST_CASE( "10 Random points are generated", "[man-ext]"){
        
    using namespace wmtk;

    const std::vector<Point2D>& points = pntgen();
    REQUIRE(points.size() == 100);
}
