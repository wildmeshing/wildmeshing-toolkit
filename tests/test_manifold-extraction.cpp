#include <catch2/catch.hpp>
#include <wmtk/utils/Manifold-extraction.hpp>
#include <wmtk/utils/Delaunay.hpp>


TEST_CASE("Manifold-Extraction2D", "[man-ext2d]"){
    
    using namespace wmtk;
    const std::vector<Point2D>& points = pntgen();
    auto [vertices, triangles] = delaunay2D(points);
    std::map<size_t, size_t>& tagass = tagassign(triangles);

    SECTION("Number check"){
        REQUIRE(points.size() == 10);
    }

    SECTION("Work together delaunay"){
        REQUIRE(vertices.size() == 10);
        REQUIRE(triangles.size() == 12);
    }

    SECTION("Tag assignment map num check"){
        REQUIRE(tagass.size() == triangles.size());
    }
}

TEST_CASE("Manifold-Extraction3D", "[man-ext3d]"){}
