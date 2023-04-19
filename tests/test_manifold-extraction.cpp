#include <catch2/catch.hpp>
#include <wmtk/utils/Manifold-extraction.hpp>
#include <wmtk/utils/Delaunay.hpp>


TEST_CASE("Manifold-Extraction2D", "[man-ext2d]"){
    
    using namespace wmtk;
    size_t nb_points = 10;
    size_t nb_triangles;
    size_t loop = 10;
    const std::vector<Point2D>* points = pntgen2d(nb_points, 10.0);  // generate nb_points of random points
    auto [vertices, triangles] = delaunay2D(*points); // do Delaunay on them, output vertices and triangles
    nb_triangles = triangles.size();
    std::vector<std::vector<size_t>*>* tagass_arr = new std::vector<std::vector<size_t>*>(loop);
    for (size_t i = 0 ; i < loop; i++){
        tagass_arr->assign(i, tagassign(nb_triangles)); // assign tags to triangles, only keep the inside ones
    }

    SECTION("Number check"){
        REQUIRE(points->size() == nb_points);
    }

    SECTION("Work together delaunay"){
        REQUIRE(vertices.size() == nb_points);
        REQUIRE(triangles.size() == 12);
    }

    SECTION("Tag assignment map num check"){
        for (size_t i = 0; i < loop; i+=1){
            REQUIRE(tagass_arr->at(i)->size() <= nb_triangles);
        }
    }
}

TEST_CASE("Manifold-Extraction3D", "[man-ext3d]"){
    using namespace wmtk;
    size_t nb_points = 10;
    size_t nb_triangles;
    size_t loop = 10;
    const std::vector<Point3D>* points = pntgen3d(nb_points, 10.0);  // generate nb_points of random points
    auto [vertices, triangles] = delaunay3D(*points); // do Delaunay on them, output vertices and triangles
    nb_triangles = triangles.size();
    std::vector<std::vector<size_t>*>* tagass_arr = new std::vector<std::vector<size_t>*>(loop);
    for (size_t i = 0 ; i < loop; i++){
        tagass_arr->assign(i, tagassign(nb_triangles)); // assign tags to triangles, only keep the inside ones
    }
    SECTION("Number check"){
        REQUIRE(points->size() == nb_points);
    }

    SECTION("Work together delaunay"){
        REQUIRE(vertices.size() == nb_points);
        REQUIRE(triangles.size() == 19);
    }

    SECTION("Tag assignment map num check"){
        for (size_t i = 0; i < loop; i++){
            REQUIRE(tagass_arr->at(i)->size() <= nb_triangles);
        }
    }
}
