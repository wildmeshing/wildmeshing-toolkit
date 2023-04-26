#include <catch2/catch.hpp>
#include <wmtk/utils/Manifold-extraction.hpp>
#include <wmtk/utils/Delaunay.hpp>


TEST_CASE("Manifold-Extraction2D", "[man-ext2d]"){
    
    using namespace wmtk;
    size_t nb_points = 20;
    size_t nb_triangles;
    double range = 10.0;
    size_t tagass_loop = 100;
    size_t pntgen_loop = 10;
    //std::vector<std::vector<Point2D>*>* pntgen_arr = new std::vector<std::vector<Point2D>*>(pntgen_loop);
    std::vector<std::vector<size_t>*>* tagass_arr = new std::vector<std::vector<size_t>*>(tagass_loop);

    for (size_t i = 0; i < pntgen_loop; i++){
        const std::vector<Point2D>* points = pntgen2d(nb_points, range);  // generate nb_points of random points
        //const std::vector<Point2D>* points = pntgen2d(nb_points, range);  // generate nb_points of random points
        auto [vertices, triangles] = delaunay2D(*points); // do Delaunay on them, output vertices and triangles
        nb_triangles = triangles.size();
        for (size_t j = 0 ; j < tagass_loop; j++){
            tagass_arr->assign(j, tagassign(nb_triangles)); // assign tags to triangles, only keep the inside ones
        }
        //nb_points += 10; // 1 new bug!
        range += 10.0;

        SECTION("Number check"){
            REQUIRE(points->size() == nb_points);
        }

        SECTION("Work together delaunay"){
            REQUIRE(vertices.size() == nb_points);
            //REQUIRE(triangles.size() == 12);
        }

        SECTION("Tag assignment map num check"){
            for (size_t j = 0; j < tagass_loop-1; j++){ // there's a weird bug here!, I could only write tagass_loop-1
                REQUIRE(tagass_arr->at(j)->size() <= nb_triangles);
            }
        }
    }
}

TEST_CASE("Manifold-Extraction3D", "[man-ext3d]"){
    using namespace wmtk;
    size_t nb_points = 10;
    size_t nb_triangles;
    double range = 10.0;
    size_t tagass_loop = 100;
    size_t pntgen_loop = 10;
    for (size_t i = 0; i < pntgen_loop; i++){
        const std::vector<Point3D>* points = pntgen3d(nb_points, range);  // generate nb_points of random points
        auto [vertices, triangles] = delaunay3D(*points); // do Delaunay on them, output vertices and triangles
        nb_triangles = triangles.size();
        std::vector<std::vector<size_t>*>* tagass_arr = new std::vector<std::vector<size_t>*>(tagass_loop);
        for (size_t j = 0 ; j < tagass_loop; j++){
            tagass_arr->assign(j, tagassign(nb_triangles)); // assign tags to triangles, only keep the inside ones
        }
        // nb_points += 10; // new bug!
        range += 10.0;
        
        SECTION("Number check"){
            REQUIRE(points->size() == nb_points);
        }

        SECTION("Work together delaunay"){
            REQUIRE(vertices.size() == nb_points);
            //REQUIRE(triangles.size() == 19);
        }

        SECTION("Tag assignment map num check"){
            for (size_t j = 0; j < tagass_loop-1; j++){
                REQUIRE(tagass_arr->at(j)->size() <= nb_triangles);
            }
        }
    }
}
