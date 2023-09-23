// #include <catch2/catch.hpp>
#include <catch2/catch_test_macros.hpp>
// #include <wmtk/utils/Manifold-extraction.hpp>
// #include <wmtk/utils/Delaunay.hpp>
#include <wmtk_components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk_components/delaunay/internal/delaunay_3d.hpp>
#include <wmtk_components/manifold_extraction/internal/Manifold-extraction.hpp>


TEST_CASE("Manifold-Extraction2D", "[man-ext2d]"){
    
    using namespace wmtk;
    unsigned int nb_points = 20;
    size_t nb_triangles;
    double range = 10.0;
    size_t tagass_loop = 100;
    size_t pntgen_loop = 10;
    std::vector<std::vector<size_t>> tag(tagass_loop);

    for (size_t i = 0; i < pntgen_loop; i++){
        std::vector<Eigen::Vector2d> points;
        points.reserve(nb_points);
        pntgen2d(nb_points,  points, range);

        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        wmtk::components::internal::delaunay_2d(points, vertices, faces);

        // auto [vertices, triangles] = delaunay2D(pntgen_arr[i]); // do Delaunay on them, output vertices and triangles
        nb_triangles = faces.rows();
        for (size_t j = 0 ; j < tagass_loop; j++){
            tag[j] = tagassign(nb_triangles); // assign tags to triangles, only keep the inside ones
            // std::vector<std::vector<Triangle>> components = findConnectedComponents(triangles, tagass_arr[j]);
        }

        SECTION("Number check"){
            REQUIRE(points.size() == nb_points);
        }

        SECTION("Work together delaunay"){
            REQUIRE(vertices.size() == nb_points);
        }

        SECTION("Tag assignment map num check"){
            for (size_t j = 0; j < tagass_loop; j++){
                REQUIRE(tag[j].size() <= nb_triangles);
            }
        }
        nb_points += 10;
        range += 10.0;
    }
}

TEST_CASE("Manifold-Extraction3D", "[man-ext3d]"){
    using namespace wmtk;
    unsigned int nb_points = 10;
    size_t nb_triangles;
    double range = 10.0;
    size_t tagass_loop = 100;
    size_t pntgen_loop = 10;
//     std::vector<std::vector<Point3D>> pntgen_arr(pntgen_loop);
//     std::vector<std::vector<size_t>> tagass_arr(tagass_loop);
    std::vector<std::vector<size_t>> tag(tagass_loop);

    for (size_t i = 0; i < pntgen_loop; i++){
        // pntgen_arr[i] = pntgen3d(nb_points, range);  // generate nb_points of random points
        // auto [vertices, triangles] = delaunay3D(pntgen_arr[i]); // do Delaunay on them, output vertices and triangles
        // nb_triangles = triangles.size();
        // for (size_t j = 0 ; j < tagass_loop; j++){
        //     tagass_arr[j] = tagassign(nb_triangles); // assign tags to triangles, only keep the inside ones 
        // }
        std::vector<Eigen::Vector3d> points;
        points.reserve(nb_points);
        pntgen3d(nb_points,  points, range);

        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        wmtk::components::internal::delaunay_3d(points, vertices, faces);
        
        nb_triangles = faces.rows();
        for (size_t j = 0 ; j < tagass_loop; j++){
            tag[j] = tagassign(nb_triangles); // assign tags to triangles, only keep the inside ones
            // std::vector<std::vector<Triangle>> components = findConnectedComponents(triangles, tagass_arr[j]);
        }

        SECTION("Number check"){
            REQUIRE(points.size() == nb_points);
        }

        SECTION("Work together delaunay"){
            REQUIRE(vertices.size() == nb_points);
        }

        SECTION("Tag assignment map num check"){
            for (size_t j = 0; j < tagass_loop; j++){
                REQUIRE(tag[j].size() <= nb_triangles);
            }
        }
        nb_points += 10;
        range += 10.0;
    }
}
