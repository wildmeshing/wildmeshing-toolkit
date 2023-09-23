// #include <catch2/catch.hpp>
#include <catch2/catch_test_macros.hpp>
#include <wmtk_components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk_components/delaunay/internal/delaunay_3d.hpp>
#include <wmtk_components/manifold_extraction/internal/Manifold-extraction.hpp>
#include <paraviewo/VTUWriter.hpp>
#include <igl/vertex_components.h>
#include <igl/adjacency_matrix.h>


TEST_CASE("Manifold-Extraction2D", "[components][man-ext2d]"){
    
    using namespace wmtk;
    unsigned int nb_points = 20; // 20
    unsigned int nb_triangles;
    unsigned int nb_vertices;
    double range = 10.0;
    size_t tagass_loop = 1; // 100
    size_t pntgen_loop = 1; // 10
    std::vector<std::vector<size_t>> tag(tagass_loop);
    for (size_t i = 0; i < pntgen_loop; i++){ // test for 10 iterations, each with 10 more vertices, so 20~110
        std::vector<Eigen::Vector2d> points;
        points.reserve(nb_points);
        pntgen2d(nb_points,  points, range);

        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        wmtk::components::internal::delaunay_2d(points, vertices, faces);
        nb_vertices = vertices.rows();
        nb_triangles = faces.rows();
        // std::cout << "Man-ext 2D whole triangles: " << std::endl << faces << std::endl;
        for (size_t j = 0 ; j < tagass_loop; j++){ // assign 100 sets of different tags for all triangles
            tag[j] = tagassign(nb_triangles); // assign tags to triangles, only keep the inside ones
            // std::vector<std::vector<Triangle>> components = findConnectedComponents(triangles, tagass_arr[j]);
            size_t nb_in = tag[j].size();
            int max_vertex = -1;
            Eigen::MatrixXi faces_in;
            faces_in.resize(nb_in, 3);
            for (size_t k = 0; k < nb_in; k++){
                for (size_t k2 = 0; k2 < 3; k2++) {
                    int curr = faces(tag[j][k], k2);
                    if (curr > max_vertex) {max_vertex = curr;}
                    faces_in(k, k2) = curr;
                }
            }
            Eigen::MatrixXi C;
            C.resize(max_vertex + 1, 1); // since the vertices are indexed from 0
            std::cout << "Man-ext 2D test case, j = " << j << std::endl;
            std::cout << "num of tagged tri: " << nb_in << ", tags: ";
            for (size_t k = 0; k < nb_in; k++){ std::cout << tag[j][k] << " ";}
            std::cout << std::endl;
            std::cout << faces_in << std::endl;
            igl::vertex_components(faces_in, C);
            std::cout << "Shape of C: row = " << C.rows() << ", col = " << C.cols() << std::endl << "Components:  ";
            for (int k = 0; k < max_vertex + 1; k++){std::cout << C(k, 0) << " ";}
            std::cout << std::endl;
        }
            
        if (false) {
            paraviewo::VTUWriter writer;
            writer.write_mesh("manifold_extraction_2d_random_test.vtu", vertices, faces);
        }

        // SECTION("Points Number && Delaunay check"){
        REQUIRE(points.size() == nb_points);
        REQUIRE(vertices.rows() == nb_vertices);
        // }

        // SECTION("Tag assignment map num check"){
        for (size_t j = 0; j < tagass_loop; j++){
            REQUIRE(tag[j].size() <= nb_triangles); // NOTE: <= here since we only keep those tagged "inside"
        }
        // }
        nb_points += 10;
        range += 10.0;
    }
}

TEST_CASE("Manifold-Extraction3D", "[components][man-ext3d]"){
    using namespace wmtk;
    unsigned int nb_points = 10;
    unsigned int nb_triangles;
    unsigned int nb_vertices;
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
        nb_vertices = vertices.rows(); 
        for (size_t j = 0 ; j < tagass_loop; j++){
            tag[j] = tagassign(nb_triangles); // assign tags to triangles, only keep the inside ones
            // std::vector<std::vector<Triangle>> components = findConnectedComponents(triangles, tagass_arr[j]);
        }

        if (false) {
            paraviewo::VTUWriter writer;
            writer.write_mesh("manifold_extraction_3d_random_test.vtu", vertices, faces);
        }

        // SECTION("Points Number && Delaunay check"){
        REQUIRE(points.size() == nb_points);
        REQUIRE(vertices.rows() == nb_vertices);
        // }

        // SECTION("Tag assignment map num check"){
        for (size_t j = 0; j < tagass_loop; j++){
            REQUIRE(tag[j].size() <= nb_triangles); // NOTE: <= here since we only keep those tagged "inside"
        }
        // }
        nb_points += 10;
        range += 10.0;
    }
}
