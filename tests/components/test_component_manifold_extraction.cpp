// #include <catch2/catch.hpp>
#include <catch2/catch_test_macros.hpp>
#include <wmtk_components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk_components/delaunay/internal/delaunay_3d.hpp>
#include <wmtk_components/manifold_extraction/internal/Manifold-extraction.hpp>
#include <paraviewo/VTUWriter.hpp>
#include <igl/vertex_components.h>
#include <igl/adjacency_matrix.h>


auto tagassign(size_t nb_triangles, double prob) -> std::vector<size_t>{
    std::vector<size_t> tagass;
    std::random_device rd{};
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0, 1);
    // std::srand(10);
    for (size_t i = 0 ; i < nb_triangles; i++){
        double tag = dis(gen);
        if (tag < prob) tagass.push_back(i); // for every tagged triangle, add its index to the end
    }
    return tagass;
}

TEST_CASE("Manifold-Extraction2D", "[components][man-ext2d]"){
    
    using namespace wmtk;
    unsigned int nb_points = 20; // 20
    unsigned int nb_triangles;
    unsigned int nb_vertices;
    double range = 10.0;
    size_t tagass_loop = 3; // 100
    size_t pntgen_loop = 2; // 10
    double prob = 0.2;
    std::vector<std::vector<size_t>> tag(tagass_loop);
    for (size_t i = 0; i < pntgen_loop; i++){ // test for 10 iterations, each with 10 more vertices, so 20~110
        std::vector<Eigen::Vector2d> points;
        points.reserve(nb_points);
        std::random_device rd{};
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0, range);
        for (size_t j = 0; j < nb_points; ++j) {
            // generate 2 random doubles between 0 and the given range
            points.push_back({dis(gen),dis(gen)});
        }

        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        wmtk::components::internal::delaunay_2d(points, vertices, faces);
        nb_vertices = vertices.rows();
        nb_triangles = faces.rows();
        std::cout << "\nMan-ext 2D test: total tri num=" << nb_triangles <<"\n";
        // std::cout<< faces << std::endl;

        // assign 100 sets of different tags for all triangles
        for (size_t j = 0 ; j < tagass_loop; j++){
            tag[j] = tagassign(nb_triangles, prob); // assign tags to triangles, only keep the inside ones
            size_t nb_in = tag[j].size();
            int max_vertex = -1;
            Eigen::MatrixXi faces_in;
            faces_in.resize(nb_in, 3);
            for (size_t k = 0; k < nb_in; k++){
                for (size_t k2 = 0; k2 < 3; k2++) {
                    faces_in(k, k2) = faces(tag[j][k], k2);
                }
            }
            Eigen::MatrixXi C;
            Eigen::SparseMatrix<int> adj;
            igl::adjacency_matrix(faces_in,adj); // A is the adjacency matrix 
            igl::vertex_components(adj, C); // C is the list of component ids
            std::cout << "Case " << j << ", tagged tri num: " << nb_in << ", tags: ";
            for (size_t k = 0; k < nb_in; k++){ std::cout << tag[j][k] << " ";}
            // std::cout << std::endl << faces_in;
            std::cout << "\nComponents: ";
            for (int k = 0; k < C.rows() ; k++){std::cout << C(k, 0) << " ";} std::cout << "\n";
        }
            
        if (false) {
            paraviewo::VTUWriter writer;
            writer.write_mesh("manifold_extraction_2d_random_test.vtu", vertices, faces);
        }

        REQUIRE(points.size() == nb_points);
        REQUIRE(vertices.rows() == nb_vertices);

        for (size_t j = 0; j < tagass_loop; j++){
            REQUIRE(tag[j].size() <= nb_triangles); // NOTE: <= since we only keep those tagged "inside"
        }
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
    double prob = 0.2;
    std::vector<std::vector<size_t>> tag(tagass_loop);

    for (size_t i = 0; i < pntgen_loop; i++){
        std::vector<Eigen::Vector3d> points;
        points.reserve(nb_points);
        std::random_device rd{};
        std::mt19937 gen(rd()); //std::mt19937 gen(10);
        std::uniform_real_distribution<double> dis(0, range);
        for (size_t j = 0; j < nb_points; ++j) {
            // generate 3 random doubles between 0 and the given range
            points.push_back({dis(gen), dis(gen), dis(gen)});
        }

        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        wmtk::components::internal::delaunay_3d(points, vertices, faces);
        
        nb_triangles = faces.rows();
        nb_vertices = vertices.rows(); 
        for (size_t j = 0 ; j < tagass_loop; j++){
            tag[j] = tagassign(nb_triangles, prob); // assign tags to triangles, only keep the inside ones
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
            REQUIRE(tag[j].size() <= nb_triangles); // NOTE: <= since we only keep those tagged "inside"
        }
        // }
        nb_points += 10;
        range += 10.0;
    }
}
