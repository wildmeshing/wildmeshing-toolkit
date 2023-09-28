// #include <catch2/catch.hpp>
#include <catch2/catch_test_macros.hpp>
#include <wmtk_components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk_components/delaunay/internal/delaunay_3d.hpp>
#include <wmtk_components/manifold_extraction/internal/Manifold-extraction.hpp>
#include <paraviewo/VTUWriter.hpp>
#include <igl/adjacency_matrix.h>
#include <igl/connected_components.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>


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
    const size_t tagass_loop = 3; // 100
    const size_t pntgen_loop = 1; // 10
    const double prob = 0.2;
    paraviewo::VTUWriter writer;
    std::vector<std::vector<size_t>> tag(tagass_loop);

    for (size_t i = 0; i < pntgen_loop; ++i){ // test for 10 iterations, each with 10 more vertices, so 20~110
        wmtk::TriMesh m;
        wmtk::RowVectors3l tris;

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

        // start using Trimesh data structure
        tris.resize(nb_triangles, 3);
        for (unsigned int j = 0; j < nb_triangles; ++j){
            tris.row(j) << faces(j, 0), faces(j, 1), faces(j, 2);
        }
        m.initialize(tris);
        wmtk::mesh_utils::set_matrix_attribute(vertices, "position", PrimitiveType::Vertex, m);
        // Question: should we save each component in each tag assignment as a separate tri mesh(or submesh of the original mesh)?

        // assign 100 sets of different tags for all triangles
        for (size_t j = 0 ; j < tagass_loop; ++j){
            tag[j] = tagassign(nb_triangles, prob); // assign tags to triangles, only keep the inside ones
            size_t nb_in = tag[j].size();
            Eigen::MatrixXi faces_in;
            bool vertices_in_bool[nb_vertices]; for (int k = 0; k < (int)nb_vertices; k++) {vertices_in_bool[k] = false;}
            int nb_vertices_in = 0;
            faces_in.resize(nb_in, 3);
            for (size_t k = 0; k < nb_in; ++k){
                for (size_t k2 = 0; k2 < 3; ++k2) {
                    // tag[j][k] is triangle index
                    faces_in(k, k2) = faces(tag[j][k], k2);
                    vertices_in_bool[faces(tag[j][k], k2)] = true;
                }
            }

            Eigen::MatrixXi C;
            Eigen::MatrixXi arg2;
            Eigen::SparseMatrix<int> adj;
            igl::adjacency_matrix(faces_in,adj); // A is the adjacency matrix 
            igl::connected_components(adj, C, arg2);

            std::cout << "Case " << j << ", tagged tri num: " << nb_in << "/" << nb_triangles;
            // std::cout << "\ntags: ";
            // for (size_t k = 0; k < nb_in; k++){ std::cout << tag[j][k] << " ";}
            // std::cout << std::endl << faces_in;

            // std::cout << "\nComponents: ";
            // for (int k = 0; k < C.rows() ; k++){std::cout << C(k, 0) << " ";}

            // std::cout << "\nsize of each cc: ";
            // for (int k = 0; k < arg2.rows() ; k++){std::cout << arg2(k, 0) << " ";}
            
            // store as a map each component and a vector of its vertices
            std::map<int, std::vector<int>> cc_in;
            for (int k = 0; k < (int)nb_vertices; ++k){
                if (vertices_in_bool[k]){ // vertex k must be in inside
                    nb_vertices_in ++;
                    // if the component that current vertex is in isn't stored in the map
                    if (cc_in.count(C(k, 0)) == 0){
                        // create a new pair
                        cc_in.insert({C(k, 0), {k}});
                    }
                    else {
                        //otherwise, append current vertex to the end of this component
                        cc_in[C(k, 0)].push_back(k);
                    }
                }
            }
            // Eigen::MatrixXd vertices_in;
            // vertices_in.resize(nb_vertices_in, 3);
            // int temp = 0;
            // for (int k = 0; k < (int)nb_vertices; k++){
            //     if (vertices_in_bool[k]){ // vertex k must be in inside
            //     for (int temp2 = 0; temp2 < 3; temp2 ++)
            //         vertices_in(temp, temp2) = vertices(k, temp2);
            //     }
            //     temp++;
            // }
            
            std::cout << "\nconnected components inside num: " << cc_in.size();
            std::cout << "\nvertices inside num: " << nb_vertices_in << "\n";
            // std::cout << "Connect components inside: \n";
            // for (int k = 0; k < (int)nb_vertices; k++){
            //     if (vertices_in[k]){
            //         std::cout << C(k, 0) << " component has " << arg2(C(k, 0)) << " vertices: ";
            //         for (int l: cc_in[C(k, 0)]) std::cout << l << " ";
            //         std::cout << std::endl;
            //     }
            // }

            int sum = 0;
            for (int k = 0; k < arg2.rows() ; ++k){
                if (arg2(k, 0) >= 3){
                    sum += arg2(k, 0);
                }
            }
            REQUIRE(sum == nb_vertices_in);
            // if (false){
            //     writer.write_mesh("manifold_extraction_2d_random_test.vtu", vertices_in, faces_in);
            // }
        }
            
        if (false) {
            writer.write_mesh("manifold_extraction_2d.vtu", vertices, faces);
        }

        REQUIRE(points.size() == nb_points);
        REQUIRE(vertices.rows() == nb_vertices);

        for (size_t j = 0; j < tagass_loop; ++j){
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
    const size_t tagass_loop = 100;
    const size_t pntgen_loop = 10;
    const double prob = 0.2;
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
