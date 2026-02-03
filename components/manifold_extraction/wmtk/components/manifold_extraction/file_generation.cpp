// #include <igl/write_triangle_mesh.h>
// #include <wmtk/TetMesh.h>
// #include <wmtk/components/topological_offset/ManExtractMesh.h>
// #include <Eigen/Core>
// #include <Eigen/Dense>
// #include <cstdlib>
// #include <random>
// #include <wmtk/utils/Delaunay.hpp>
// #include <wmtk/utils/io.hpp>
// #include "read_image_msh.hpp"
// #include "write_image_msh.hpp"


// using namespace wmtk::delaunay;
// using namespace wmtk::components::topological_offset;


// void write_manual_msh(
//     std::string path,
//     std::vector<Point3D> verts,
//     std::vector<Tetrahedron> tets,
//     std::vector<double> wns)
// {
//     // logger().info("Write {}", file);

//     wmtk::MshData msh;

//     msh.add_tet_vertices(verts.size(), [&](size_t i) { return verts[i]; });

//     msh.add_tets(tets.size(), [&](size_t i) {
//         std::array<size_t, 4> data;
//         for (int j = 0; j < 4; j++) {
//             data[j] = tets[i][j];
//             assert(data[j] < vtx.size());
//         }
//         return data;
//     });

//     msh.add_tet_attribute<1>("winding_number_tracked", [&](size_t i) { return wns[i]; });

//     msh.save(path, true);
// }


// void vertNonManifoldOBJ()
// {
//     std::string path =
//         "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/nonman_vert.obj";

//     Eigen::MatrixXd V(7, 3);
//     V << 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 1, 0, 1, 0;

//     Eigen::MatrixXi F(8, 3);
//     F << 0, 2, 1, 0, 1, 3, 2, 0, 3, 1, 2, 3, 4, 5, 0, 0, 5, 6, 4, 0, 6, 6, 5, 4;

//     igl::write_triangle_mesh(path, V, F);
// }


// void edgeNonManifoldMesh()
// {
//     std::string path =
//         "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/nonman_edge.msh";

//     std::vector<Point3D> verts;
//     verts.push_back({-1, 0, 0});
//     verts.push_back({0, -1, 0});
//     verts.push_back({0, 0, 0});
//     verts.push_back({0, 0, 1});
//     verts.push_back({1, 0, 0});
//     verts.push_back({0, 1, 0});

//     std::vector<Tetrahedron> tets;
//     tets.push_back({0, 1, 2, 3});
//     tets.push_back({2, 3, 4, 5});

//     std::vector<double> wns;
//     wns.push_back(0.8);
//     wns.push_back(0.9);

//     write_manual_msh(path, verts, tets, wns);
// }


// void edgeNonManifoldOBJ()
// {
//     std::string path =
//         "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/nonman_edge.obj";

//     Eigen::MatrixXd V(6, 3);
//     V << -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0;

//     Eigen::MatrixXi F(8, 3);
//     F << 0, 1, 3, 1, 2, 3, 0, 3, 2, 2, 1, 0, 5, 3, 4, 2, 3, 5, 4, 3, 2, 4, 2, 5;

//     igl::write_triangle_mesh(path, V, F);
// }


// void randMesh()
// {
//     const int N_POINTS = 20;
//     unsigned seed = 0;
//     std::mt19937 engine(seed);
//     std::uniform_real_distribution<double> dist(-10, 10);
//     // std::string opath = "/Users/seb9449/Desktop/wildmeshing/point_cloud.obj";
//     std::string opath = "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/"
//                         "rand_mesh_20p_robust.msh";

//     std::vector<Point3D> points;
//     for (int i = 0; i < N_POINTS; i++) {
//         double x = dist(engine);
//         double y = dist(engine);
//         double z = dist(engine);
//         Point3D p = {x, y, z};
//         points.push_back(p);
//     }

//     Eigen::MatrixXd V(N_POINTS, 3);
//     for (int i = 0; i < N_POINTS; i++) {
//         V(i, 0) = points[i][0];
//         V(i, 1) = points[i][1];
//         V(i, 2) = points[i][2];
//     }
//     // Eigen::MatrixXi F(0, 3);  // dont need
//     // igl::write_triangle_mesh(opath, V, F);

//     auto [verts, tets] = delaunay3D(points);

//     // random tagging
//     std::vector<double> wns;
//     std::uniform_real_distribution<double> wn_dist(0, 1);
//     for (int i = 0; i < tets.size(); i++) {
//         double val = wn_dist(engine);
//         wns.push_back(val);
//     }

//     // save
//     write_manual_msh(opath, verts, tets, wns);
// }


// void randMeshSphere()
// {
//     // load mesh
//     std::string ipath = "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/"
//                         "sphere_output_final.msh";
//     MatrixXd V;
//     MatrixXi T;
//     MatrixXd Tags;
//     std::map<std::string, int> label_map;
//     read_image_msh(ipath, V, T, Tags, label_map);

//     // wmtk::TetMesh mesh;
//     // mesh.init(V.rows(), T);

//     // random winding numbers
//     unsigned seed = 0;
//     std::mt19937 engine(seed);
//     std::uniform_real_distribution<double> dist(0, 1);
//     for (int i = 0; i < T.rows(); i++) {
//         Vector3d v1 = V.row(T(i, 0));
//         Vector3d v2 = V.row(T(i, 1));
//         Vector3d v3 = V.row(T(i, 2));
//         Vector3d v4 = V.row(T(i, 3));
//         Vector3d m = (v1 + v2 + v3 + v4) / 4;
//         if (m.norm() > 0.5) {
//             Tags(i, label_map["winding_number_tracked"]) = 1.0;
//         } else {
//             Tags(i, label_map["winding_number_tracked"]) = dist(engine);
//         }
//     }

//     // save mesh
//     std::string opath = "/Users/seb9449/Desktop/wildmeshing/manifold_extraction/topological_offset/"
//                         "rand_sphere_2.msh";
//     write_image_msh(opath, V, T, Tags, label_map);
// }
