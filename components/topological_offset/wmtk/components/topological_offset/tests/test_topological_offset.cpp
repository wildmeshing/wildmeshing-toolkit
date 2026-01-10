#include <catch2/catch_test_macros.hpp>
#include <wmtk/components/topological_offset/TopoOffsetMesh.h>
#include <wmtk/TetMesh.h>

using namespace wmtk;
using namespace components::topological_offset;


// // TESTING
// void topological_offset(nlohmann::json json_params) {
//     Eigen::MatrixXd V(4, 3);
//     V << 0, 0, 0,
//          1, 0, 0,
//          0, 1, 0,
//          0, 0, 1;
//     Eigen::MatrixXi T(1, 4);
//     T << 0, 1, 2, 3;
//     Eigen::MatrixXi Tags(1, 1);
//     Tags << 0;

//     Parameters param;
//     param.tag = 0;
//     param.sep_tags.push_back(1); param.sep_tags.push_back(4);
//     topological_offset::TopoOffsetMesh mesh(param, 0);
//     std::string outf = "/Users/seb9449/Desktop/wildmeshing/test/out";

//     mesh.init_from_image(V, T, Tags);
//     logger().info("{} {} {} {}", mesh.tet_size(), mesh.get_faces().size(), mesh.get_edges().size(), mesh.vertex_size());
//     mesh.write_vtu(outf + fmt::format("_{}", mesh.m_vtu_counter++));

//     std::vector<wmtk::TetMesh::Tuple> new_edges;
//     wmtk::TetMesh::Tuple e = mesh.tuple_from_edge({0, 1});
//     mesh.split_edge(e, new_edges);
//     logger().info("{} {} {} {}", mesh.tet_size(), mesh.get_faces().size(), mesh.get_edges().size(), mesh.vertex_size());
//     mesh.write_vtu(outf + fmt::format("_{}", mesh.m_vtu_counter++));

//     mesh.consolidate_mesh();
//     logger().info("{} {} {} {}", mesh.tet_size(), mesh.get_faces().size(), mesh.get_edges().size(), mesh.vertex_size());
// }


// EDGE SPLIT TESTING
// this should go somewhere else, just testing for now
void unit_tests1() {
    Eigen::MatrixXd V(4, 3);
    V << 0, 0, 0,
         1, 0, 0,
         0, 0, 1,
         0, 1, 0;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXi Tags(1, 1);
    Tags << 0;

    Parameters param;
    param.tag = 0;
    param.sep_tags.push_back(1); param.sep_tags.push_back(4);
    topological_offset::TopoOffsetMesh mesh(param, 0);
    // std::string outf = "/Users/seb9449/Desktop/wildmeshing/test/edgesplittest_out";
    
    mesh.init_from_image(V, T, Tags);
    // mesh.write_vtu(outf + "_0");
    // auto res = igl::predicates::orient3d(mesh.m_vertex_attribute[0].m_posf,
    //                                      mesh.m_vertex_attribute[1].m_posf,
    //                                      mesh.m_vertex_attribute[2].m_posf,
    //                                      mesh.m_vertex_attribute[3].m_posf);
    // for (int i = 0; i < 4; i++) {
    //     std::cout << mesh.m_vertex_attribute[i].m_posf << std::endl;
    //     std::cout << "---" << std::endl;
    // }
    // std::cout << (res == igl::predicates::Orientation::POSITIVE) << std::endl;

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = 1;
    mesh.m_vertex_attribute[1].label = 2;
    mesh.m_vertex_attribute[2].label = 3;
    mesh.m_vertex_attribute[3].label = 4;
    mesh.m_edge_attribute[0].label = 5;
    mesh.m_edge_attribute[1].label = 6;
    mesh.m_edge_attribute[2].label = 7;
    mesh.m_edge_attribute[3].label = 8;
    mesh.m_edge_attribute[4].label = 9;
    mesh.m_edge_attribute[5].label = 10;
    mesh.m_face_attribute[0].label = 11;
    mesh.m_face_attribute[1].label = 12;
    mesh.m_face_attribute[2].label = 13;
    mesh.m_face_attribute[3].label = 14;
    mesh.m_tet_attribute[0].label = 15;

    TetMesh::Tuple e = mesh.tuple_from_edge({1, 3});
    std::vector<TetMesh::Tuple> garbage;
    // std::cout << fmt::format("{} {} {} {}", e.tid(mesh), e.fid(mesh), e.eid(mesh), e.vid(mesh));
    mesh.split_edge(e, garbage);
    // std::cout << fmt::format("{} {} {} {}", e.tid(mesh), e.fid(mesh), e.eid(mesh), e.vid(mesh));
    // mesh.write_vtu(outf + "_1");

    // auto edges = mesh.get_edges();
    // for (const TetMesh::Tuple& edge : edges) {
    //     std::cout << fmt::format("{} {} {} {}\n", edge.tid(mesh), edge.fid(mesh), edge.eid(mesh), edge.vid(mesh));
    // }
    auto [_, global_fid] = mesh.tuple_from_face({4, 0, 3});
    // std::cout << global_fid << std::endl;

    std::cout << (mesh.m_vertex_attribute[0].label == 1) << std::endl;
    std::cout << (mesh.m_vertex_attribute[1].label == 2) << std::endl;
    std::cout << (mesh.m_vertex_attribute[2].label == 3) << std::endl;
    std::cout << (mesh.m_vertex_attribute[3].label == 4) << std::endl;
    std::cout << (mesh.m_vertex_attribute[4].label == 9) << std::endl;
    std::cout << (mesh.m_edge_attribute[0].label == 5) << std::endl;
    std::cout << (mesh.m_edge_attribute[1].label == 6) << std::endl;
    std::cout << (mesh.m_edge_attribute[2].label == 7) << std::endl;
    std::cout << (mesh.m_edge_attribute[3].label == 13) << std::endl;
    std::cout << (mesh.m_edge_attribute[4].label == 9) << std::endl;
    std::cout << (mesh.m_edge_attribute[5].label == 14) << std::endl;
    std::cout << (mesh.m_edge_attribute[9].label == 8) << std::endl;
    std::cout << (mesh.m_edge_attribute[10].label == 9) << std::endl;
    std::cout << (mesh.m_edge_attribute[11].label == 10) << std::endl;
    std::cout << (mesh.m_face_attribute[0].label == 11) << std::endl;
    std::cout << (mesh.m_face_attribute[1].label == 15) << std::endl;
    std::cout << (mesh.m_face_attribute[2].label == 13) << std::endl;
    std::cout << (mesh.m_face_attribute[3].label == 14) << std::endl;
    std::cout << (mesh.m_face_attribute[5].label == 12) << std::endl;
    std::cout << (mesh.m_face_attribute[6].label == 13) << std::endl;
    std::cout << (mesh.m_face_attribute[7].label == 14) << std::endl;
    std::cout << (mesh.m_tet_attribute[0].label == 15) << std::endl;
    std::cout << (mesh.m_tet_attribute[1].label == 15) << std::endl;
}
// void topological_offset(nlohmann::json json_params) {unit_tests1();}


// FACE SPLIT TESTING
// this should go somewhere else, just testing for now
void unit_tests2() {
    Eigen::MatrixXd V(4, 3);
    V << 0, 0, 0,
         1, 0, 0,
         0, 0, 1,
         0, 1, 0;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXi Tags(1, 1);
    Tags << 0;

    Parameters param;
    param.tag = 0;
    param.sep_tags.push_back(1); param.sep_tags.push_back(4);
    topological_offset::TopoOffsetMesh mesh(param, 0);
    std::string outf = "/Users/seb9449/Desktop/wildmeshing/test/edgesplittest_out";
    
    mesh.init_from_image(V, T, Tags);
    mesh.write_vtu(outf + "_0");

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = 1;
    mesh.m_vertex_attribute[1].label = 2;
    mesh.m_vertex_attribute[2].label = 3;
    mesh.m_vertex_attribute[3].label = 4;
    mesh.m_edge_attribute[0].label = 5;
    mesh.m_edge_attribute[1].label = 6;
    mesh.m_edge_attribute[2].label = 7;
    mesh.m_edge_attribute[3].label = 8;
    mesh.m_edge_attribute[4].label = 9;
    mesh.m_edge_attribute[5].label = 10;
    mesh.m_face_attribute[0].label = 11;
    mesh.m_face_attribute[1].label = 12;
    mesh.m_face_attribute[2].label = 13;
    mesh.m_face_attribute[3].label = 14;
    mesh.m_tet_attribute[0].label = 15;

    auto [f, _] = mesh.tuple_from_face({1, 2, 3});
    std::vector<TetMesh::Tuple> garbage;
    // logger().info("t{} f{} e{} v{}", f.tid(mesh), f.fid(mesh), f.eid(mesh), f.vid(mesh));
    mesh.split_face(f, garbage);
    // logger().info("{} {} {} {}", f.tid(mesh), f.fid(mesh), f.eid(mesh), f.vid(mesh));
    mesh.write_vtu(outf + "_1");

    std::cout << (mesh.m_vertex_attribute[0].label == 1) << std::endl;
    std::cout << (mesh.m_vertex_attribute[1].label == 2) << std::endl;
    std::cout << (mesh.m_vertex_attribute[2].label == 3) << std::endl;
    std::cout << (mesh.m_vertex_attribute[3].label == 4) << std::endl;
    std::cout << (mesh.m_vertex_attribute[4].label == 14) << std::endl;
    std::cout << (mesh.m_edge_attribute[0].label == 5) << std::endl;
    std::cout << (mesh.m_edge_attribute[1].label == 14) << std::endl;
    std::cout << (mesh.m_edge_attribute[2].label == 15) << std::endl;
    std::cout << (mesh.m_edge_attribute[3].label == 8) << std::endl;
    std::cout << (mesh.m_edge_attribute[4].label == 9) << std::endl;
    std::cout << (mesh.m_edge_attribute[5].label == 14) << std::endl;
    std::cout << (mesh.m_edge_attribute[7].label == 6) << std::endl;
    std::cout << (mesh.m_edge_attribute[8].label == 7) << std::endl;
    std::cout << (mesh.m_edge_attribute[11].label == 14) << std::endl;
    std::cout << (mesh.m_edge_attribute[17].label == 10) << std::endl;
    std::cout << (mesh.m_face_attribute[0].label == 15) << std::endl;
    std::cout << (mesh.m_face_attribute[1].label == 15) << std::endl;
    std::cout << (mesh.m_face_attribute[2].label == 13) << std::endl;
    std::cout << (mesh.m_face_attribute[3].label == 14) << std::endl;
    std::cout << (mesh.m_face_attribute[4].label == 11) << std::endl;
    std::cout << (mesh.m_face_attribute[5].label == 15) << std::endl;
    std::cout << (mesh.m_face_attribute[7].label == 14) << std::endl;
    std::cout << (mesh.m_face_attribute[9].label == 12) << std::endl;
    std::cout << (mesh.m_face_attribute[11].label == 14) << std::endl;
    std::cout << (mesh.m_tet_attribute[0].label == 15) << std::endl;
    std::cout << (mesh.m_tet_attribute[1].label == 15) << std::endl;
    std::cout << (mesh.m_tet_attribute[2].label == 15) << std::endl;
    std::cout << std::endl;
    std::cout << mesh.m_edge_attribute[4].label << std::endl;
    std::cout << mesh.m_edge_attribute[7].label << std::endl;
    std::cout << mesh.m_edge_attribute[17].label << std::endl;
}
// void topological_offset(nlohmann::json json_params) {unit_tests2();}


// TET SPLIT TESTING
// this should go somewhere else, just testing for now
void unit_tests3() {
    Eigen::MatrixXd V(4, 3);
    V << 0, 0, 0,
         1, 0, 0,
         0, 0, 1,
         0, 1, 0;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXi Tags(1, 1);
    Tags << 0;

    Parameters param;
    param.tag = 0;
    param.sep_tags.push_back(1); param.sep_tags.push_back(4);
    topological_offset::TopoOffsetMesh mesh(param, 0);
    std::string outf = "/Users/seb9449/Desktop/wildmeshing/test/edgesplittest_out";
    
    mesh.init_from_image(V, T, Tags);
    mesh.write_vtu(outf + "_0");

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = 1;
    mesh.m_vertex_attribute[1].label = 2;
    mesh.m_vertex_attribute[2].label = 3;
    mesh.m_vertex_attribute[3].label = 4;
    mesh.m_edge_attribute[0].label = 5;
    mesh.m_edge_attribute[1].label = 6;
    mesh.m_edge_attribute[2].label = 7;
    mesh.m_edge_attribute[3].label = 8;
    mesh.m_edge_attribute[4].label = 9;
    mesh.m_edge_attribute[5].label = 10;
    mesh.m_face_attribute[0].label = 11;
    mesh.m_face_attribute[1].label = 12;
    mesh.m_face_attribute[2].label = 13;
    mesh.m_face_attribute[3].label = 14;
    mesh.m_tet_attribute[0].label = 15;

    TetMesh::Tuple t = mesh.tuple_from_tet(0);
    std::vector<TetMesh::Tuple> garbage;
    // logger().info("t{} f{} e{} v{}", f.tid(mesh), f.fid(mesh), f.eid(mesh), f.vid(mesh));
    mesh.split_tet(t, garbage);
    // logger().info("{} {} {} {}", f.tid(mesh), f.fid(mesh), f.eid(mesh), f.vid(mesh));
    mesh.write_vtu(outf + "_1");

    // //  TESTING
    // logger().info("--------");
    // for (int i = 0; i < mesh.vertex_size(); i++) {
    //     Eigen::Vector3d p = mesh.m_vertex_attribute[i].m_posf;
    //     logger().info("v{}: ({}, {}, {})", i, p(0), p(1), p(2));
    // }
    // logger().info("--------");

    // auto edges1 = mesh.get_edges();
    // logger().info("--------");
    // for (const TetMesh::Tuple e : edges1) {
    //     std::array<size_t, 2> vs = {e.vid(mesh), mesh.switch_vertex(e).vid(mesh)};
    //     logger().info("e{}: {} {}", e.eid(mesh), vs[0], vs[1]);
    // }
    // logger().info("--------");

    // auto faces1 = mesh.get_faces();
    // logger().info("--------");
    // for (const TetMesh::Tuple f : faces1) {
    //     auto vs = mesh.get_face_vids(f);
    //     logger().info("f{}: {} {} {}", f.fid(mesh), vs[0], vs[1], vs[2]);
    // }
    // logger().info("--------");

    // auto tets1 = mesh.get_tets();
    // logger().info("--------");
    // for (const TetMesh::Tuple t : tets1) {
    //     auto vs = mesh.oriented_tet_vids(t);
    //     logger().info("t{}: {} {} {} {}", t.tid(mesh), vs[0], vs[1], vs[2], vs[3]);
    // }
    // logger().info("--------");
    // // TESTING

    logger().info("{}", mesh.m_vertex_attribute[0].label == 1);
    logger().info("{}", mesh.m_vertex_attribute[1].label == 2);
    logger().info("{}", mesh.m_vertex_attribute[2].label == 3);
    logger().info("{}", mesh.m_vertex_attribute[3].label == 4);
    logger().info("{}", mesh.m_vertex_attribute[4].label == 15);

    logger().info("{}", mesh.m_edge_attribute[0].label == 15);
    logger().info("{}", mesh.m_edge_attribute[1].label == 6);
    logger().info("{}", mesh.m_edge_attribute[2].label == 15);
    logger().info("{}", mesh.m_edge_attribute[3].label == 15);
    logger().info("{}", mesh.m_edge_attribute[4].label == 9);
    logger().info("{}", mesh.m_edge_attribute[5].label == 10);
    logger().info("{}", mesh.m_edge_attribute[6].label == 15);
    logger().info("{}", mesh.m_edge_attribute[8].label == 7);
    logger().info("{}", mesh.m_edge_attribute[9].label == 8);
    logger().info("{}", mesh.m_edge_attribute[12].label == 5);

    logger().info("{}", mesh.m_face_attribute[0].label == 15);
    logger().info("{}", mesh.m_face_attribute[1].label == 15);
    logger().info("{}", mesh.m_face_attribute[2].label == 15);
    logger().info("{}", mesh.m_face_attribute[3].label == 14);
    logger().info("{}", mesh.m_face_attribute[4].label == 15);
    logger().info("{}", mesh.m_face_attribute[5].label == 12);
    logger().info("{}", mesh.m_face_attribute[6].label == 15);
    logger().info("{}", mesh.m_face_attribute[8].label == 15);
    logger().info("{}", mesh.m_face_attribute[10].label == 13);
    logger().info("{}", mesh.m_face_attribute[12].label == 11);

    logger().info("{}", mesh.m_tet_attribute[0].label == 15);
    logger().info("{}", mesh.m_tet_attribute[1].label == 15);
    logger().info("{}", mesh.m_tet_attribute[2].label == 15);
    logger().info("{}", mesh.m_tet_attribute[3].label == 15);
}
// void topological_offset(nlohmann::json json_params) {unit_tests3();}
