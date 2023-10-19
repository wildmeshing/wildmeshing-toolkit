#include "extract_subset_2d.hpp"

wmtk::TriMesh extract_subset_2d(
    const std::vector<Eigen::Vector2d>& points,
    Eigen::MatrixXi& triangles, std::vector<size_t> tag){

    int nb_vertex = points.size();
    int nb_vertex_in = 0;
    int nb_tri_in = tag.size();
    // maintain a vector of bool, true if an old vertex is preserved after extraction
    std::vector<bool> vertices_in_bool(nb_vertex);
    for (int k = 0; k < nb_vertex; ++k) {vertices_in_bool[k] = false;}

    Eigen::MatrixXi faces_in;
    faces_in.resize(nb_tri_in, 3);

    //tag the preserved ones and count number of vertex in new extraction
    for (size_t k = 0; k < nb_tri_in; ++k){
        for (size_t k2 = 0; k2 < 3; ++k2) {
            // faces_in(k, k2) = triangles(tag[k], k2);
            vertices_in_bool[triangles(tag[k], k2)] = true;
        }
    }
    for (bool b: vertices_in_bool) {
        if (b) nb_vertex_in ++;
    }

    // construct a map from old vertex id to new new id
    std::map<int, int> old2new;
    int j = 0;
    for (int i = 0; i < nb_vertex; ++i){
        // ignore the not extracted vertices
        if (vertices_in_bool[i]){
            // old vertex id i map to new vertex id j, where j increases by count
            old2new.insert({i, j});
            j++;
        }
    }
    assert(j == nb_vertex_in);

    wmtk::TriMesh mesh;
    wmtk::RowVectors3l tris;
    tris.resize(nb_tri_in, 3);
    for (unsigned int i = 0; i < nb_tri_in; ++i){
        // only put in the extracted ones
        size_t tri = tag[i];
        tris.row(i) << old2new[triangles(tri, 0)], old2new[triangles(tri, 1)], old2new[triangles(tri, 2)];
    }
    mesh.initialize(tris);

    Eigen::MatrixXd points_in;
    points_in.resize(nb_vertex_in, 2);
    for (int i = 0; i < nb_vertex; ++i){
        if (vertices_in_bool[i]){
            points_in(old2new[i], 0) = points[i][0];
            points_in(old2new[i], 1) = points[i][1];
        }
    }
    wmtk::mesh_utils::set_matrix_attribute(points_in, "position", wmtk::PrimitiveType::Vertex, mesh);
    return mesh;
}


// Note: the above is a draft version of the algo, implemented in bad, drafted data structure
// The following is new code to be finished
wmtk::TriMesh extract_subset_2d(std::vector<wmtk::Tuple> vertices, std::vector<wmtk::Tuple> triangles, std::vector<size_t> tag){
    assert(tag.size() <= triangles.size());

    int nb_vertex = vertices.size();
    int nb_vertex_in = 0;
    int nb_tri_in = tag.size();
    std::vector<bool> vertices_in_bool(nb_vertex);
    for (int k = 0; k < nb_vertex; ++k) {vertices_in_bool[k] = false;}
    Eigen::MatrixXi faces_in;
    faces_in.resize(nb_tri_in, 3);

    //tag the preserved ones and count number of vertex in new extraction
    for (size_t k = 0; k < nb_tri_in; ++k){
        // wmtk::attribute::MeshAttributeHandle<long> m_vf_handle;
        // wmtk::attribute::MeshAttributeHandle<long> m_vf_handle;
        // wmtk::ConstAccessor<long> vf_accessor = wmtk::Mesh::create_const_accessor<long>(m_vf_handle);
        // auto f = vf_accessor.index_access().scalar_attribute(k);
        // wmtk::ConstAccessor<long> fv_accessor = wmtk::Mesh::create_const_accessor<long>(m_fv_handle);
        // auto fv = fv_accessor.index_access().vector_attribute(f);
        for (size_t k2 = 0; k2 < 3; ++k2) {
            // if (fv(k2) == k){
                    
            // }

            // vertices_in_bool[triangles(tag[k], k2)] = true;
        }
    }
//     for (bool b: vertices_in_bool) {
//         if (b) nb_vertex_in ++;
//     }

//     // construct a map from old vertex id to new new id
//     std::map<int, int> old2new;
//     int j = 0;
//     for (int i = 0; i < nb_vertex; ++i){
//         // ignore the not extracted vertices
//         if (vertices_in_bool[i]){
//             // old vertex id i map to new vertex id j, where j increases by count
//             old2new.insert({i, j});
//             j++;
//         }
//     }
//     assert(j == nb_vertex_in);

    wmtk::TriMesh mesh;
//     wmtk::RowVectors3l tris;
//     tris.resize(nb_tri_in, 3);
//     for (unsigned int i = 0; i < nb_tri_in; ++i){
//         // only put in the extracted ones
//         size_t tri = tag[i];
//         tris.row(i) << old2new[triangles(tri, 0)], old2new[triangles(tri, 1)], old2new[triangles(tri, 2)];
//     }
//     mesh.initialize(tris);

//     Eigen::MatrixXd points_in;
//     points_in.resize(nb_vertex_in, 2);
//     for (int i = 0; i < nb_vertex; ++i){
//         if (vertices_in_bool[i]){
//             points_in(old2new[i], 0) = points[i][0];
//             points_in(old2new[i], 1) = points[i][1];
//         }
//     }
//     wmtk::mesh_utils::set_matrix_attribute(points_in, "position", wmtk::PrimitiveType::Vertex, mesh);
    return mesh;
}