#include "extract_subset_2d.hpp"


wmtk::TriMesh extract_subset_2d(
    const std::vector<Eigen::Vector2d>& points,
    Eigen::MatrixXi& triangles, std::vector<size_t> tag){

    int nb_vertex = points.size();
    int nb_vertex_in = 0;
    int nb_tri_in = tag.size();
    std::vector<bool> vertices_in_bool(nb_vertex);
    for (int k = 0; k < nb_vertex; ++k) {vertices_in_bool[k] = false;}
    Eigen::MatrixXi faces_in;
    faces_in.resize(nb_tri_in, 3);
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
        if (!vertices_in_bool[i])
                    old2new.insert({i, -1});
        else {
            old2new.insert({i, j});
            j++;
        }
    }

    wmtk::TriMesh mesh;
    wmtk::RowVectors3l tris;
    tris.resize(nb_tri_in, 3);
    for (unsigned int i = 0; i < nb_tri_in; ++i){
        tris.row(i) << old2new[triangles(tag[i], 0)], old2new[triangles(tag[i], 1)], old2new[triangles(tag[i], 2)];
    }
    mesh.initialize(tris);
    Eigen::MatrixXd points_in;
    points_in.resize(nb_vertex_in, 2);
    for (int i = 0; i < nb_vertex; i++){
        if (old2new[i] != -1){
            points_in(old2new[i], 0) = points[i][0];
            points_in(old2new[i], 1) = points[i][1];
        }
    }
    wmtk::mesh_utils::set_matrix_attribute(points_in, "position", wmtk::PrimitiveType::Vertex, mesh);
    return mesh;
}