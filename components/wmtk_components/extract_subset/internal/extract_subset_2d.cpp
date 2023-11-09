#include "extract_subset_2d.hpp"


namespace wmtk::components::internal {

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
    for (size_t i = 0; i < nb_tri_in; ++i){
        for (size_t j = 0; j < 3; ++j) {
            // faces_in(k, k2) = triangles(tag[k], k2);
            vertices_in_bool[triangles(tag[i], j)] = true;
        }
    }
    for (bool b: vertices_in_bool) {
        if (b) nb_vertex_in ++;
    }

    // construct a map from old vertex id to new new id
    std::map<int, int> old2new;
    int counter_in = 0;
    for (int i = 0; i < nb_vertex; ++i){
        // ignore the not extracted vertices
        if (vertices_in_bool[i]){
            // old vertex id i map to new vertex id j, where j increases by count
            old2new.insert({i, counter_in});
            counter_in++;
        }
    }
    assert(counter_in == nb_vertex_in);

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
    return mesh;
}

wmtk::TriMesh extract_subset_2d(
    wmtk::TriMesh m, 
    wmtk::MeshAttributeHandle<long> tag_handle){

    auto tag_acc = m.create_accessor(tag_handle);
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = m.capacity(wmtk::PrimitiveType::Vertex);
    int nb_tri = m.capacity(wmtk::PrimitiveType::Face);

    // storing whether each vertex is tagged inside, false by default
    std::map<wmtk::Tuple, bool> vertices_in_bool;
    for (auto t: vertices) vertices_in_bool.insert({t, false});
    
    // both init to 0, increment by count later
    long nb_vertex_in = 0, nb_tri_in = 0;

    // store the temporary "id" of the tagged triangles
    std::vector<long> tag_tri_index;
    for (size_t i = 0; i < nb_tri; ++i){
        long tri_tag = tag_acc.const_scalar_attribute(faces.at(i));
        switch (tri_tag) {
            // inside: store the temp id of this tri
            case 1: nb_tri_in++; tag_tri_index.push_back(i); break;
            // outside: do nothing
            case 0: break;
            // neither: runtime error
            default: throw std::runtime_error("illegal tag!");
        }
    }

    // for the tagged tri, mark their vertices as inside (duplicates handled by boolean)
    for (size_t i = 0; i < nb_tri_in; ++i){
        Simplex s = Simplex::face(faces[tag_tri_index[i]]);
        auto tuple_list = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        for (auto t: tuple_list) {
            vertices_in_bool[t] = true;
        }
    }

    // construct a map from old tuple to temp new "id" of a vertex
    std::map<const wmtk::Tuple, long> old2new;
    for (auto t: vertices){
        // ignore the not extracted vertices
        if (vertices_in_bool[t]){
            // old vertex tuple t mapped to new vertex id j, where j increases by count
            old2new.insert({t, nb_vertex_in});
            nb_vertex_in++;
        }
    }

    wmtk::TriMesh mesh;
    wmtk::RowVectors3l tris;
    tris.resize(nb_tri_in, 3);
    // only put in the extracted ones
    for (size_t i = 0; i < nb_tri_in; ++i){
        Simplex s = Simplex::face(faces[tag_tri_index[i]]);
        auto list = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        assert(list.size() == 3);
        tris.row(i) << old2new[list[0]], old2new[list[1]], old2new[list[2]];
    }
    mesh.initialize(tris); // init the topology

    Eigen::MatrixXd points_in;
    points_in.resize(nb_vertex_in, 2);
    auto pos_handle = m.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    auto pos_acc = m.create_const_accessor(pos_handle);
    for (auto t: vertices){
        // ignore the outside vertices
        if (vertices_in_bool[t]){
            // QUESTION: not sure, is this the legal way to get the coord of a vertex tuple?
            points_in.row(old2new[t]) << pos_acc.const_vector_attribute(t);
        }
    }
    wmtk::mesh_utils::set_matrix_attribute(points_in, "position", wmtk::PrimitiveType::Vertex, mesh);
    return mesh;
}
}