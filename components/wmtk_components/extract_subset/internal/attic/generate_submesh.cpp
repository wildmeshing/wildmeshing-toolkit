#pragma once

#include "generate_submesh.hpp"

namespace wmtk::components::internal {

// TODO: to be abandoned
wmtk::Mesh& generate_submesh(wmtk::Mesh& m, wmtk::MeshAttributeHandle<long> tag_handle, bool pos)
{
    /*
    Algo:
    1. get all the top simplices, for each top simplex, get the tag
    2. if tag is 1, store the index of this simplex
    3. for each vertex, get all the top simplices sharing this vertex
    4. check if the tag of the top simplices sharing the vertex is 1, if yes, store the index of the
    top simplex
    4. create a new mesh
    5. for each simplex, get the vertices and store them in the new mesh
    6. if pos is true, get the position of the vertices and store them in the new mesh
    */
    int top_simplex_dim = m.top_cell_dimension();
    if (top_simplex_dim != 2 && top_simplex_dim != 3)
        throw std::runtime_error("Invalid top dimension in separating topology!");
    wmtk::Accessor<long> tag_acc = m.create_accessor(tag_handle);
    wmtk::PrimitiveType topType = m.top_simplex_type();
    std::vector<wmtk::Tuple> top_simplices = m.get_all(topType);
    long top_simplex_count = top_simplices.size();
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = vertices.size();

    long nb_vertex_in = 0, nb_cell_in = 0;

    // store the temporary "id" of the tagged triangles
    std::vector<long> tag_tri_index;
    for (size_t i = 0; i < top_simplex_count; ++i) {
        long tri_tag = tag_acc.const_scalar_attribute(top_simplices.at(i));
        switch (tri_tag) {
        // inside: store the temp id of this tri
        case 1: tag_tri_index.push_back(i); break;
        // outside: do nothing
        case 0: break;
        // neither: runtime error
        default: throw std::runtime_error("illegal tag!");
        }
    }
    nb_cell_in = tag_tri_index.size();
    assert(nb_cell_in <= top_simplex_count);

    std::map<long, long> old2new;
    for (int i = 0; i < nb_vertex; ++i) {
        wmtk::simplex::SimplexCollection sc =
            wmtk::simplex::top_dimension_cofaces(m, Simplex::vertex(vertices[i]));
        for (wmtk::Simplex adj_simplex : sc) {
            wmtk::Tuple adj_simplex_tuple = adj_simplex.tuple();
            long l = tag_acc.const_scalar_attribute(adj_simplex_tuple);
            if (l == 1) {
                old2new.insert({i, nb_vertex_in});
                nb_vertex_in++;
                break;
            }
        }
    }
    if (top_simplex_dim == 2) {
        wmtk::TriMesh mesh;
        wmtk::RowVectors3l tris;
        tris.resize(nb_cell_in, 3);
        for (long i = 0; i < nb_cell_in; ++i) {
            Simplex s = Simplex::face(top_simplices[i]);
            auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            for (int j = 0; j < 3; ++j) {
                // tris.row(i) << dup_acc.vector_attribute(corners[j]);
            }
        }
        mesh.initialize(tris);
        // return std::make_unique<wmtk::TriMesh>(mesh);
    } else if (top_simplex_dim == 3) {
        wmtk::TetMesh mesh;
        wmtk::RowVectors4l tets;
        tets.resize(top_simplex_count, 4);
        for (long i = 0; i < top_simplex_count; ++i) {
            Simplex s = Simplex::tetrahedron(top_simplices[i]);
            auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            for (int j = 0; j < 4; ++j) {
                // tets.row(i) << dup_acc.vector_attribute(corners[j]);
            }
        }
        mesh.initialize(tets);
        // return std::make_unique<wmtk::TetMesh>(mesh);
    } else {
        throw std::runtime_error("Invalid top dimension in separating topology!");
    }

    return m;
}
} // namespace wmtk::components::internal