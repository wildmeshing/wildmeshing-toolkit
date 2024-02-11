#include "generate_submesh.hpp"

namespace wmtk::components::internal {

// Getting submesh and operate on it is essential because extracting subset will change connectivity
std::unique_ptr<wmtk::Mesh>
generate_submesh(wmtk::Mesh& m, wmtk::MeshAttributeHandle<long> tag_handle, bool pos)
{
    /*
    [I didn't implement the algo listed here, I just restored the prev dumb version]
    (Incorrect) Algo:
    1. register a new attribute to each vertex and init as -1
    2. store all the indices of tagged top simplices
    3. for each tagged top dim simplex, for each vertex of the simplex,
        if the vertex has attribute -1, then assign it a new index, increment the counter (leave it
        there if already having an index)

    Reconstruction: For each top dim simplex, get the tag if tag is 1
    get attribute of all vertices of the simplex, and store them in a new mesh
    if pos is true, get the position of the vertices and store them in the new mesh
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

    std::map<long, bool> vertices_in_bool;
    for (long t = 0; t < nb_vertex; ++t) vertices_in_bool.insert({t, false});

    // // Step 1: register a new attribute to each vertex and init as -1
    // wmtk::VectorXl submesh_index_vector;
    // submesh_index_vector.resize(nb_vertex, 1);
    // for (long i = 0; i < nb_vertex; ++i) submesh_index_vector.row(i) << -1;
    // wmtk::MeshAttributeHandle<long> duplicate_handle = wmtk::mesh_utils::set_matrix_attribute(
    //     submesh_index_vector,
    //     "submesh_index",
    //     wmtk::PrimitiveType::Vertex,
    //     m);
    // wmtk::Accessor<long> dup_acc = m.create_accessor(duplicate_handle);


    long nb_vertex_in = 0, nb_cell_in = 0;
    // Step 2: store all the indices of tagged top simplices
    std::vector<long> tag_simplex_index;
    for (size_t i = 0; i < top_simplex_count; ++i) {
        long tri_tag = tag_acc.const_scalar_attribute(top_simplices.at(i));
        switch (tri_tag) {
        // inside: store the temp id of this tri
        case 1: tag_simplex_index.push_back(i); break;
        // outside: do nothing
        case 0: break;
        // neither: runtime error
        default: throw std::runtime_error("illegal tag!");
        }
    }
    nb_cell_in = tag_simplex_index.size();
    assert(nb_cell_in <= top_simplex_count);

    // // Step 3.1: for each tagged top dim simplex
    // for (size_t index : tag_simplex_index) {
    //     Simplex s = (top_simplex_dim == 2) ? Simplex::face(top_simplices[index])
    //                                        : Simplex::tetrahedron(top_simplices[index]);
    //     std::vector<wmtk::Tuple> vertices =
    //         wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
    //     // for each vertex of the simplex
    //     for (wmtk::Tuple t : vertices) {
    //         // Step 3.2: if the vertex has attribute -1, then assign it a new index,
    //         if (dup_acc.scalar_attribute(t) == -1) {
    //         }
    //     }
    // }

    // TODO: improve the algorithm to achieve O(N)
    for (size_t i = 0; i < nb_cell_in; ++i) {
        Simplex s = (top_simplex_dim == 2)
                        ? Simplex::face(top_simplices[tag_simplex_index[i]])
                        : Simplex::tetrahedron(top_simplices[tag_simplex_index[i]]);
        std::vector<wmtk::Tuple> tuple_list =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        for (wmtk::Tuple t : tuple_list) vertices_in_bool[find_vertex_index(m, t)] = true;
    }

    std::map<long, long> old2new;
    for (long i = 0; i < nb_vertex; ++i) {
        if (vertices_in_bool[i]) {
            // std::cout << "inside! nb_vertex_in = " << nb_vertex_in << std::endl;
            // old vertex tuple t mapped to new vertex id j, where j increases by count
            old2new.insert({i, nb_vertex_in});
            nb_vertex_in++;
        }
    }

    // std::map<long, long> old2new;
    // for (int i = 0; i < nb_vertex; ++i) {
    //     wmtk::simplex::SimplexCollection sc =
    //         wmtk::simplex::top_dimension_cofaces(m, Simplex::vertex(vertices[i]));
    //     for (wmtk::Simplex adj_simplex : sc) {
    //         wmtk::Tuple adj_simplex_tuple = adj_simplex.tuple();
    //         long l = tag_acc.const_scalar_attribute(adj_simplex_tuple);
    //         if (l == 1) {
    //             old2new.insert({i, nb_vertex_in});
    //             nb_vertex_in++;
    //             break;
    //         }
    //     }
    // }
    wmtk::RowVectors4l tris;
    tris.resize(nb_cell_in, m.top_cell_dimension() + 1);
    for (size_t i = 0; i < nb_cell_in; ++i) {
        Simplex s = (top_simplex_dim == 2)
                        ? Simplex::face(top_simplices[tag_simplex_index[i]])
                        : Simplex::tetrahedron(top_simplices[tag_simplex_index[i]]);
        std::vector<wmtk::Tuple> list =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        std::vector<long> data(4, -1);
        for (int index = 0; index < m.top_cell_dimension() + 1; ++index) {
            data[index] = old2new[find_vertex_index(m, list[index])];
            tris.row(i) << data[index];
        }
        // tris.row(i) << data[0], data[1], data[2];
    }

    if (top_simplex_dim == 2) {
        wmtk::TriMesh mesh;
        mesh.initialize(tris);
        return std::make_unique<wmtk::TriMesh>(mesh);
        // return std::make_unique<wmtk::TriMesh>(mesh);
    } else if (top_simplex_dim == 3) {
        wmtk::TetMesh mesh;
        mesh.initialize(tris);
        return std::make_unique<wmtk::TetMesh>(mesh);
        // return std::make_unique<wmtk::TetMesh>(mesh);
    } else
        throw std::runtime_error("Invalid top dimension in separating topology!");

    // if (top_simplex_dim == 2) {
    //     wmtk::TriMesh mesh;
    //     wmtk::RowVectors3l tris;
    //     tris.resize(nb_cell_in, 3);
    //     for (long i = 0; i < nb_cell_in; ++i) {
    //         Simplex s = Simplex::face(top_simplices[i]);
    //         auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
    //         for (int j = 0; j < 3; ++j) {
    //             // tris.row(i) << dup_acc.vector_attribute(corners[j]);
    //         }
    //     }
    //     mesh.initialize(tris);
    //     // return std::make_unique<wmtk::TriMesh>(mesh);
    // } else if (top_simplex_dim == 3) {
    //     wmtk::TetMesh mesh;
    //     wmtk::RowVectors4l tets;
    //     tets.resize(top_simplex_count, 4);
    //     for (long i = 0; i < top_simplex_count; ++i) {
    //         Simplex s = Simplex::tetrahedron(top_simplices[i]);
    //         auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
    //         for (int j = 0; j < 4; ++j) {
    //             // tets.row(i) << dup_acc.vector_attribute(corners[j]);
    //         }
    //     }
    //     mesh.initialize(tets);
    //     // return std::make_unique<wmtk::TetMesh>(mesh);
    // } else {
    //     throw std::runtime_error("Invalid top dimension in separating topology!");
    // }

    // return m;
}
} // namespace wmtk::components::internal