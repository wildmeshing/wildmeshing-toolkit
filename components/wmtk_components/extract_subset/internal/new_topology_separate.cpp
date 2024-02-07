#include "new_topology_separate.hpp"
#include <iostream>
namespace wmtk::components::internal {

// general function to separate topology, regardless of dimension
std::unique_ptr<wmtk::Mesh> topology_separate(wmtk::Mesh& m)
{
    int top_simplex_dim = m.top_cell_dimension();
    if (top_simplex_dim != 2 && top_simplex_dim != 3)
        throw std::runtime_error("Invalid top dimension in separating topology!");
    wmtk::PrimitiveType topType = m.top_simplex_type();
    std::vector<wmtk::Tuple> top_simplices = m.get_all(topType);
    long top_simplex_count = top_simplices.size();
    long counter = 0;

    // first, register a vector attribute to store the corner ids for each top dimension simplex
    wmtk::RowVectors4l dup;
    for (long i = 0; i < top_simplex_count; ++i) {
        for (int j = 0; j < top_simplex_dim + 1; ++j) {
            dup.row(i) << -1;
        }
    }
    wmtk::MeshAttributeHandle<long> duplicate_handle =
        wmtk::mesh_utils::set_matrix_attribute(dup, "duplicate_index", topType, m);
    wmtk::Accessor<long> dup_acc = m.create_accessor(duplicate_handle);

    // second, go over all top dimension simplices and adjust the duplicate index
    for (long i = 0; i < top_simplex_count; ++i) {
        auto v = dup_acc.vector_attribute(top_simplices[i]);
        // Question: Why can't I use the constructor here to build a Simplex?
        Simplex s = (top_simplex_dim == 2) ? Simplex::face(top_simplices[i])
                                           : Simplex::tetrahedron(top_simplices[i]);
        auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        for (long j = 0; i < top_simplex_dim + 1; ++j) {
            if (v[j] == -1) {
                // if the corner has not been assigned a duplicate index, assign it
                v[j] = counter;

                // find all top simplices sharing the same corner vertex,
                // update duplicate index of theie corner accordingly

                // get all top dimension simplices sharing the corner vertex
                wmtk::simplex::SimplexCollection sc =
                    wmtk::simplex::top_dimension_cofaces(m, Simplex::vertex(corners[j]));
                for (wmtk::Simplex adj_simplex : sc) {
                    // tuple for a top dimension simplex would be the same as tuple for the corner
                    wmtk::Tuple adj_corner_tuple = adj_simplex.tuple();
                    auto adj_vector = dup_acc.vector_attribute(adj_corner_tuple);
                    long k = adj_corner_tuple.get_local_vid();
                    assert(adj_vector[k] == -1);
                    adj_vector[k] = counter;
                }
            }
            // finally, increment the counter
            counter++;
        }
    }

    // third, create a new mesh and copy the topology
    if (top_simplex_dim == 2) {
        wmtk::TriMesh mesh;
        wmtk::RowVectors3l tris;
        tris.resize(top_simplex_count, 3);
        for (long i = 0; i < top_simplex_count; ++i) {
            Simplex s = Simplex::face(top_simplices[i]);
            auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            for (int j = 0; j < 3; ++j) {
                tris.row(i) << dup_acc.vector_attribute(corners[j]);
            }
        }
        mesh.initialize(tris);
        return std::make_unique<wmtk::TriMesh>(mesh);
    } else if (top_simplex_dim == 3) {
        wmtk::TetMesh mesh;
        wmtk::RowVectors4l tets;
        tets.resize(top_simplex_count, 4);
        for (long i = 0; i < top_simplex_count; ++i) {
            Simplex s = Simplex::tetrahedron(top_simplices[i]);
            auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            for (int j = 0; j < 4; ++j) {
                tets.row(i) << dup_acc.vector_attribute(corners[j]);
            }
        }
        mesh.initialize(tets);
        return std::make_unique<wmtk::TetMesh>(mesh);
    } else {
        throw std::runtime_error("Invalid top dimension in separating topology!");
    }
}
} // namespace wmtk::components::internal