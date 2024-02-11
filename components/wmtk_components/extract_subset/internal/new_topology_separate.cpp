#include "new_topology_separate.hpp"
#include <iostream>
namespace wmtk::components::internal {

// general function to separate topology, regardless of dimension
std::unique_ptr<wmtk::Mesh> topology_separate(wmtk::Mesh& m, bool pos)
{
    // First extract the non-manifold submesh, then execute the following algo
    /*Algo outline:
    1. get subset of top simplices with tag 1, reconstruct a submesh that is non-manifold
    2. for each top simplex, get all corners and assign them a unique index
    3. for each corner, get all top simplices sharing the corner and update their duplicate index
    4. create a new mesh and copy the topology
    5. if pos is true, copy the geometry as well
    */
    int top_simplex_dim = m.top_cell_dimension();
    if (top_simplex_dim != 2 && top_simplex_dim != 3)
        throw std::runtime_error("Invalid top dimension in separating topology!");
    wmtk::PrimitiveType topType = m.top_simplex_type();
    std::vector<wmtk::Tuple> top_simplices = m.get_all(topType);
    long top_simplex_count = top_simplices.size();
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = vertices.size();

    // Now begins the second part of the algorithm: make it topologically manifold
    long counter = 0;

    // first, register a vector attribute to store the corner ids for each top dimension simplex
    wmtk::RowVectors4l dup;
    dup.resize(top_simplex_count, top_simplex_dim + 1);
    for (long i = 0; i < top_simplex_count; ++i) {
        switch (top_simplex_dim) {
        case 2: dup.row(i) << -1, -1, -1; break;
        case 3: dup.row(i) << -1, -1, -1, -1; break;
        default: break;
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
        // get all corners for current top simplex
        auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        // std::cout << "Hello1, # of corners = " << corners.size() << std::endl;
        for (long j = 0; j < corners.size(); ++j) {
            // check whether it has been visited
            // std::cout << "j = " << j << ", vertex id = " << find_vertex_index(m, corners[j])
            //           << ", v[j] = " << v[j] << std::endl;
            if (v[j] != -1) continue;
            // if the corner has not been assigned a duplicate index, assign it
            v[j] = counter;

            // find all top simplices sharing the same corner vertex,
            // update duplicate index of theie corner accordingly

            // get all top dimension simplices sharing the vertex and are face-connected
            wmtk::simplex::SimplexCollection sc =
                wmtk::simplex::top_dimension_cofaces(m, Simplex::vertex(corners[j]));
            for (wmtk::Simplex adj_simplex : sc) {
                // tuple for a top dimension simplex would be the same as tuple for the corner
                wmtk::Tuple adj_corner_tuple = adj_simplex.tuple();
                auto adj_vector = dup_acc.vector_attribute(adj_corner_tuple);
                long k = adj_corner_tuple.get_local_vid();
                // std::cout << "before adjusting, = " << adj_vector[k] << std::endl;
                if (adj_vector[k] == counter) continue;
                if (adj_vector[k] != -1 && adj_vector[k] != counter)
                    throw std::runtime_error("Duplicate index conflict!");
                adj_vector[k] = counter;
                // std::cout << "after adjusting, = " <<
                // dup_acc.vector_attribute(adj_corner_tuple)[k]
                //           << std::endl;
            }
            // finally, increment the counter
            counter++;
        }
    }

    // third, create a new mesh and copy the topology
    Eigen::MatrixXd points_in;
    if (pos) {
        std::map<long, bool> vertices_pos_visited;
        for (long t = 0; t < counter + 1; ++t) vertices_pos_visited.insert({t, false});
        points_in.resize(counter + 1, top_simplex_dim);
        wmtk::MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        wmtk::ConstAccessor<double> pos_acc = m.create_const_accessor(pos_handle);
        for (int i = 0; i < top_simplex_count; ++i) {
            Simplex s = (top_simplex_dim == 2) ? Simplex::face(top_simplices[i])
                                               : Simplex::tetrahedron(top_simplices[i]);
            std::vector<wmtk::Tuple> corners =
                wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            for (int j = 0; j < top_simplex_dim + 1; ++j) {
                long index = dup_acc.vector_attribute(top_simplices[i])[j];
                if (!vertices_pos_visited[index]) {
                    points_in.row(index) = pos_acc.const_vector_attribute(corners[j]);
                    vertices_pos_visited[index] = true;
                }
            }
        }
    }

    if (top_simplex_dim == 2) {
        wmtk::TriMesh mesh;
        wmtk::RowVectors3l tris;
        tris.resize(top_simplex_count, 3);
        for (long i = 0; i < top_simplex_count; ++i) {
            auto v = dup_acc.vector_attribute(top_simplices[i]);
            tris.row(i) << v[0], v[1], v[2];
        }
        mesh.initialize(tris);
        if (pos) {
            wmtk::mesh_utils::set_matrix_attribute(
                points_in,
                "position",
                wmtk::PrimitiveType::Vertex,
                mesh);
        }
        return std::make_unique<wmtk::TriMesh>(mesh);
    } else {
        wmtk::TetMesh mesh;
        wmtk::RowVectors4l tets;
        tets.resize(top_simplex_count, 4);
        for (long i = 0; i < top_simplex_count; ++i) {
            auto v = dup_acc.vector_attribute(top_simplices[i]);
            tets.row(i) << v[0], v[1], v[2], v[3];
        }
        mesh.initialize(tets);
        if (pos) {
            wmtk::mesh_utils::set_matrix_attribute(
                points_in,
                "position",
                wmtk::PrimitiveType::Vertex,
                mesh);
        }
        return std::make_unique<wmtk::TetMesh>(mesh);
    }
}
} // namespace wmtk::components::internal