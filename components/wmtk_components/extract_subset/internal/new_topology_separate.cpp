#include "new_topology_separate.hpp"
#include <iostream>
namespace wmtk::components::internal {

// general function to separate topology, regardless of dimension
std::unique_ptr<wmtk::Mesh>
topology_separate(wmtk::Mesh& m, const wmtk::MeshAttributeHandle<long>& tag_handle, bool pos)
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
    wmtk::Accessor<long> tag_acc = m.create_accessor(tag_handle);
    wmtk::PrimitiveType topType = m.top_simplex_type();
    std::vector<wmtk::Tuple> top_simplices = m.get_all(topType);
    long top_simplex_count = top_simplices.size();
    long counter = 0;
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = vertices.size();
    std::map<long, bool> vertices_in_bool;
    for (long t = 0; t < nb_vertex; ++t) vertices_in_bool.insert({t, false});

    long nb_vertex_in = 0, nb_cell_in = 0;
    std::vector<long> tag_simplex_index;
    for (size_t i = 0; i < top_simplex_count; ++i) {
        long cell_tag = tag_acc.const_scalar_attribute(top_simplices.at(i));
        switch (cell_tag) {
        // inside: store the temp id of this cell
        case 1: tag_simplex_index.push_back(i); break;
        // outside: do nothing
        case 0: break;
        // neither: runtime error
        default: throw std::runtime_error("illegal tag!");
        }
    }
    nb_cell_in = tag_simplex_index.size();
    assert(nb_cell_in <= top_simplex_count);
    std::cout << "# of cell inside = " << nb_cell_in << std::endl;

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
            // std::cout << "inside! vertex_index = " << i << std::endl;
            // old vertex tuple t mapped to new vertex id j, where j increases by count
            old2new.insert({i, nb_vertex_in});
            nb_vertex_in++;
        }
    }
    std::cout << "nb_vertex_in = " << nb_vertex_in << std::endl;

    wmtk::TriMesh tri_ext_mesh;
    wmtk::RowVectors3l tri_exts;
    wmtk::TetMesh tet_ext_mesh;
    wmtk::RowVectors4l tet_exts;

    if (top_simplex_dim == 2) {
        tri_exts.resize(nb_cell_in, 3);
        for (int i = 0; i < nb_cell_in; ++i) {
            Simplex s = Simplex::face(top_simplices[tag_simplex_index[i]]);
            std::vector<wmtk::Tuple> list =
                wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            std::vector<long> data(3, -1);
            for (int index = 0; index < 3; ++index)
                data[index] = old2new[find_vertex_index(m, list[index])];
            tri_exts.row(i) << data[0], data[1], data[2];
        }
        // std::cout << "tri_exts = " << tri_exts << std::endl;
        tri_ext_mesh.initialize(tri_exts);
        assert(tri_ext_mesh.get_all(topType).size() == nb_cell_in);
        if (!pos) return std::make_unique<wmtk::TriMesh>(tri_ext_mesh);
    } else if (top_simplex_dim == 3) {
        tet_exts.resize(nb_cell_in, m.top_cell_dimension() + 1);
        for (size_t i = 0; i < nb_cell_in; ++i) {
            Simplex s = Simplex::tetrahedron(top_simplices[tag_simplex_index[i]]);
            std::vector<wmtk::Tuple> list =
                wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            std::vector<long> data(m.top_cell_dimension() + 1, -1);
            for (int index = 0; index < 4; ++index)
                data[index] = old2new[find_vertex_index(m, list[index])];
            tet_exts.row(i) << data[0], data[1], data[2], data[3];
        }
        tet_ext_mesh.initialize(tet_exts);
        assert(tet_ext_mesh.get_all(topType).size() == nb_cell_in);
        if (!pos) return std::make_unique<wmtk::TetMesh>(tet_ext_mesh);
    }

    // if told to, extract and preserve the coordinates
    if (pos) {
        Eigen::MatrixXd points_in;
        points_in.resize(nb_vertex_in, m.top_cell_dimension());
        wmtk::MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        wmtk::ConstAccessor<double> pos_acc = m.create_const_accessor(pos_handle);
        for (const Tuple& t : vertices) {
            // ignore the outside vertices
            long old_index = find_vertex_index(m, t);
            if (vertices_in_bool[old_index]) {
                points_in.row(old2new[old_index]) = pos_acc.const_vector_attribute(t);
            }
            // call the set_matrix_attribute function according to the top dimension
            switch (m.top_cell_dimension()) {
            case 2:
                wmtk::mesh_utils::set_matrix_attribute(
                    points_in,
                    "position",
                    wmtk::PrimitiveType::Vertex,
                    tri_ext_mesh);
                return std::make_unique<wmtk::TriMesh>(tri_ext_mesh);
            case 3:
                wmtk::mesh_utils::set_matrix_attribute(
                    points_in,
                    "position",
                    wmtk::PrimitiveType::Vertex,
                    tet_ext_mesh);
                return std::make_unique<wmtk::TetMesh>(tet_ext_mesh);
            default: throw std::runtime_error("Invalid top dimension in separating topology!");
            }
        }
    }

    // first, register a vector attribute to store the corner ids for each top dimension simplex
    wmtk::RowVectors4l dup;
    dup.resize(top_simplex_count, 4);
    for (long i = 0; i < top_simplex_count; ++i) dup.row(i) << -1, -1, -1, -1;
    wmtk::MeshAttributeHandle<long> duplicate_handle =
        wmtk::mesh_utils::set_matrix_attribute(dup, "duplicate_index", topType, m);
    wmtk::Accessor<long> dup_acc = m.create_accessor(duplicate_handle);

    std::vector<long> tag_index;
    // second, go over all top dimension simplices and adjust the duplicate index
    for (long i = 0; i < top_simplex_count; ++i) {
        long tri_tag = tag_acc.const_scalar_attribute(top_simplices.at(i));
        std::cout << "i = " << i << ", tag = " << tri_tag << std::endl;
        // only consider the top simplices with tag 1
        if (tri_tag == 0) continue;
        tag_index.push_back(i);
        auto v = dup_acc.vector_attribute(top_simplices[i]);
        // Question: Why can't I use the constructor here to build a Simplex?
        Simplex s = (top_simplex_dim == 2) ? Simplex::face(top_simplices[i])
                                           : Simplex::tetrahedron(top_simplices[i]);
        // get all corners for current top simplex
        auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        std::cout << "Hello1, # of corners = " << top_simplex_dim + 1 << std::endl;
        for (long j = 0; i < top_simplex_dim + 1; ++j) {
            // check whether it has been visited
            std::cout << "j = " << j << ", v[j] = " << v[j] << std::endl;
            if (v[j] != -1) continue;
            // if the corner has not been assigned a duplicate index, assign it
            v[j] = counter;

            // find all top simplices sharing the same corner vertex,
            // update duplicate index of theie corner accordingly

            // get all top dimension simplices sharing the vertex and are face-connected
            wmtk::simplex::SimplexCollection sc =
                wmtk::simplex::top_dimension_cofaces(m, Simplex::vertex(corners[j]));
            std::cout << "num of face-connected simplices = " << sc.simplex_vector().size()
                      << std::endl;
            for (wmtk::Simplex adj_simplex : sc) {
                // tuple for a top dimension simplex would be the same as tuple for the corner
                wmtk::Tuple adj_corner_tuple = adj_simplex.tuple();
                auto adj_vector = dup_acc.vector_attribute(adj_corner_tuple);
                long k = adj_corner_tuple.get_local_vid();
                assert(adj_vector[k] == -1);
                adj_vector[k] = counter;
                std::cout << "after adjusting, = " << dup_acc.vector_attribute(adj_corner_tuple)[k]
                          << std::endl;
            }
            // finally, increment the counter
            counter++;
        }
        std::cout << "Hello3" << std::endl;
    }
    long tag_count = tag_index.size();

    // third, create a new mesh and copy the topology
    // TODO: also copy the geometry if asked to do so
    if (top_simplex_dim == 2) {
        wmtk::TriMesh mesh;
        wmtk::RowVectors3l tris;
        tris.resize(tag_count, 3);
        for (long i = 0; i < top_simplex_count; ++i) {
            if (tag_acc.const_scalar_attribute(top_simplices.at(i)) == 0) continue;
            Simplex s = Simplex::face(top_simplices[i]);
            auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            for (int j = 0; j < 3; ++j) {
                tris.row(i) << dup_acc.vector_attribute(corners[j]);
            }
        }
        mesh.initialize(tris);
        return std::make_unique<wmtk::TriMesh>(mesh);
    } else {
        wmtk::TetMesh mesh;
        wmtk::RowVectors4l tets;
        tets.resize(tag_count, 4);
        for (long i = 0; i < top_simplex_count; ++i) {
            if (tag_acc.const_scalar_attribute(top_simplices.at(i)) == 0) continue;
            Simplex s = Simplex::tetrahedron(top_simplices[i]);
            auto corners = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            for (int j = 0; j < 4; ++j) {
                tets.row(i) << dup_acc.vector_attribute(corners[j]);
            }
        }
        mesh.initialize(tets);
        return std::make_unique<wmtk::TetMesh>(mesh);
    }
}
} // namespace wmtk::components::internal