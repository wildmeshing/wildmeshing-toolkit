#include "extract_subset_2d.hpp"
#include <iostream>
namespace wmtk::components::internal {

wmtk::TriMesh
extract_subset_2d(wmtk::TriMesh m, wmtk::MeshAttributeHandle<long> tag_handle, bool pos)
{
    wmtk::Accessor<long> tag_acc = m.create_accessor(tag_handle);
    std::vector<wmtk::Tuple> faces = m.get_all(wmtk::PrimitiveType::Face);
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = vertices.size();
    int nb_tri = faces.size();

    // a tag on each "real" vertex, true if tagged inside
    std::map<long, bool> vertices_in_bool;
    for (long t = 0; t < nb_vertex; ++t) vertices_in_bool.insert({t, false});

    // both init to 0, increment by count later
    long nb_vertex_in = 0, nb_tri_in = 0;

    // store the temporary "id" of the tagged triangles
    std::vector<long> tag_tri_index;
    for (size_t i = 0; i < nb_tri; ++i) {
        long tri_tag = tag_acc.const_scalar_attribute(faces.at(i));
        switch (tri_tag) {
        // inside: store the temp id of this tri
        case 1:
            tag_tri_index.push_back(i);
            break;
        // outside: do nothing
        case 0: break;
        // neither: runtime error
        default: throw std::runtime_error("illegal tag!");
        }
    }
    nb_tri_in = tag_tri_index.size();
    assert(nb_tri_in <= nb_tri);
    // std::cout << "# of tri inside = " << nb_tri_in << std::endl;

    // for the tagged tri, mark their "real" vertices as inside (duplicates handled by boolean)
    // current algo for bug fixing: O(N^2), go over all vertices and look for match,
    // only assign tag to inside ones
    // TODO: improve the algorithm to achieve O(N)
    for (size_t i = 0; i < nb_tri_in; ++i) {
        Simplex s = Simplex::face(faces[tag_tri_index[i]]);
        std::vector<wmtk::Tuple> tuple_list =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        for (wmtk::Tuple t : tuple_list) vertices_in_bool[find_vertex_index(m, t)] = true;
    }

    // std::cout << "# of vertex inside = " << vertices_in_bool.size() << std::endl;
    // construct a map from old tuple to temp new "id" of a "real" vertex
    std::map<long, long> old2new;
    for (long i = 0; i < nb_vertex; ++i) {
        if (vertices_in_bool[i]) {
            // std::cout << "inside! nb_vertex_in = " << nb_vertex_in << std::endl;
            // old vertex tuple t mapped to new vertex id j, where j increases by count
            old2new.insert({i, nb_vertex_in});
            nb_vertex_in++;
        }
    }
    // std::cout << "# of pairs in map = " << old2new.size() << std::endl;
    // for (auto [_, i] : old2new) std::cout << i << std::endl;

    wmtk::TriMesh mesh;
    wmtk::RowVectors3l tris;
    tris.resize(nb_tri_in, 3);
    // only put in the extracted ones
    for (size_t i = 0; i < nb_tri_in; ++i) {
        Simplex s = Simplex::face(faces[tag_tri_index[i]]);
        std::vector<wmtk::Tuple> list =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        std::vector<long> data(3, -1);
        for (int index = 0; index < 3; ++index)
            data[index] = old2new[find_vertex_index(m, list[index])];
        tris.row(i) << data[0], data[1], data[2];
    }
    // for (size_t i = 0; i < nb_tri_in; ++i) {
    //     std::cout << tris.row(i)[0] << tris.row(i)[1] << tris.row(i)[2] << std::endl;
    // }
    mesh.initialize(tris); // init the topology

    // if told to extract and preserve the coordinates
    if (pos) {
        Eigen::MatrixXd points_in;
        points_in.resize(nb_vertex_in, 2);
        wmtk::MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        wmtk::ConstAccessor<double> pos_acc = m.create_const_accessor(pos_handle);
        for (const Tuple& t : vertices) {
            // ignore the outside vertices
            long old_index = find_vertex_index(m, t);
            if (vertices_in_bool[old_index]) {
                points_in.row(old2new[old_index]) = pos_acc.const_vector_attribute(t);
            }
            wmtk::mesh_utils::set_matrix_attribute(
                points_in,
                "position",
                wmtk::PrimitiveType::Vertex,
                mesh);
        }
    }
    return mesh;
}
} // namespace wmtk::components::internal