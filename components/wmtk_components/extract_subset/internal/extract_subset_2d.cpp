#include "extract_subset_2d.hpp"
#include <iostream>
namespace wmtk::components::internal {

wmtk::TriMesh
extract_subset_2d(wmtk::TriMesh m, wmtk::MeshAttributeHandle<long> tag_handle, bool pos)
{
    auto tag_acc = m.create_accessor(tag_handle);
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = m.capacity(wmtk::PrimitiveType::Vertex);
    int nb_tri = m.capacity(wmtk::PrimitiveType::Face);

    // storing whether each vertex is tagged inside, false by default
    std::map<wmtk::Tuple, bool> vertices_in_bool;
    for (auto t : vertices) vertices_in_bool.insert({t, false});

    // both init to 0, increment by count later
    long nb_vertex_in = 0, nb_tri_in = 0;

    // store the temporary "id" of the tagged triangles
    std::vector<long> tag_tri_index;
    for (size_t i = 0; i < nb_tri; ++i) {
        long tri_tag = tag_acc.const_scalar_attribute(faces.at(i));
        switch (tri_tag) {
        // inside: store the temp id of this tri
        case 1:
            nb_tri_in++;
            tag_tri_index.push_back(i);
            break;
        // outside: do nothing
        case 0: break;
        // neither: runtime error
        default: throw std::runtime_error("illegal tag!");
        }
    }
    assert(nb_tri_in <= m.capacity(wmtk::PrimitiveType::Face));
    // std::cout << "# of tri inside = " << nb_tri_in << std::endl;

    // for the tagged tri, mark their vertices as inside (duplicates handled by boolean)
    // current algo for bug fixing: O(N^2), go over all vertices and look for match, 
    // only assign tag to inside ones
    // TODO: improve the algorithm to achieve O(N)
    for (size_t i = 0; i < nb_tri_in; ++i) {
        Simplex s = Simplex::face(faces[tag_tri_index[i]]);
        auto tuple_list = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        for (auto t : tuple_list) {
            // This inner loop gives you additional N complexity
            for (int j = 0; j < vertices.size(); ++j) {
                if (m.simplices_are_equal(
                        wmtk::Simplex::vertex(t),
                        wmtk::Simplex::vertex(vertices[j]))) {
                    vertices_in_bool[vertices[j]] = true;
                    break;
                }
            }
        }
    }

    // std::cout << "# of vertex inside = " << vertices_in_bool.size() << std::endl;
    // construct a map from old tuple to temp new "id" of a vertex
    std::map<const wmtk::Tuple, long> old2new;
    for (auto t : vertices) {
        if (vertices_in_bool.at(t)) {
            // std::cout << "inside! nb_vertex_in = " << nb_vertex_in << std::endl;
            // old vertex tuple t mapped to new vertex id j, where j increases by count
            old2new.insert({t, nb_vertex_in});
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
        auto list = wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        std::vector<long> data(3, -1);
        for (int index = 0; index < 3; ++index) {
            for (int j = 0; j < vertices.size(); ++j) {
                if (m.simplices_are_equal(
                        wmtk::Simplex::vertex(list[index]),
                        wmtk::Simplex::vertex(vertices[j]))) {
                    data[index] = old2new[vertices[j]];
                    break;
                }
            }
        }
        tris.row(i) << data[0], data[1], data[2];
        // tris.row(i) << old2new[list[0]], old2new[list[1]], old2new[list[2]];
    }
    // for (size_t i = 0; i < nb_tri_in; ++i) {
    //     std::cout << tris.row(i)[0] << tris.row(i)[1] << tris.row(i)[2] << std::endl;
    // }
    mesh.initialize(tris); // init the topology

    // if told to extract and preserve the coordinates
    if (pos) {
        Eigen::MatrixXd points_in;
        points_in.resize(nb_vertex_in, 2);
        auto pos_handle = m.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        auto pos_acc = m.create_const_accessor(pos_handle);
        for (const Tuple& t : vertices) {
            // ignore the outside vertices
            if (vertices_in_bool[t]) {
                points_in.row(old2new[t]) = pos_acc.const_vector_attribute(t);
            }
        }
        wmtk::mesh_utils::set_matrix_attribute(
            points_in,
            "position",
            wmtk::PrimitiveType::Vertex,
            mesh);
    }
    return mesh;
}
} // namespace wmtk::components::internal