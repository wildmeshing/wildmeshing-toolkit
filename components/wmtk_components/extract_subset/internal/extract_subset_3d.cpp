#include "extract_subset_3d.hpp"

namespace wmtk::components::internal {
wmtk::TetMesh
extract_subset_3d(wmtk::TetMesh m, wmtk::MeshAttributeHandle<long> taghandle, bool pos)
{
    wmtk::Accessor<long> tag_acc = m.create_accessor(taghandle);
    std::vector<wmtk::Tuple> tets = m.get_all(wmtk::PrimitiveType::Tetrahedron);
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = m.capacity(wmtk::PrimitiveType::Vertex);
    int nb_tet = m.capacity(wmtk::PrimitiveType::Tetrahedron);

    std::map<long, bool> vertices_in_bool;
    for (long t = 0; t < nb_vertex; ++t) vertices_in_bool.insert({t, false});

    long nb_vertex_in = 0, nb_tet_in = 0;

    // store the temporary "id" of the tagged tets
    std::vector<long> tag_tet_index;
    for (size_t i = 0; i < nb_tet; ++i) {
        long tri_tag = tag_acc.const_scalar_attribute(tets.at(i));
        switch (tri_tag) {
        // inside: store the temp id of this tri
        case 1:
            nb_tet_in++;
            tag_tet_index.push_back(i);
            break;
        // outside: do nothing
        case 0: break;
        // neither: runtime error
        default: throw std::runtime_error("illegal tag!");
        }
    }
    assert(nb_tet_in <= nb_tet);

    // for the tagged tri, mark their "real" vertices as inside (duplicates handled by boolean)
    // current algo for bug fixing: O(N^2), go over all vertices and look for match,
    // only assign tag to inside ones
    // TODO: improve the algorithm to achieve O(N)
    for (size_t i = 0; i < nb_tet_in; ++i) {
        Simplex s = Simplex::tetrahedron(tets[tag_tet_index[i]]);
        std::vector<wmtk::Tuple> tuple_list =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        for (wmtk::Tuple t : tuple_list)
            vertices_in_bool[find_vertex_index<wmtk::TetMesh>(m, t)] = true;
    }

    wmtk::Accessor<long> tag_acc = m.create_accessor(taghandle);
    std::vector<wmtk::Tuple> tets = m.get_all(wmtk::PrimitiveType::Tetrahedron);
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = m.capacity(wmtk::PrimitiveType::Vertex);
    int nb_tet = m.capacity(wmtk::PrimitiveType::Tetrahedron);

    std::map<long, bool> vertices_in_bool;
    for (long t = 0; t < nb_vertex; ++t) vertices_in_bool.insert({t, false});

    long nb_vertex_in = 0, nb_tet_in = 0;

    // store the temporary "id" of the tagged tets
    std::vector<long> tag_tet_index;
    for (size_t i = 0; i < nb_tet; ++i) {
        long tri_tag = tag_acc.const_scalar_attribute(tets.at(i));
        switch (tri_tag) {
        // inside: store the temp id of this tri
        case 1:
            nb_tet_in++;
            tag_tet_index.push_back(i);
            break;
        // outside: do nothing
        case 0: break;
        // neither: runtime error
        default: throw std::runtime_error("illegal tag!");
        }
    }
    assert(nb_tet_in <= nb_tet);

    // for the tagged tri, mark their "real" vertices as inside (duplicates handled by boolean)
    // current algo for bug fixing: O(N^2), go over all vertices and look for match,
    // only assign tag to inside ones
    // TODO: improve the algorithm to achieve O(N)
    for (size_t i = 0; i < nb_tet_in; ++i) {
        Simplex s = Simplex::tetrahedron(tets[tag_tet_index[i]]);
        std::vector<wmtk::Tuple> tuple_list =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        for (wmtk::Tuple t : tuple_list) vertices_in_bool[find_vertex_index(m, t)] = true;
    }

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

    wmtk::TetMesh mesh;
    wmtk::RowVectors4l tris;
    tris.resize(nb_tet_in, 4);
    // only put in the extracted ones
    for (size_t i = 0; i < nb_tet_in; ++i) {
        Simplex s = Simplex::tetrahedron(tets[tag_tet_index[i]]);
        std::vector<wmtk::Tuple> list =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        std::vector<long> data(4, -1);
        for (int index = 0; index < 4; ++index)
            data[index] = old2new[find_vertex_index(m, list[index])];
        tris.row(i) << data[0], data[1], data[2], data[3];
    }
    return mesh;
}

} // namespace wmtk::components::internal