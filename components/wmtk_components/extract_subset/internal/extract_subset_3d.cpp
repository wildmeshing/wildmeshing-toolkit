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
        for (wmtk::Tuple t : tuple_list) vertices_in_bool[find_vertex_index<wmtk::TetMesh>(m, t)] = true;
    }




    return m;
}

} // namespace wmtk::components::internal