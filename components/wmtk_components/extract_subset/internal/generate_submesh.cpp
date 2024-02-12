#include "generate_submesh.hpp"

namespace wmtk::components::internal {

// Getting submesh and operate on it is essential because extracting subset will change connectivity
std::unique_ptr<wmtk::Mesh>
generate_submesh(wmtk::Mesh& m, wmtk::MeshAttributeHandle<long> tag_handle, bool pos)
{
    /*
    new Algo for this first part of plainly getting a subset from the original mesh:
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
    /*
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
         }
         // call the set_matrix_attribute function according to the top dimension
         switch (m.top_cell_dimension()) {
         case 2:
             wmtk::mesh_utils::set_matrix_attribute(
                 points_in,
                 "position",
                 wmtk::PrimitiveType::Vertex,
                 tri_ext_mesh);
             top_simplices = tri_ext_mesh.get_all(topType);
             return std::make_unique<wmtk::TriMesh>(tri_ext_mesh);
         case 3:
             wmtk::mesh_utils::set_matrix_attribute(
                 points_in,
                 "position",
                 wmtk::PrimitiveType::Vertex,
                 tet_ext_mesh);
             top_simplices = tet_ext_mesh.get_all(topType);
             return std::make_unique<wmtk::TetMesh>(tet_ext_mesh);
         default: throw std::runtime_error("Invalid top dimension in separating topology!");
         }
     }
     throw std::runtime_error("Should not reach here!");

     */
    // The following is an implementation of the algo listed in the comment above
    ///*
    // Step 1: register a new attribute to each vertex and init as -1
    wmtk::VectorXl submesh_index_vector;
    submesh_index_vector.resize(nb_vertex, 1);
    for (long i = 0; i < nb_vertex; ++i) submesh_index_vector.row(i) << -1;
    wmtk::MeshAttributeHandle<long> new_ver_index_handle = wmtk::mesh_utils::set_matrix_attribute(
        submesh_index_vector,
        "submesh_index",
        wmtk::PrimitiveType::Vertex,
        m);
    wmtk::Accessor<long> new_ver_index_acc = m.create_accessor(new_ver_index_handle);

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
    // std::cout << "# of cell inside = " << nb_cell_in << std::endl;
    // std::cout << "index of cells inside = " << tag_simplex_index << std::endl;
    assert(nb_cell_in <= top_simplex_count);

    // Step 3.1: for each tagged top dim simplex
    for (size_t index : tag_simplex_index) {
        // std::cout << "index = " << index;
        // std::cout << "tag = " << tag_acc.const_scalar_attribute(top_simplices.at(index))
        //           << std::endl;
        Simplex s = (top_simplex_dim == 2) ? Simplex::face(top_simplices[index])
                                           : Simplex::tetrahedron(top_simplices[index]);
        std::vector<wmtk::Tuple> vertices_in_simplex =
            wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        // for each vertex of the simplex
        for (wmtk::Tuple t : vertices_in_simplex) {
            // std::cout << "current vertex id = " << find_vertex_index(m, t) << std::endl;
            // std::cout << "before: attribute = " << new_ver_index_acc.scalar_attribute(t)
            //           << std::endl;
            // Step 3.2: if the vertex has attribute -1, then assign it a new index,
            if (new_ver_index_acc.scalar_attribute(t) == -1) {
                new_ver_index_acc.scalar_attribute(t) = nb_vertex_in;
                nb_vertex_in++;
            }
            // std::cout << "after: attribute = " << new_ver_index_acc.scalar_attribute(t)
            //           << std::endl;
        }
    }
    // std::cout << "nb_vertex_in = " << nb_vertex_in << std::endl;

    wmtk::TriMesh tri_ext_mesh;
    wmtk::RowVectors3l tri_exts;
    wmtk::TetMesh tet_ext_mesh;
    wmtk::RowVectors4l tet_exts;
    if (top_simplex_dim == 2) {
        tri_exts.resize(nb_cell_in, 3);
        for (int i = 0; i < nb_cell_in; ++i) {
            if (tag_acc.const_scalar_attribute(top_simplices.at(tag_simplex_index[i])) == 0)
                continue;
            Simplex s = Simplex::face(top_simplices[tag_simplex_index[i]]);
            std::vector<wmtk::Tuple> list =
                wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            std::vector<long> data(3, -1);
            for (int index = 0; index < 3; ++index) {
                data[index] = new_ver_index_acc.scalar_attribute(list[index]);
                // std::cout << "data[" << index << "] = " << data[index] << std::endl;
            }
            tri_exts.row(i) << data[0], data[1], data[2];
        }
        // std::cout << "tri_exts = " << tri_exts << std::endl;
        tri_ext_mesh.initialize(tri_exts);
        assert(tri_ext_mesh.get_all(topType).size() == nb_cell_in);
        if (!pos) return std::make_unique<wmtk::TriMesh>(tri_ext_mesh);
    } else if (top_simplex_dim == 3) {
        tet_exts.resize(nb_cell_in, 4);
        for (size_t i = 0; i < nb_cell_in; ++i) {
            if (tag_acc.const_scalar_attribute(top_simplices.at(tag_simplex_index[i])) == 0)
                continue;
            Simplex s = Simplex::tetrahedron(top_simplices[tag_simplex_index[i]]);
            std::vector<wmtk::Tuple> list =
                wmtk::simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
            std::vector<long> data(4, -1);
            for (int index = 0; index < 4; ++index)
                data[index] = new_ver_index_acc.scalar_attribute(list[index]);
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
        for (int i = 0; i < nb_vertex; ++i) {
            if (new_ver_index_acc.scalar_attribute(vertices[i]) != -1) {
                points_in.row(new_ver_index_acc.scalar_attribute(vertices[i])) =
                    pos_acc.const_vector_attribute(vertices[i]);
            }
        }
        // call the set_matrix_attribute function according to the top dimension
        switch (m.top_cell_dimension()) {
        case 2:
            wmtk::mesh_utils::set_matrix_attribute(
                points_in,
                "position",
                wmtk::PrimitiveType::Vertex,
                tri_ext_mesh);
            top_simplices = tri_ext_mesh.get_all(topType);
            return std::make_unique<wmtk::TriMesh>(tri_ext_mesh);
        case 3:
            wmtk::mesh_utils::set_matrix_attribute(
                points_in,
                "position",
                wmtk::PrimitiveType::Vertex,
                tet_ext_mesh);
            top_simplices = tet_ext_mesh.get_all(topType);
            return std::make_unique<wmtk::TetMesh>(tet_ext_mesh);
        default: throw std::runtime_error("Invalid top dimension in separating topology!");
        }
    }
    throw std::runtime_error("Should not reach here!");
    //*/
}
} // namespace wmtk::components::internal