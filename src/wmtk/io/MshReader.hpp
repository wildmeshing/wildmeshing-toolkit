#pragma once

#include <Eigen/Core>
#include "MeshReader.hpp"

#include <mshio/mshio.h>

namespace wmtk {

class MshReader
{
public:
    std::shared_ptr<Mesh> read(const std::filesystem::path& filename, const bool ignore_z = false);

private:
    void set_vertex(size_t i, double x, double y, double z)
    {
        if (ignore_z)
            V.row(i) << x, y;
        else
            V.row(i) << x, y, z;
    }

    void set_edge(size_t i, int i0, int i1) { S.row(i) << i0, i1; }
    void set_face(size_t i, int i0, int i1, int i2)
    {
        assert(i0 >= 0 && i1 >= 0 && i2 >= 0);
        S.row(i) << i0, i1, i2;
    }
    void set_tet(size_t i, int i0, int i1, int i2, int i3) { S.row(i) << i0, i1, i2, i3; }


    inline size_t get_num_edge_vertices() const { return get_num_vertices<1>(); }
    inline size_t get_num_face_vertices() const { return get_num_vertices<2>(); }
    inline size_t get_num_tet_vertices() const { return get_num_vertices<3>(); }

    inline size_t get_num_edges() const { return get_num_simplex_elements<1>(); }
    inline size_t get_num_faces() const { return get_num_simplex_elements<2>(); }
    inline size_t get_num_tets() const { return get_num_simplex_elements<3>(); }

    void extract_tet_vertices();
    void extract_face_vertices();
    void extract_edge_vertices();

    void extract_edges();
    void extract_faces();
    void extract_tets();


    // set_attr

    // std::vector<std::string> get_edge_vertex_attribute_names() const;
    // std::vector<std::string> get_face_vertex_attribute_names() const;
    // std::vector<std::string> get_tet_vertex_attribute_names() const;
    // std::vector<std::string> get_edge_attribute_names() const;
    // std::vector<std::string> get_face_attribute_names() const;
    // std::vector<std::string> get_tet_attribute_names() const;

    // void extract_edge_vertex_attribute(const std::string& attr_name, Fn&& set_attr);
    // void extract_face_vertex_attribute(const std::string& attr_name, Fn&& set_attr);
    // void extract_tet_vertex_attribute(const std::string& attr_name, Fn&& set_attr);
    // void extract_edge_attribute(const std::string& attr_name, Fn&& set_attr);
    // void extract_face_attribute(const std::string& attr_name, Fn&& set_attr);
    // void extract_tet_attribute(const std::string& attr_name, Fn&& set_attr);

    template <int DIM>
    const mshio::NodeBlock* get_vertex_block() const;

    template <int DIM>
    const mshio::ElementBlock* get_simplex_element_block() const;

    template <int DIM>
    size_t get_num_vertices() const;

    template <int DIM>
    size_t get_num_simplex_elements() const;

    template <int DIM>
    void extract_vertices();

    template <int DIM>
    void extract_simplex_elements();


    // template <int DIM>
    // std::vector<std::string> get_vertex_attribute_names() const;

    // template <int DIM>
    // std::vector<std::string> get_element_attribute_names() const;

    // template <int DIM, typename Fn>
    // void extract_vertex_attribute(const std::string& attr_name, Fn&& set_attr)

    // template <int DIM, typename Fn>
    // void extract_element_attribute(const std::string& attr_name, Fn&& set_attr)

private:
    mshio::MshSpec m_spec;
    bool ignore_z;


    Eigen::MatrixXd V;
    Eigen::Matrix<long, -1, -1> S;
};

} // namespace wmtk
