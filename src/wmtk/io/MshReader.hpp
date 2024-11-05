#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <wmtk/Mesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>

#include <mshio/mshio.h>

namespace wmtk::io {

class MshReader
{
public:
    MshReader();
    ~MshReader();
    std::shared_ptr<Mesh> read(
        const std::filesystem::path& filename,
        const bool ignore_z_if_zero,
        const std::vector<std::string>& extra_facet_attributes = {});
    std::shared_ptr<Mesh> read(
        const std::filesystem::path& filename,
        const int64_t embedded_dimension,
        const std::vector<std::vector<std::string>>& extra_attributes);
    std::shared_ptr<Mesh> read(
        const std::filesystem::path& filename,
        const int64_t embedded_dimension = -1);

private:
    const mshio::NodeBlock* get_vertex_block(int DIM) const;

    const mshio::ElementBlock* get_simplex_element_block(int DIM) const;

    size_t get_num_vertices(int DIM) const;

    size_t get_num_simplex_elements(int DIM) const;

    template <int DIM>
    void extract_vertices();

    template <int DIM>
    void extract_simplex_elements();


    template <int DIM>
    void extract();


    template <int DIM>
    void validate();

    int get_mesh_dimension() const;
    int get_embedded_dimension() const;

    std::shared_ptr<Mesh> generate(
        const std::optional<std::vector<std::vector<std::string>>>& extra_attributes = {});

    template <int DIM>
    auto generateT() -> std::shared_ptr<wmtk::utils::mesh_type_from_dimension_t<DIM>>;

    template <int DIM>
    auto construct() -> std::shared_ptr<wmtk::utils::mesh_type_from_dimension_t<DIM>>;


    // std::vector<std::string> get_vertex_attribute_names(int DIM) const;

    // std::vector<std::string> get_element_attribute_names(int DIM) const;

    // void extract_vertex_attribute(
    //     std::shared_ptr<wmtk::TetMesh> m,
    //     const std::string& attr_name,
    //     int DIM);

    void extract_element_attribute(wmtk::Mesh& m, const std::string& attr_name, PrimitiveType pt);

    // template <int DIM, typename Fn>
    // bool extract_element_attribute(const std::string& attr_name, int DIM);


    // template <int DIM, typename Fn>
    // void extract_element_attribute(const std::string& attr_name, Fn&& set_attr);


private:
    mshio::MshSpec m_spec;
    int64_t m_embedded_dimension;


    Eigen::MatrixXd V;
    MatrixXl S;

    static const int64_t AUTO_EMBEDDED_DIMENSION = -2;
};

} // namespace wmtk::io
namespace wmtk {
using MshReader = io::MshReader;
}
