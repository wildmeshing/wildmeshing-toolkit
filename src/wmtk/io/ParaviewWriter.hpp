#pragma once

#include "MeshWriter.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <functional>

namespace paraviewo {
class ParaviewWriter;
} // namespace paraviewo

namespace wmtk {
class Mesh;
namespace simplex {
class IdSimplex;
}

namespace io {
class ParaviewWriter : public MeshWriter
{
private:
    class ParaviewInternalWriter
    {
    public:
        ParaviewInternalWriter();
        ~ParaviewInternalWriter();

        void init(
            const std::filesystem::path& filename,
            const std::string& vertices_name,
            const Eigen::MatrixXi& elements,
            const bool enabled);

        void write(
            const std::string& name,
            const int64_t stride,
            const std::vector<double>& val,
            const bool is_cell_field);


        Eigen::MatrixXd& vertices() { return m_vertices; }

    private:
        std::string m_vertices_name;

        std::filesystem::path m_filename;

        bool m_enabled;

        std::shared_ptr<paraviewo::ParaviewWriter> m_paraview_file;

        Eigen::MatrixXd m_vertices;
        Eigen::MatrixXi m_elements;
    };

public:
    /**
     * @brief Write in VTU format.
     *
     * The writer generates one file for each simplex dimension and attaches all the attributes for
     * the corresponding simplex dimension. All higher dimensions also contain the simplex
     * attributes.
     *
     * The writer stores ALL simplices, even those that are invalid. This helps with debugging, as
     * the IDs in the VTU file correspond to those in the code. All invalid simplices will contain 0
     * vertex IDs, so in case one vertex looks strange in Paraview, that might be because of that.
     *
     * The filter function can be used to treat simplices as invalid ones. A simplex is treated as
     * invalid if the filter function returns false.
     */
    ParaviewWriter(
        const std::filesystem::path& filename,
        const std::string& vertices_name,
        const Mesh& mesh,
        bool write_points = true,
        bool write_edges = true,
        bool write_faces = true,
        bool write_tetrahedra = true,
        const std::function<bool(const simplex::IdSimplex&)>& filter = {});

    bool write(const int dim) override { return dim == 0 || m_enabled[dim]; }

    void write_top_simplex_type(const PrimitiveType) override {}
    void write_absolute_id(const std::vector<int64_t>& id) override
    {
        m_write = m_mm_level == 0;
        ++m_mm_level;
    }

    // paraview doesn't care about mesh capacities
    void write_capacities(const std::vector<int64_t>& capacities) override {}

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<char>& val,
        const char default_val) override;

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<int64_t>& val,
        const int64_t default_val) override;

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<double>& val,
        const double default_val) override;

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<Rational>& val,
        const Rational& default_val) override;


private:
    std::array<ParaviewInternalWriter, 4> m_writers;
    std::array<bool, 4> m_enabled;
    std::string m_vertices_name;
    bool m_write;

    void write_internal(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<double>& val);
};
} // namespace io

using ParaviewWriter = io::ParaviewWriter;

} // namespace wmtk
