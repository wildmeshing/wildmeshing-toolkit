#pragma once

#include "MeshWriter.hpp"

#include <Eigen/Core>
#include <filesystem>

namespace paraviewo {
class HDF5VTUWriter;
}

namespace wmtk {
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
            const std::string& elements_name,
            const bool enabled);

        void write(const std::string& name, const long stride, const std::vector<double>& val);

        Eigen::MatrixXd& vertices() { return m_vertices; }

    private:
        std::string m_vertices_name;
        std::string m_elements_name;

        std::filesystem::path m_filename;

        bool m_enabled;

        std::shared_ptr<paraviewo::HDF5VTUWriter> m_paraview_file;

        Eigen::MatrixXd m_vertices;
        Eigen::MatrixXi m_elements;
    };

public:
    ParaviewWriter(
        const std::filesystem::path& filename,
        const std::string& vertices_name,
        bool write_points = true,
        bool write_edges = true,
        bool write_faces = true,
        bool write_tetrahedra = true);

    bool write(const int dim) override { return dim == 0 || m_enabled[dim]; }

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<char>& val) override;

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<long>& val) override;

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<double>& val) override;

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<Rational>& val) override;

private:
    std::array<ParaviewInternalWriter, 4> m_writers;
    std::array<bool, 4> m_enabled;
    std::string m_vertices_name;

    void write_internal(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<double>& val);
};

} // namespace wmtk