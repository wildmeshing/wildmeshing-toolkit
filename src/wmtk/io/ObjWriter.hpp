#pragma once
#include <vector>

#include "MeshWriter.hpp"

#include <Eigen/Core>
#include <filesystem>
namespace wmtk {
class Mesh;
namespace io {
/** currently only support writing vertex positions for triangles */
class ObjWriter : public MeshWriter
{
private:
    class ObjInternalWriter
    {
    public:
        ObjInternalWriter() = default;
        ~ObjInternalWriter() = default;

        void init(
            const std::filesystem::path& filename,
            const std::string& vertices_name,
            const Eigen::MatrixXi& elements);

        void write(const std::string& name, const int64_t stride, const std::vector<double>& val);
        Eigen::MatrixXd& vertices() { return m_vertices; }

    private:
        std::string m_vertices_name;

        std::filesystem::path m_filename;

        Eigen::MatrixXd m_vertices;
        Eigen::MatrixXi m_elements;
    };

public:
    ~ObjWriter() {}
    ObjWriter();
    ObjWriter(
        const std::filesystem::path& filename,
        const std::string& vertices_name,
        const Mesh& mesh);
    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<double>& val,
        double default_val) override;
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
        const std::vector<Rational>& val,
        const Rational& default_val) override;
    bool write(const int dim) override;
    void write_top_simplex_type(const PrimitiveType type) override;

    void write_absolute_id(const std::vector<int64_t>& id) override;

    void write_capacities(const std::vector<int64_t>& capacities) override;

protected:
    ObjInternalWriter m_writer;
};
} // namespace io
} // namespace wmtk
