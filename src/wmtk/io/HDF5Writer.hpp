#pragma once

#include "MeshWriter.hpp"

#include <filesystem>

namespace h5pp {
class File;
}

namespace wmtk {
class HDF5Writer : public MeshWriter
{
public:
    HDF5Writer(const std::filesystem::path& filename);

    void write_top_simplex_type(const PrimitiveType type) override;
    void write_absolute_id(const std::vector<int64_t>& id) override;

    bool write(const int) override { return true; }

    void write_capacities(const std::vector<int64_t>& capacities) override;

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

    template <typename T>
    void write_attribute_names(int dim, const std::vector<std::string>& names);
private:
    std::shared_ptr<h5pp::File> m_hdf5_file;
    std::string m_name;

    std::string dataset_path() const;

    template <typename Data, typename T>
    void write_internal(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const Data& val,
        const T& default_val);

};

} // namespace wmtk
