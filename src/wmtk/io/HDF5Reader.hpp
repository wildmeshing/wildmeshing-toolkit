#pragma once

#include <wmtk/Primitive.hpp>

#include <filesystem>
#include <vector>

namespace h5pp {
class File;
}

namespace wmtk {

class Mesh;

class HDF5Reader
{
public:
    HDF5Reader();

    std::shared_ptr<Mesh> read(const std::filesystem::path& filename);

private:
    std::shared_ptr<Mesh> read_mesh(h5pp::File& hdf5_file, const std::string& dataset);


    void read_attribute(
        h5pp::File& hdf5_file,
        Mesh& m,
        const std::string& dataset,
        const std::string& name);
    template <typename T>
    void set_attribute(
        const T& default_val,
        const std::string& name,
        PrimitiveType pt,
        int64_t stride,
        const std::vector<T>& v,
        Mesh& mesh);
};
} // namespace wmtk
