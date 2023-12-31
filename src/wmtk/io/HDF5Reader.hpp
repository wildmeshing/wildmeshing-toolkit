#pragma once

#include <wmtk/Primitive.hpp>

#include <filesystem>
#include <vector>

namespace wmtk {

class Mesh;

class HDF5Reader
{
public:
    HDF5Reader();

    std::shared_ptr<Mesh> read(const std::filesystem::path& filename);

private:
    template <typename T>
    void set_attribute(
        const std::string& name,
        PrimitiveType pt,
        int64_t stride,
        const std::vector<T>& v,
        Mesh& mesh);
};
} // namespace wmtk
