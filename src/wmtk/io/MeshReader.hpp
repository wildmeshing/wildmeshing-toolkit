#pragma once

#include <wmtk/Primitive.hpp>

#include <filesystem>
#include <vector>

namespace wmtk {

class Mesh;

class MeshReader
{
public:
    MeshReader(const std::filesystem::path& filename);

    void read(Mesh& mesh);

private:
    const std::filesystem::path& m_filename;

    template <typename T>
    void set_attribute(
        const std::string& name,
        PrimitiveType pt,
        long stride,
        const std::vector<T>& v,
        Mesh& mesh);
};
} // namespace wmtk