#pragma once

#include "MeshReader.hpp"

#include <wmtk/Primitive.hpp>

#include <filesystem>
#include <vector>

namespace wmtk {

class Mesh;

class HDF5Reader : public MeshReader
{
public:
    HDF5Reader();

protected:
    void read_aux(const std::filesystem::path& filename, Mesh& mesh) override;

private:
    template <typename T>
    void set_attribute(
        const std::string& name,
        PrimitiveType pt,
        long stride,
        const std::vector<T>& v,
        Mesh& mesh);
};
} // namespace wmtk
