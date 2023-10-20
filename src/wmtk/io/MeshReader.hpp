#pragma once

#include <filesystem>

namespace wmtk {

class Mesh;

class MeshReader
{
public:
    static void read(const std::filesystem::path& filename, Mesh& mesh);

    virtual ~MeshReader() {}

protected:
    virtual void read_aux(const std::filesystem::path& filename, Mesh& mesh) = 0;
};
} // namespace wmtk
