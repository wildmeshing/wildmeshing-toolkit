#pragma once

#include <filesystem>
#include <memory>

namespace wmtk {

class Mesh;

class MeshReader
{
public:
    static std::shared_ptr<Mesh> read(const std::filesystem::path& filename);

    virtual ~MeshReader() {}

protected:
    virtual std::shared_ptr<Mesh> read_aux(const std::filesystem::path& filename) = 0;
};
} // namespace wmtk
