#pragma once

#include <filesystem>

namespace wmtk {
class MeshReader
{
public:
    MeshReader(const std::filesystem::path& filename);

    void read();
};
} // namespace wmtk