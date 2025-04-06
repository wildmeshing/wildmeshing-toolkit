#pragma once

#include <mshio/mshio.h>
#include <filesystem>
#include <wmtk/Mesh.hpp>

namespace wmtk::io {

class MshWriter
{
public:
    MshWriter() = delete;

    static void write(
        const std::filesystem::path& m_name,
        const Mesh& mesh,
        const std::string& position_attribute_name,
        const std::vector<std::string>& cell_attribute_names = {});

private:
};

} // namespace wmtk::io