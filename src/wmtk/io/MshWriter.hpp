#pragma once

#include <mshio/mshio.h>
#include <filesystem>
#include <wmtk/Mesh.hpp>

namespace wmtk::io {

class MshWriter
{
public:
    MshWriter(const std::filesystem::path& filename);

    void write(const Mesh& mesh, const std::string& position_attribute_name);

private:
    std::filesystem::path m_name;

    mshio::MshSpec m_spec;
};

} // namespace wmtk::io