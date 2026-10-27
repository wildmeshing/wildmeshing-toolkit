#pragma once

#include <mshio/mshio.h>
#include <filesystem>
#include <wmtk/MeshData.hpp>

namespace wmtk::io {
MeshData read_mesh(const std::filesystem::path& path);
void write_mesh(const std::filesystem::path& path, const MeshData& mesh);

// Shared with existing MSH adapters; these functions do not access the filesystem.
MeshData from_msh(const mshio::MshSpec& spec);
mshio::MshSpec to_msh(const MeshData& mesh);
} // namespace wmtk::io
