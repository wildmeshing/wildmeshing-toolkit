#pragma once
#include <filesystem>
#include <memory>
#include <vector>

namespace wmtk {
class Mesh;
}
namespace wmtk::io {

enum class FileType { Auto, Msh, HDF5 };

namespace utils {
FileType guess_file_type(const std::string_view& file_name);
// expects things like ".msh" or ".hdf5", unknown results return auto
FileType guess_file_type_from_extension(const std::string_view& suffix);
} // namespace utils

std::shared_ptr<Mesh> read_mesh(const std::filesystem::path& filename, FileType file_type);

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const std::vector<std::vector<std::string>>& retrieved_attributes,
    FileType file_type = FileType::Auto);

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const bool ignore_z = false,
    const std::vector<std::string>& tetrahedron_attributes = {},
    FileType file_type = FileType::Auto);


} // namespace wmtk::io
