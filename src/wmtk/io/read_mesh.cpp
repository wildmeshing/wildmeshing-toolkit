#include "read_mesh.hpp"
#include "HDF5Reader.hpp"
#include "MshReader.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {
namespace utils {
FileType guess_file_type_from_extension(const std::string_view& suffix)
{
    if (suffix == ".hdf5") {
        return FileType::HDF5;
    } else if (suffix == ".msh") {
        return FileType::Msh;
    } else {
        return FileType::Auto;
    }
}

FileType guess_file_type(const std::filesystem::path& filename)
{
    const auto extension = filename.extension().string();

    const FileType ret = guess_file_type_from_extension(extension);
    if (ret == FileType::Auto) {
        log_and_throw_error(
            "Automatic file type detection could not identify type of file {} with extension of {}",
            filename.string(),
            extension);
    }
    return ret;
}

} // namespace utils
std::shared_ptr<Mesh> read_mesh(const std::filesystem::path& filename, FileType file_type)
{
    if (file_type == FileType::Auto) {
        file_type = utils::guess_file_type(filename);
    }
    switch (file_type) {
    case FileType::HDF5: {
        HDF5Reader reader;
        return reader.read(filename);
    }
    case FileType::Msh: {
        MshReader reader;
        return reader.read(filename);
    }
    default:
    case FileType::Auto: {
        assert(false); // get_file_type should have caught failure to obtain right ifle type already
    }
    }
    return {};
}
std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const std::vector<std::vector<std::string>>& retrieved_attributes,
    FileType file_type)
{
    if (file_type == FileType::Auto) {
        file_type = utils::guess_file_type(filename);
    }
    switch (file_type) {
    case FileType::HDF5: {
        HDF5Reader reader;
        return reader.read(filename);
    }
    case FileType::Msh: {
        MshReader reader;
        return reader.read(filename, -1, retrieved_attributes);
    }
    default:
    case FileType::Auto: {
        assert(false); // get_file_type should have caught failure to obtain right ifle type already
    }
    }
    return {};
}

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const bool ignore_z,
    const std::vector<std::string>& tetrahedron_attributes,
    FileType file_type)
{
    if (file_type == FileType::Auto) {
        file_type = utils::guess_file_type(filename);
    }
    switch (file_type) {
    case FileType::HDF5: {
        HDF5Reader reader;
        return reader.read(filename);
    }
    case FileType::Msh: {
        MshReader reader;
        return reader.read(filename, ignore_z, tetrahedron_attributes);
    }
    default:
    case FileType::Auto: {
        assert(false); // get_file_type should have caught failure to obtain right ifle type already
    }
    }
    return {};
}
} // namespace wmtk::io
