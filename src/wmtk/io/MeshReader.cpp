#include "MeshReader.hpp"

#include <wmtk/Mesh.hpp>

#include "HDF5Reader.hpp"

#include <memory>

namespace wmtk {

void MeshReader::read(const std::filesystem::path& filename, Mesh& mesh)
{
    std::unique_ptr<MeshReader> reader = nullptr;
    const auto extension = filename.extension().string();
    if (extension == ".hdf5") {
        reader = std::make_unique<HDF5Reader>();
    }

    if (!reader)
        throw std::runtime_error(extension + " not supported");
    else
        reader->read_aux(filename, mesh);
}

} // namespace wmtk