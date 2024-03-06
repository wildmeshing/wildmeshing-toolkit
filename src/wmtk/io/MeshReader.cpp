#include "MeshReader.hpp"

// #include <wmtk/Mesh.hpp>

#include "HDF5Reader.hpp"

#include <memory>

namespace wmtk {

std::shared_ptr<Mesh> read_mesh(const std::filesystem::path& filename, const bool ignore_z)
{
    const auto extension = filename.extension().string();
    if (extension == ".hdf5") {
        HDF5Reader reader;
        return reader.read(filename);
    } else if (extension == ".msh") {
        MshReader reader;
        // std::shared_ptr<Mesh> ret = reader.read(filename, ignore_z);
        // std::vector<std::string> attribute_names = reader.get_element_attribute_names<3>();
        // for (const std::string& name : attribute_names) {
        //     std::cout << "Extracting attribute " << name << std::endl;
        // }
        // return ret;
        return reader.read(filename, ignore_z);
    }

    throw std::runtime_error(extension + " not supported");
}

} // namespace wmtk