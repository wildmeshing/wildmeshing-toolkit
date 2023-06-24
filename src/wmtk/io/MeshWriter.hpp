#pragma once

#include <filesystem>

namespace h5pp {
class File;
}

namespace wmtk {
class MeshWriter
{
public:
    MeshWriter(const std::filesystem::path& filename);

    template <typename T>
    void
    write(const std::string& name, const long type, const long stride, const std::vector<T>& val);

private:
    std::shared_ptr<h5pp::File> m_hdf5_file;
};

} // namespace wmtk