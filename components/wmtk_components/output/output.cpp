#include "output.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include "internal/OutputOptions.hpp"

namespace wmtk {
namespace components {
void output(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    OutputOptions options = j.get<OutputOptions>();

    switch (options.cell_dimension) {
    case 2: {
        TriMesh mesh;
        {
            const std::filesystem::path& file = files[options.input];
            MeshReader reader(file);
            reader.read(mesh);
        }

        if (options.file.extension().empty()) {
            ParaviewWriter writer(options.file, "position", mesh, true, true, true, false);
            mesh.serialize(writer);
        } else if (options.file.extension() == ".off" || options.file.extension() == ".obj") {
            throw "not implemented yet";
        } else {
            throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
        }
        break;
    }
    case 3: {
        TetMesh mesh;
        {
            const std::filesystem::path& file = files[options.input];
            MeshReader reader(file);
            reader.read(mesh);
        }

        if (options.file.extension().empty()) {
            ParaviewWriter writer(options.file, "position", mesh, true, true, true, true);
            mesh.serialize(writer);
        } else {
            throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
        }
        break;
    }
    default: {
        throw "output component cannot process the given cell dimension";
        break;
    }
    }
}
} // namespace components
} // namespace wmtk