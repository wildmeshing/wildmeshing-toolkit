#pragma once

#include <wmtk/Mesh.hpp>

#include <filesystem>


namespace wmtk::components {

/**
 * @brief Write the mesh to file.
 *
 * The mesh can be either outputted as .vtu or as .hdf5.
 *
 * .vtu: Do not add a file extension, if you want to write in .vtu format. This writes a file for
 * each dimension. For example, specifying the file "abc" will create "abc_vertices.vtu",
 * "abc_edges.vtu", and "abc_triangles.vtu", if the mesh is a triangle mesh.
 * Note that .vtu output is meant for visualization. We do not support reading from .vtu files!
 *
 * .hdf5: Write the entire mesh in .hdf5 file format. The file contains all attributes of the mesh
 * and can be loaded again using the input component.
 *
 * Other file formats are not supported at this moment.
 *
 * @param mesh The mesh that should be written to file.
 * @param file The desired output file(s).
 * @param position_attr_name The name of the position attribute. It must be of type double.
 *
 */
void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const std::string& position_attr_name = {});

/**
 * @brief Write the mesh to file.
 *
 * The mesh can be either outputted as .vtu or as .hdf5.
 *
 * .vtu: Do not add a file extension, if you want to write in .vtu format. This writes a file for
 * each dimension. For example, specifying the file "abc" will create "abc_vertices.vtu",
 * "abc_edges.vtu", and "abc_triangles.vtu", if the mesh is a triangle mesh.
 * Note that .vtu output is meant for visualization. We do not support reading from .vtu files!
 *
 * .hdf5: Write the entire mesh in .hdf5 file format. The file contains all attributes of the mesh
 * and can be loaded again using the input component.
 *
 * Other file formats are not supported at this moment.
 *
 * @param mesh The mesh that should be written to file.
 * @param file The desired output file(s).
 * @param position_attr. The position attribute. It must be of type double.
 *
 */
void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const attribute::MeshAttributeHandle& position_attr);


/**
 * @brief Write the mesh to file.
 *
 * The mesh can be either outputted as .hdf5.
 * .hdf5: Write the entire mesh in .hdf5 file format. The file contains all attributes of the mesh
 * and can be loaded again using the input component.
 *
 * @param mesh The mesh that should be written to file.
 * @param file The desired output file(s).
 */
void output_hdf5(const Mesh& mesh, const std::filesystem::path& file);
} // namespace wmtk::components
