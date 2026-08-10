#pragma once

#include <wmtk/Types.hpp>

#include <filesystem>

namespace wmtk::io {

/**
 * @brief Read an STL file, ASCII or binary, into vertex and face matrices.
 *
 * Derived from libigl's igl::readSTL (attribution and the list of changes are in the .cpp).
 * The reason for keeping a copy is that the original reports malformed input in ways a
 * caller cannot act on: it asserts where it should report -- which aborts a Debug build
 * inside the reader, uncatchably -- and it reads facet data before checking that the read
 * succeeded. This version validates first and throws an error naming the file.
 *
 * Normals are parsed for validation and then discarded; nothing in the toolkit uses them.
 *
 * @param path File to read.
 * @param V Output vertex positions, #V x 3. An STL stores each facet's three corners
 *          independently, with no sharing, so #V is always 3 * #F.
 * @param F Output face indices, #F x 3.
 *
 * Both matrices come back with three columns whatever the input, the empty mesh included,
 * so callers need not special-case the shape.
 *
 * @throws std::runtime_error (logged) if the file cannot be opened or is malformed.
 */
void read_stl(const std::filesystem::path& path, MatrixXd& V, MatrixXi& F);

} // namespace wmtk::io
