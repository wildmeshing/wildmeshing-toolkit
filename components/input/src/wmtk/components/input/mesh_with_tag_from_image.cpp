#include "mesh_with_tag_from_image.hpp"

#include <filesystem>
#include <wmtk/utils/mesh_utils.hpp>

// #define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace wmtk::components::input {

Eigen::Matrix<int64_t, -1, -1> tags_from_image(const std::filesystem::path& file)
{
    int width, height, channels;
    unsigned char* img = stbi_load(file.string().c_str(), &width, &height, &channels, 0);
    if (img == NULL) {
        throw std::runtime_error("Error in loading the image");
    }
    const size_t img_size = width * height * channels;

    Eigen::Matrix<int64_t, -1, -1> pixel_matrix;
    pixel_matrix.resize(height, width);

    for (size_t i = 0; i < height; ++i) {
        for (size_t j = 0; j < width; ++j) {
            const int pixel = *(img + ((width * i) + j) * channels);
            pixel_matrix(i, j) = pixel;
        }
    }

    stbi_image_free(img);

    return pixel_matrix;
}

std::shared_ptr<wmtk::TriMesh> mesh_with_tag_from_image(
    const std::filesystem::path& file,
    const std::string& tag_name)
{
    std::shared_ptr<wmtk::TriMesh> m = std::make_shared<wmtk::TriMesh>();

    Eigen::Matrix<int64_t, -1, -1> img = tags_from_image(file);

    std::cout << "img:\n" << img << std::endl;

    auto lex_index_v = [&img](size_t i, size_t j) { return i * (img.cols() + 1) + j; };
    auto lex_index_f = [&img](size_t i, size_t j) { return i * img.cols() + j; };

    // vertices
    Eigen::MatrixXd V;
    V.resize((img.rows() + 1) * (img.cols() + 1), 2);
    for (size_t i = 0; i < img.rows() + 1; ++i) {
        for (size_t j = 0; j < img.cols() + 1; ++j) {
            V.row(lex_index_v(i, j)) = Eigen::Matrix<double, 2, 1>(j, img.rows() - i);
        }
    }

    // triangles
    RowVectors3l F;
    F.resize(img.rows() * img.cols() * 2, 3);
    RowVectors<int64_t, 1> tags;
    tags.resize(F.rows(), 1);
    for (size_t i = 0; i < img.rows(); ++i) {
        for (size_t j = 0; j < img.cols(); ++j) {
            const int v0 = lex_index_v(i, j);
            const int v1 = lex_index_v(i, j + 1);
            const int v2 = lex_index_v(i + 1, j);
            const int v3 = lex_index_v(i + 1, j + 1);
            F.row(2 * lex_index_f(i, j) + 0) = Eigen::Matrix<int64_t, 3, 1>(v0, v2, v1);
            F.row(2 * lex_index_f(i, j) + 1) = Eigen::Matrix<int64_t, 3, 1>(v2, v3, v1);
            tags(2 * lex_index_f(i, j) + 0) = img(i, j);
            tags(2 * lex_index_f(i, j) + 1) = img(i, j);
        }
    }

    m->initialize(F);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *m);
    mesh_utils::set_matrix_attribute(tags, tag_name, PrimitiveType::Triangle, *m);

    return m;
}

} // namespace wmtk::components::input
