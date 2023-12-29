#include "load_image_exr.hpp"

#define TINYEXR_USE_STB_ZLIB 1
#define TINYEXR_USE_MINIZ 0
#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>
#include <cassert>
#include <wmtk/utils/Logger.hpp>
using namespace wmtk;
auto load_image_exr_red_channel(const std::filesystem::path& path)
    -> std::tuple<size_t, size_t, std::vector<float>>
{
    using namespace wmtk;
    wmtk::logger().debug("[load_image_exr_red_channel] start \"{}\"", path.string());
    assert(std::filesystem::exists(path));
    const std::string filename_ = path.string();
    const char* filename = filename_.c_str();

    const auto exr_version = [&filename, &path]() -> EXRVersion { // parse version
        EXRVersion exr_version_;

        const auto ret = ParseEXRVersionFromFile(&exr_version_, filename);
        if (ret != TINYEXR_SUCCESS) {
            wmtk::logger().error("failed LoadImageEXR \"{}\" \"version error\"", path.string());
            throw std::runtime_error("LoadImageEXRError");
        }

        if (exr_version_.multipart || exr_version_.non_image) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"multipart or non image\"",
                path.string());
            throw std::runtime_error("LoadImageEXRError");
        }

        return exr_version_;
    }();

    auto exr_header_data =
        [&filename, &path, &exr_version]() -> std::tuple<EXRHeader, int> { // parse header
        EXRHeader exr_header_;
        InitEXRHeader(&exr_header_);

        [[maybe_unused]] const char* err = nullptr;
        const auto ret = ParseEXRHeaderFromFile(&exr_header_, &exr_version, filename, &err);
        if (ret != TINYEXR_SUCCESS) {
            wmtk::logger().error("failed LoadImageEXR \"{}\" \"header error\"", path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        // sanity check, only support all channels are the same type
        for (int i = 0; i < exr_header_.num_channels; i++) {
            if (exr_header_.pixel_types[i] != exr_header_.pixel_types[0] ||
                exr_header_.requested_pixel_types[i] != exr_header_.pixel_types[i]) {
                wmtk::logger().error(
                    "failed LoadImageEXR \"{}\" \"inconsistent pixel_types\"",
                    path.string());
                FreeEXRHeader(&exr_header_);
                throw std::runtime_error("LoadImageEXRError");
            }
        }

        // read HALF channel as FLOAT.
        for (int i = 0; i < exr_header_.num_channels; i++) {
            if (exr_header_.pixel_types[i] == TINYEXR_PIXELTYPE_HALF) {
                exr_header_.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
            }
        }

        // only FLOAT are supported
        if (exr_header_.requested_pixel_types[0] != TINYEXR_PIXELTYPE_FLOAT) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"only float exr are supported\"",
                path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        // only non tiled image are supported
        if (exr_header_.tiled) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"only non tiled exr are supported\"",
                path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }


        int index_red_ = -1;
        for (int i = 0; i < exr_header_.num_channels; i++) {
            if (strcmp(exr_header_.channels[i].name, "R") == 0) index_red_ = i;
        }
        if (index_red_ < 0) {
            wmtk::logger().warn("Could not find R channel. Looking for Y channel instead.");
            for (int i = 0; i < exr_header_.num_channels; i++) {
                if (strcmp(exr_header_.channels[i].name, "Y") == 0) index_red_ = i;
            }
        }

        if (index_red_ < 0) {
            std::vector<std::string> channels;
            for (int i = 0; i < exr_header_.num_channels; i++) {
                channels.push_back(exr_header_.channels[i].name);
            }
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" can't find all expected channels: [{}]",
                path.string(),
                fmt::join(channels, ","));
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        return {exr_header_, index_red_};
    }();
    auto& exr_header = std::get<0>(exr_header_data);
    const auto& index_data = std::get<1>(exr_header_data);

    auto exr_image = [&filename, &path, &exr_header]() -> EXRImage {
        EXRImage exr_image_;
        InitEXRImage(&exr_image_);

        [[maybe_unused]] const char* err = nullptr;
        const auto ret = LoadEXRImageFromFile(&exr_image_, &exr_header, filename, &err);
        if (ret != TINYEXR_SUCCESS) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"failed to load image data\"",
                path.string());
            FreeEXRHeader(&exr_header);
            FreeEXRImage(&exr_image_);
            throw std::runtime_error("LoadImageEXRError");
        }

        return exr_image_;
    }();

    wmtk::logger().debug(
        "[load_image_exr_red_channel] num_channels {} tiled {}",
        exr_header.num_channels,
        exr_header.tiled);
    wmtk::logger().debug("[load_image_exr_red_channel] index_data {}", index_data);
    assert(index_data >= 0);
    assert(!exr_header.tiled);

    std::vector<float> data_r;
    data_r.reserve(static_cast<size_t>(exr_image.width) * static_cast<size_t>(exr_image.height));

    const auto images = reinterpret_cast<float**>(exr_image.images);
    for (int i = 0; i < exr_image.width * exr_image.height; i++)
        data_r.emplace_back(images[index_data][i]);

    FreeEXRHeader(&exr_header);
    FreeEXRImage(&exr_image);

    wmtk::logger().debug("[load_image_exr_red_channel] done \"{}\"", path.string());

    return {
        static_cast<size_t>(exr_image.width),
        static_cast<size_t>(exr_image.height),
        std::move(data_r),
    };
}

auto load_image_exr_split_3channels(const std::filesystem::path& path) -> std::
    tuple<size_t, size_t, int, int, int, std::vector<float>, std::vector<float>, std::vector<float>>
{
    using namespace wmtk;
    wmtk::logger().debug("[load_image_exr_red_channel] start \"{}\"", path.string());
    assert(std::filesystem::exists(path));
    const std::string filename_ = path.string();
    const char* filename = filename_.c_str();

    const auto exr_version = [&filename, &path]() -> EXRVersion { // parse version
        EXRVersion exr_version_;

        const auto ret = ParseEXRVersionFromFile(&exr_version_, filename);
        if (ret != TINYEXR_SUCCESS) {
            wmtk::logger().error("failed LoadImageEXR \"{}\" \"version error\"", path.string());
            throw std::runtime_error("LoadImageEXRError");
        }

        if (exr_version_.multipart || exr_version_.non_image) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"multipart or non image\"",
                path.string());
            throw std::runtime_error("LoadImageEXRError");
        }

        return exr_version_;
    }();

    auto exr_header_data =
        [&filename, &path, &exr_version]() -> std::tuple<EXRHeader, int, int, int> { // parse header
        EXRHeader exr_header_;
        InitEXRHeader(&exr_header_);

        [[maybe_unused]] const char* err = nullptr;
        const auto ret = ParseEXRHeaderFromFile(&exr_header_, &exr_version, filename, &err);
        if (ret != TINYEXR_SUCCESS) {
            wmtk::logger().error("failed LoadImageEXR \"{}\" \"header error\"", path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        // sanity check, only support all channels are the same type
        for (int i = 0; i < exr_header_.num_channels; i++) {
            if (exr_header_.pixel_types[i] != exr_header_.pixel_types[0] ||
                exr_header_.requested_pixel_types[i] != exr_header_.pixel_types[i]) {
                wmtk::logger().error(
                    "failed LoadImageEXR \"{}\" \"inconsistent pixel_types\"",
                    path.string());
                FreeEXRHeader(&exr_header_);
                throw std::runtime_error("LoadImageEXRError");
            }
        }

        // read HALF channel as FLOAT.
        for (int i = 0; i < exr_header_.num_channels; i++) {
            if (exr_header_.pixel_types[i] == TINYEXR_PIXELTYPE_HALF) {
                exr_header_.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
            }
        }

        // only FLOAT are supported
        if (exr_header_.requested_pixel_types[0] != TINYEXR_PIXELTYPE_FLOAT) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"only float exr are supported\"",
                path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        // only non tiled image are supported
        if (exr_header_.tiled) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"only non tiled exr are supported\"",
                path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }


        int index_red_ = -1;
        int index_green_ = -1;
        int index_blue_ = -1;
        if (exr_header_.num_channels == 1) {
            wmtk::logger().warn("Treat grayscale image as RGB: {}", path.string());
            index_red_ = 0;
            index_green_ = 0;
            index_blue_ = 0;
        } else {
            for (int i = 0; i < exr_header_.num_channels; i++) {
                if (strcmp(exr_header_.channels[i].name, "R") == 0) index_red_ = i;
                if (strcmp(exr_header_.channels[i].name, "G") == 0) index_green_ = i;
                if (strcmp(exr_header_.channels[i].name, "B") == 0) index_blue_ = i;
            }
        }

        if (index_red_ < 0) {
            std::vector<std::string> channels;
            for (int i = 0; i < exr_header_.num_channels; i++) {
                channels.push_back(exr_header_.channels[i].name);
            }
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" can't find all 3 expected channels: [{}]",
                path.string(),
                fmt::join(channels, ","));
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        return {exr_header_, index_red_, index_green_, index_blue_};
    }();
    auto& exr_header = std::get<0>(exr_header_data);
    const auto& index_red = std::get<1>(exr_header_data);
    const auto& index_green = std::get<2>(exr_header_data);
    const auto& index_blue = std::get<3>(exr_header_data);

    auto exr_image = [&filename, &path, &exr_header]() -> EXRImage {
        EXRImage exr_image_;
        InitEXRImage(&exr_image_);

        [[maybe_unused]] const char* err = nullptr;
        const auto ret = LoadEXRImageFromFile(&exr_image_, &exr_header, filename, &err);
        if (ret != TINYEXR_SUCCESS) {
            wmtk::logger().error(
                "failed LoadImageEXR \"{}\" \"failed to load image data\"",
                path.string());
            FreeEXRHeader(&exr_header);
            FreeEXRImage(&exr_image_);
            throw std::runtime_error("LoadImageEXRError");
        }

        return exr_image_;
    }();

    wmtk::logger().debug(
        "[load_image_exr_3channels] num_channels {} tiled {}",
        exr_header.num_channels,
        exr_header.tiled);
    wmtk::logger().debug("[load_image_exr_3channels] index_red {}", index_red);
    wmtk::logger().debug("[load_image_exr_3channels] index_green {}", index_green);
    wmtk::logger().debug("[load_image_exr_3channels] index_blue {}", index_blue);
    assert(index_red >= 0);
    assert(index_green >= 0);
    assert(index_blue >= 0);
    assert(!exr_header.tiled);

    std::vector<float> data_r;
    std::vector<float> data_g;
    std::vector<float> data_b;
    data_r.reserve(static_cast<size_t>(exr_image.width) * static_cast<size_t>(exr_image.height));
    data_g.reserve(static_cast<size_t>(exr_image.width) * static_cast<size_t>(exr_image.height));
    data_b.reserve(static_cast<size_t>(exr_image.width) * static_cast<size_t>(exr_image.height));

    const auto images = reinterpret_cast<float**>(exr_image.images);
    for (int i = 0; i < exr_image.width * exr_image.height; i++) {
        data_r.emplace_back(images[index_red][i]);
        data_g.emplace_back(images[index_green][i]);
        data_b.emplace_back(images[index_blue][i]);
    }
    FreeEXRHeader(&exr_header);
    FreeEXRImage(&exr_image);

    wmtk::logger().debug("[load_image_exr_3channels] done \"{}\"", path.string());

    return {
        static_cast<size_t>(exr_image.width),
        static_cast<size_t>(exr_image.height),
        std::move(index_red),
        std::move(index_green),
        std::move(index_blue),
        std::move(data_r),
        std::move(data_g),
        std::move(data_b)};
}
