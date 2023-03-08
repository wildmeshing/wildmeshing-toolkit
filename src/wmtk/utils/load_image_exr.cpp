#include "load_image_exr.h"

#include <spdlog/spdlog.h>
#define TINYEXR_USE_MINIZ 0
#define TINYEXR_USE_STB_ZLIB 1
#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>
#include <cassert>
#include "Logger.hpp"

auto wmtk::load_image_exr_red_channel(const std::filesystem::path& path)
    -> std::tuple<size_t, size_t, std::vector<float>>
{
    using namespace wmtk;
    spdlog::trace("[load_image_exr_red_channel] start \"{}\"", path.string());
    assert(std::filesystem::exists(path));
    const std::string filename_ = path.string();
    const char* filename = filename_.c_str();

    const auto exr_version = [&filename, &path]() -> EXRVersion { // parse version
        EXRVersion exr_version_;

        const auto ret = ParseEXRVersionFromFile(&exr_version_, filename);
        if (ret != TINYEXR_SUCCESS) {
            spdlog::error("failed LoadImageEXR \"{}\" \"version error\"", path.string());
            throw std::runtime_error("LoadImageEXRError");
        }

        if (exr_version_.multipart || exr_version_.non_image) {
            spdlog::error("failed LoadImageEXR \"{}\" \"multipart or non image\"", path.string());
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
            spdlog::error("failed LoadImageEXR \"{}\" \"header error\"", path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        // sanity check, only support all channels are the same type
        for (int i = 0; i < exr_header_.num_channels; i++) {
            if (exr_header_.pixel_types[i] != exr_header_.pixel_types[0] ||
                exr_header_.requested_pixel_types[i] != exr_header_.pixel_types[i]) {
                spdlog::error(
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
            spdlog::error(
                "failed LoadImageEXR \"{}\" \"only float exr are supported\"",
                path.string());
            FreeEXRHeader(&exr_header_);
            throw std::runtime_error("LoadImageEXRError");
        }

        // only non tiled image are supported
        if (exr_header_.tiled) {
            spdlog::error(
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
            spdlog::warn("Could not find R channel. Looking for Y channel instead.");
            for (int i = 0; i < exr_header_.num_channels; i++) {
                if (strcmp(exr_header_.channels[i].name, "Y") == 0) index_red_ = i;
            }
        }

        if (index_red_ < 0) {
            std::vector<std::string> channels;
            for (int i = 0; i < exr_header_.num_channels; i++) {
                channels.push_back(exr_header_.channels[i].name);
            }
            spdlog::error(
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
            spdlog::error(
                "failed LoadImageEXR \"{}\" \"failed to load image data\"",
                path.string());
            FreeEXRHeader(&exr_header);
            FreeEXRImage(&exr_image_);
            throw std::runtime_error("LoadImageEXRError");
        }

        return exr_image_;
    }();

    wmtk::logger().info(
        "[load_image_exr] num_channels {} tiled {}",
        exr_header.num_channels,
        exr_header.tiled);
    wmtk::logger().info("[load_image_exr] index_data {}", index_data);
    assert(index_data >= 0);
    assert(!exr_header.tiled);

    std::vector<float> data;
    data.reserve(static_cast<size_t>(exr_image.width) * static_cast<size_t>(exr_image.height));

    const auto images = reinterpret_cast<float**>(exr_image.images);
    for (int i = 0; i < exr_image.width * exr_image.height; i++)
        data.emplace_back(images[index_data][i]);

    FreeEXRHeader(&exr_header);
    FreeEXRImage(&exr_image);

    wmtk::logger().info("[load_image_exr_red_channel] done \"{}\"", path.string());

    return {
        static_cast<size_t>(exr_image.width),
        static_cast<size_t>(exr_image.height),
        std::move(data),
    };
}
