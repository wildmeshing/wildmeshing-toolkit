
#pragma once
#include <spdlog/logger.h>
#include <filesystem>
namespace wmtk {


std::shared_ptr<spdlog::logger> make_json_file_logger(
    const std::string& name,
    const std::filesystem::path& path,
    bool messages_are_json = false,
    bool also_output_stdout = true);
}
