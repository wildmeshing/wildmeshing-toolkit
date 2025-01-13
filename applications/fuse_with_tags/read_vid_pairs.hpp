
#pragma once
#include <vector>
#include <array>

#include <nlohmann/json_fwd.hpp>
#include <filesystem>
std::pair<std::array<int64_t,2>, std::vector<std::array<int64_t,2>>> read_vid_pairs(const std::filesystem::path& path);
std::pair<std::array<int64_t,2>, std::vector<std::array<int64_t,2>>> read_vid_pairs(const nlohmann::json& js);
