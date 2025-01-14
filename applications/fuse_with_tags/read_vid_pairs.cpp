#include "read_vid_pairs.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
std::pair<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>> read_vid_pairs(
    const std::filesystem::path& path)
{
    std::ifstream ifs(path);
    nlohmann::json js;
    ifs >> js;
    return read_vid_pairs(js);
}
std::pair<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>> read_vid_pairs(
    const nlohmann::json& js)
{
    std::pair<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>> ret;
    auto& [inds, values] = ret;

    int64_t index = 0;
    for (const auto& [name, vals] : js.items()) {
        inds[index] = std::stoi(name);

        values.resize(vals.size());
        for (size_t j = 0; j < values.size(); ++j) {
            values[j][index] = vals[j];
        }
        index++;
    }

    return ret;
}

