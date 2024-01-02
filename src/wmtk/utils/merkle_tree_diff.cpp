#include "merkle_tree_diff.hpp"
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/utils/Rational.hpp>
#include "Hashable.hpp"
#include "MerkleTreeInteriorNode.hpp"

namespace wmtk::utils {
namespace {

template <typename T>
nlohmann::json attribute_detailed_data_diff(
    const wmtk::attribute::Attribute<T>& a_a,
    const wmtk::attribute::Attribute<T>& a_b)
{
    nlohmann::json data;
    if (a_a.dimension() != a_b.dimension()) {
        data["dimension"] = {a_a.dimension(), a_b.dimension()};
        return data;
    }
    const int64_t a_size = a_a.reserved_size();
    const int64_t b_size = a_b.reserved_size();

    if (a_size != b_size) {
        data["sizes"] = {a_size, b_size};
    }

    const int64_t size = std::max(a_size, b_size);

    for (int64_t index = 0; index < size; ++index) {
        auto write_value = [&](const auto& attr) -> std::vector<T> {
            if (index >= attr.reserved_size()) {
                return {};
            }
            std::vector<T> r(attr.dimension());
            auto v = attr.const_vector_attribute(index);
            std::copy(v.begin(), v.end(), r.begin());
            return r;
        };
        if constexpr (std::is_same_v<T, wmtk::Rational>) {
            // TODO:
        } else {
            nlohmann::json a_values = write_value(a_a);
            nlohmann::json b_values = write_value(a_b);
            if (a_values.size() == 0) {
                a_values = {};
            }
            if (b_values.size() == 0) {
                b_values = {};
            }
            if (a_values != b_values) {
                nlohmann::json& mydat = data["values"].emplace_back();
                mydat["index"] = index;
                mydat["values"] = {a_values, b_values};
            }
        }
    }

    return data;
}
nlohmann::json merkle_tree_diff(
    const MerkleTreeInteriorNode& a,
    const MerkleTreeInteriorNode& b,
    bool detailed = false)
{
    nlohmann::json diff;
    auto a_child_hashables = a.child_hashables();
    auto b_child_hashables = b.child_hashables();
    for (const auto& [name, a_hashable_ptr] : a_child_hashables) {
        if (auto it = b_child_hashables.find(name); it != b_child_hashables.end()) {
            const auto& b_hashable_ptr = std::get<1>(*it);
            auto cdiff_opt = merkle_tree_diff(*a_hashable_ptr, *b_hashable_ptr, detailed = true);
            if (!cdiff_opt.has_value()) {
                continue;
            }
            diff[name] = cdiff_opt.value();
        }
    }
    return diff;
}
} // namespace
std::optional<nlohmann::json> merkle_tree_diff(const Hashable& a, const Hashable& b, bool detailed)
{
    nlohmann::json diff;

    if (a.hash() == b.hash()) {
        return {};
    }
    auto a_child_hashes = a.child_hashes();
    auto b_child_hashes = b.child_hashes();

    // temporaries to store the left/right of something
    nlohmann::json a_value;
    nlohmann::json b_value;
    for (const auto& [name, a_hash] : a_child_hashes) {
        a_value = a_hash;
        if (b_child_hashes.find(name) != b_child_hashes.end()) {
            size_t b_hash = b_child_hashes[name];
            if (a_hash == b_hash) {
                continue;
            }
            b_value = b_hash;
        } else {
            b_value = nlohmann::json();
        }
        diff[name] = nlohmann::json::array({a_value, b_value});
    }

    a_value = nlohmann::json();
    for (const auto& [name, hash] : b_child_hashes) {
        b_value = hash;
        if (b_child_hashes.find(name) != b_child_hashes.end()) {
            // covered in previous loop
            continue;
        }
        diff[name] = nlohmann::json::array({a_value, b_value});
    }

    // takes in some pointer of the type being cast to (just pass in nullptr
    // this is just a hack to get around not having
    // [&]<typename T>() {
    // }
    // try_cast<T>
    // which is a cpp20 feature :(
    auto try_cast = [&](auto&& type) {
        using T = std::decay_t<decltype(*type)>;
        std::optional<std::array<const T*, 2>> ret;
        auto a_ptr = dynamic_cast<const T*>(&a), b_ptr = dynamic_cast<const T*>(&b);
        if (a_ptr != nullptr && b_ptr != nullptr) {
            ret = std::array<const T*, 2>{{a_ptr, b_ptr}};
        }
        return ret;
    };

    // pass in a ptr to the desired type
    if (auto merkle_pair_opt = try_cast((MerkleTreeInteriorNode*)(nullptr));
        merkle_pair_opt.has_value()) {
        const auto& m_pr = merkle_pair_opt.value();
        const auto& m_a = *m_pr[0];
        const auto& m_b = *m_pr[1];
        auto cdiff = merkle_tree_diff(m_a, m_b, detailed);

        for (const auto& kv : cdiff.items()) {
            diff[kv.key()] = kv.value();
        }
    }

    auto try_attr = [&](auto&& type) {
        using T = std::decay_t<decltype(type)>;

        if (auto attr_pair_opt = try_cast((attribute::Attribute<T>*)(nullptr));
            attr_pair_opt.has_value()) {
            const auto& a_pr = attr_pair_opt.value();
            const auto& a_a = *a_pr[0];
            const auto& a_b = *a_pr[1];

            if (detailed) {
                diff["details"] = attribute_detailed_data_diff(a_a, a_b);
            }
            // if (!cdiff.is_null()) {
            //     diff[name] = cdiff;
            // }
        }
    };
    try_attr(double());
    try_attr(int64_t());
    try_attr(char());
    // try_attr(Rational());
    return diff;
}
} // namespace wmtk::utils
