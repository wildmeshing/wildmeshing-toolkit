#pragma once
#include <map>
#include <string>


namespace wmtk::utils {
class Hashable
{
public:
    Hashable();
    Hashable(const Hashable&);
    Hashable(Hashable&&);
    Hashable& operator=(const Hashable&);
    Hashable& operator=(Hashable&&);
    virtual ~Hashable();
    virtual std::size_t hash() const;
    virtual std::map<std::string, size_t> child_hashes() const;
};

} // namespace wmtk::utils
