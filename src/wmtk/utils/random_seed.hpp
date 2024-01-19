#pragma once

#include <random>

namespace wmtk::utils {

class RandomSeedSingleton
{
public:
    static RandomSeedSingleton& instance()
    {
        static RandomSeedSingleton _instance;
        return _instance;
    }

    inline void set_seed(uint64_t val) { m_gen.seed(val); }
    inline unsigned int get_seed() { return m_gen(); }

private:
    RandomSeedSingleton()
    {
        std::random_device rd;
        m_gen.seed(rd());
    }

    std::mt19937 m_gen;
};


inline unsigned int get_random_seed()
{
    return RandomSeedSingleton::instance().get_seed();
}

inline void set_random_seed(uint64_t val)
{
    RandomSeedSingleton::instance().set_seed(val);
}

} // namespace wmtk::utils
