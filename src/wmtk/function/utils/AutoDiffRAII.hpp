#pragma once
#include <cstddef>


namespace wmtk::function::utils {


class AutoDiffRAII
{
public:
    AutoDiffRAII(size_t size);
    ~AutoDiffRAII();

private:
    size_t m_previous_variable_count;
};
} // namespace wmtk::function
