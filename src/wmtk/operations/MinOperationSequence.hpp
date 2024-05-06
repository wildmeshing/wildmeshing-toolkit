#pragma once

#include "OperationSequence.hpp"

#include <wmtk/Tuple.hpp>


namespace wmtk {
class Mesh;

namespace operations {


class MinOperationSequence : public OperationSequence
{
public:
    MinOperationSequence(
        Mesh& mesh,
        const std::vector<std::shared_ptr<Operation>>& operations = {});
    virtual ~MinOperationSequence();


    inline void set_value_function(
        const std::function<double(int64_t, const simplex::Simplex&)>& func)
    {
        m_value = func;
    }


protected:
    std::vector<simplex::Simplex> execute_operations(const simplex::Simplex& simplex) override;


private:
    std::function<double(int64_t, const simplex::Simplex&)> m_value = nullptr;
};

} // namespace operations
} // namespace wmtk
