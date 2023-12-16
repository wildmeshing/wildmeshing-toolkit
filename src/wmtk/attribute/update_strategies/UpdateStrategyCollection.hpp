#pragma once

namespace wmtk::attribute::update_strategies {

class UpdateStrategyCollection
{
    public:
    private:
        std::vector<std::shared_ptr<UpdateStrategy>> m_strategies;
};
} // namespace wmtk::attribute::strategies
