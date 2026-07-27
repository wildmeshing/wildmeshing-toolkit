#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/threading/enumerable_thread_specific.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <map>
#include <optional>
#include <unordered_map>
#include <vector>

namespace wmtk {
/**
 * @brief serving as buffers for attributes data that can be modified by operations
 *
 */
class AbstractAttributeContainer
{
public:
    virtual ~AbstractAttributeContainer() = default;
    virtual void move(size_t from, size_t to) {};
    virtual void resize(size_t) = 0;
    virtual void clear() = 0;
    virtual void rollback() = 0;
    virtual void begin_protect() = 0;
    virtual void end_protect() = 0;
};


template <typename T>
struct AttributeCollection : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    // In the preallocated model this sets the storage capacity: it is called
    // (single-threaded) at init / consolidation with the reserved size. It is
    // grow-only so live data below `s` is never dropped. During operations the
    // storage is never resized -- operations only fail when they run out of the
    // preallocated slots.
    void resize(size_t s) override
    {
        if (s > m_attributes.size()) m_attributes.resize(s);
    }
    void clear() override { m_attributes.clear(); }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording.local()) m_rollback_list.local()[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list.local()) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list.local().clear();
        recording.local() = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list.local().clear();
        recording.local() = false;
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    const T& operator[](size_t i) const { return at(i); }

    T& operator[](size_t i)
    {
        if (recording.local()) {
            m_rollback_list.local().emplace(i, m_attributes[i]);
        }
        return m_attributes[i];
    }


    size_t size() const { return m_attributes.size(); }
    wmtk::threading::enumerable_thread_specific<std::unordered_map<size_t, T>> m_rollback_list;
    // Plain preallocated storage: never grows during operations.
    std::vector<T> m_attributes;
    wmtk::threading::enumerable_thread_specific<bool> recording{false};
};
} // namespace wmtk
