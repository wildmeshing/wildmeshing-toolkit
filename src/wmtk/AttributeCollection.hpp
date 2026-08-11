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

/**
 * @brief Several attribute collections for the same simplex type, behind one container.
 *
 * A mesh exposes exactly one `p_vertex_attrs` / `p_face_attrs` / ... slot, so a class hierarchy
 * that wants to split its attributes -- shared fields on a base class, application-specific
 * fields on the derived one -- has nowhere to register the second collection. Point the slot at
 * one of these instead and register both.
 *
 * Splitting rather than unioning matters because an `AttributeCollection<T>` is a member: a
 * derived class cannot change its type, and `AttributeCollection<Derived>` is not usable as an
 * `AttributeCollection<Base>` (the storage is a `std::vector<Derived>`, so it has the wrong
 * stride). Without this, every application pays for every other application's fields on every
 * simplex.
 *
 * The children keep their own rollback lists and are protected and rolled back independently,
 * which is exactly right: `AttributeCollection::operator[]` records the rollback entry, so a
 * write to an extras collection inside an operation is undone on failure just like a write to
 * the shared one. Order of registration is the order of forwarding; no child observes another.
 *
 * Non-owning: the collections outlive the group by being members of the same objects.
 */
class AttributeContainerGroup : public AbstractAttributeContainer
{
public:
    void add(AbstractAttributeContainer* c) { m_children.push_back(c); }

    void move(size_t from, size_t to) override
    {
        for (auto* c : m_children) c->move(from, to);
    }
    void resize(size_t s) override
    {
        for (auto* c : m_children) c->resize(s);
    }
    void clear() override
    {
        for (auto* c : m_children) c->clear();
    }
    void rollback() override
    {
        for (auto* c : m_children) c->rollback();
    }
    void begin_protect() override
    {
        for (auto* c : m_children) c->begin_protect();
    }
    void end_protect() override
    {
        for (auto* c : m_children) c->end_protect();
    }

private:
    std::vector<AbstractAttributeContainer*> m_children;
};
} // namespace wmtk
