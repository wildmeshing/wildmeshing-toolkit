# Attribute Classes

We describe the structure of attributes and their usage. All access to attributes is guarded by several layers, where each one fulfills a different purpose. From top to bottom the layers are:

1. The `Mesh` contains an `AttributeManager`.
2. The `AttributeManager` contains type specific `TypedAttributeManager`.
3. Every `TypedAttributeManager` stores a vector of `CachingAttribute`s and a map from their names to their handles.
4. The `CachingAttributes` provide utility to revert changes on `Attribute`s.
5. The `Attribute` class is a wrapper around the raw attribute data that provides read and write functionality.
6. For referencing and accessing attributes, we use handles and accessors.

In the following, we provide a summary of the most important concepts that were used in all attribute-related classes.

## Attributes

All properties of a mesh (including its topology) are stored in attributes. At its core, attributes are vectors:

```C++
template <typename T>
class Attribute {
    std::vector<T> m_data;
    Eigen::Matrix<T>::MapType vector_attribute(int64_t index);
    T& scalar_attribute(int64_t index);
};
```

Attributes have a `dimension`. If the `dimension = 1` then the attribute is a scalar attribute, otherwise, it is a vector attribute. This distinction is necessary for accessing the attribute. While the former case returns just a scalar, the other one returns an `Eigen::Map` (similar to a `Eigen::Vector` but without creating a copy).

Changes on Attributes can be reverted through the `CachingAttribute` that inherits from `Attribute`. The `CachingAttribute` holds scopes that can be pushed and popped:

```C++
template <typename T>
class CachingAttribute : public Attribute<T>
{
    void push_scope();
    void pop_scope(bool preserve_changes);
}
```

The scopes work as a recording mechanism. When a new scope is pushed with `push_scope()` all changes to the attribute are tracked. When the scope is popped, the changes can either be accepted with `pop_scope(true)` or discarded with `pop_scope(false)`.

## Managers

All attributes of the same type are stored in a `TypedAttributeManager`, along with a `std::map` that provides `AttributeHandle`s based on the attribute's name. The manager also deals with the registration and removal of attributes. Note that while attributes can be removed, the vector of attributes is not resized. Otherwise, `AttributeHandle`s would point to the wrong data. If the vector should be resized, e.g., for serialization/writing to file, `clear_dead_attributes()` needs to be called.

```C++
template <typename T>
class TypedAttributeManager
{
    AttributeHandle register_attribute(/*...*/);
    void remove_attribute(const AttributeHandle& attribute);

    std::map<std::string, AttributeHandle> m_handles;
    std::vector<std::unique_ptr<CachingAttribute<T>>> m_attributes;
}
```

The `AttributeManager` that is a container for all `TypedAttributeManager`.

```C++
class AttributeManager
{
    std::vector<TypedAttributeManager<char>> m_char_attributes;
    std::vector<TypedAttributeManager<int64_t>> m_long_attributes;
    std::vector<TypedAttributeManager<double>> m_double_attributes;
    std::vector<TypedAttributeManager<Rational>> m_rational_attributes;
}
```

The `Mesh` contains an `AttributeManager`.

## Handles

The `TypedAttributeManager` holds a map from names to `AttributeHandle`s. They are just glorified indices:

```C++
class AttributeHandle
{
    int64_t m_index = -1;
}
```

> Daniel: I am not sure why we have these handles. I think they are only used within the TypedAttributeManager so we could merge them into one class, deleting AttributeHandle and only keeping the TypedAttributeHandle.

The `TypedAttributeHandle` is the most basic handle used outside the `namespace attribute`. It consists of an `AttributeHandle` and a `PrimitiveType`. Together with the attribute's `Type`, an attribute is uniquely defined for a single `Mesh`.

```C++
template <typename T>
class TypedAttributeHandle
{
    using Type = T;
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type;
}
```

The `TypedAttributeHandle` contains a template parameter. To avoid using templates all over the place, there exists the `std::variant` `HandleVariant` in the non-templated `MeshAttributeHandle`. It also contains a reference to its `Mesh`.

```C++
class MeshAttributeHandle
{
    using HandleVariant = std::variant<TypedAttributeHandle<char>,TypedAttributeHandle<int64_t>,
        TypedAttributeHandle<double>,TypedAttributeHandle<wmtk::Rational>>;
    HandleVariant& handle() { return m_handle; }

    Mesh* m_mesh = nullptr;
    HandleVariant m_handle;
}
```

The reference to the `Mesh` makes the handle unique within a multimesh, where multiple meshes might have the "same" attribute, e.g., vertex positions. The `MeshAttributeHandle` is almost always the handle that should be used. The visitor pattern must be used if someone wants to access the `Type` of an attribute from the `MeshAttributeHandle`.

```C++
MeshAttributeHandle attr;
std::visit([](auto&& val) {
        //get type T of the attribute
        using T = std::decay_t<decltype(val)>;
    },
    attr.handle());
```

> Daniel: I wrote this example without testing it. This should be done properly.

## Accessors

> Daniel: Do we need accessors when we have TypedAttributeHandle? Can't we just use these as accessors?
