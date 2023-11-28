# Operations
This toolkit schedules modifications on meshes, through a formal
concept of **operations** that can be ordered and executed. Within this toolkit
each mesh algorithm is specified by how it schedules and performs different
operations, which are all encapsulated in the `Opeartion` class.

Conceptually an operation has three main components:
* The manipulations it should perform when executed, implemented in a`execute()` member function.
* [Invariants](#Invariants) that specify the preconditions and postconditions for whether an operation is feasible or whether the result is desirable, implemented in `before()` and `after()` member functions. 
* [Priorities](#Priorities) for how to schedule the operation, implemented with a `priority()` member function.



## TupleOperation
Although our primitive operations like edge splits and edge collapses are local
operations implemented in terms of input/output simplices, the base operation
definition _explicitly_ does not have a concept of a tuple built in. This is to
allow for generic (potentially global) operations. When operations are
specified on a particular simplex we have a derived class called
`TupleOperation` that implements these concepts. The use of `Tuple` instead of
`Simplex` is to enforce a concept of orientation with each of these operations.

Each `TupleOperation` is defined in terms of a particular `PrimitiveType`,
to indicate how the input `Tuple` will be interpretted / what sort of
simplex the ooperation is implemented with respect to. 
(In general we could
specify this as a primitive rather than a
simplex to include halfedges or whatnot, but this is not planned thusfar.)

A `TupleOperation` has a default specification for its `before()` and `after()`
using the `Invariant` system, which allows users to attach arbitrary pre/post
conditions to any particular `TupleOperation`. This structure allows for users
to configure global constraints on operations like "no operation should occur
on edge lengthe beyond this theshold" or "no operation should be accepted if it
inverts the area of a triangle".

It (currently) constructs an operation assuming the following constructor
prototype: ``` OpType(Mesh& m, Tuple& t, OperationSettings<OpType>& op); ```
(In the future this third parameter could be changed to something like the
[factory](#Factories) or to add a fourth argument to pass in the
[invariants](#Invariants) directly).

### OperationSettings
The `wmtk` namepsace has a `OperationSettings<T>` class that is intended as a
default location for settings/parameters shared between different intances.
The choice of specializing a templated class exists to make sure users
explicitly construct settings for each of their operations rather than choosing
to share them.

#### Current Usage
Currently users should make sure that each class specifies an
`InvariantCollection` somewhere in the settings object, which is accessed in
the constructor for `TupleOperation`. This should be removed at some point.


### Factories
Within this toolkit we generally do not construct tuple operations directly,
but rather utilize the [Factory method
pattern](https://en.wikipedia.org/wiki/Factory_method_pattern) to create them.
After a user has constructed settings for an operation (ignoring which Mesh /
simplex on a mesh the operation should occur on), they can construct a factory,
and finally create operations. If we ignore the workflow of the scheduler we
can create operations by the following:

``` OperationSettings<OpType> settings(...); // configure settings more...
OperationFactory<OpType> factory(std::move(settings)); Mesh m; for(const auto&
t: m.get_all(PrimitiveType::Edge) {// assuming OpType::primitive_type() == Edge
std::unique_ptr<Operation> op = factory.create(m,t); (*op)(); // run the
operation } ```

The `OperationFactory<OpType>` class (will) store two members
1. The [settings](#OperationSettings) shared by each operation constructed.
2. The [invariants](#Invariants) used for each operation constructed (TODO)
   (currently the settings are responsible for holding the invariants, but in
   the future this should change).

#### Factories and Invariants
The desired mechanism (which is not implemented thusfar) is for the factory to
take responsibility for constructing invariants.
Each `OpeartionSettings<OpType>` object is responsible for implementing a
function for constructing an `InvariantCollection` that the
`OperationFactory<OpType>` takes as a member. Any futher invariants should then
be added to the `InvariantCollection` member in the `OperationFactory<OpType>`.


### Invariants The invariant system assumes a single Tuple to compute a
precondition (`before()`) and assumes a collection of simplices to indicate
which simplices were modified to evaluate if postconditions are satisfied by
each invariant (`after()`). (TODO mtao: I think we need some way of
constraining Operations on a specific primitive to specific types of
invariants).

The expected interface is that users will specify which settings in their
per-operation `OperationSettings<OpType>` class, and in the initialization of
those settings the environment will construct invariants and store them. This
specification is purposely underdetermined as currently most users are using an
`initialize_invariants` function that stores an `invariantCollection` in the
settings object, but it should be noted that in the future hte desired
interface will converge towards something like storing an `InvariantCollection`
in each factory to guarantee safe storage of invariants during the lifetime of
an Operation.

For futher details look at the [Invariant](../invariants) page.

