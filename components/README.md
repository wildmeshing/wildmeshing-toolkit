# WMTK Components

In the toolkit, we distinguish between _applications_ and _components_. Applications are used for prototyping. This is usually your entry point when writing new code. Once an application is fully implemented, it may become a component. Components are maintained as part of the toolkit and continuously tested in CI. We want to keep a high standard for our components, meaning the code should be cleaned up, documented, and tested! While unit tests are great and should also be part of every component, we emphasize integration tests to ensure that the entire application continues to work in the future, when dependencies or core functionality might change.

## Integration Tests

Each component should be accompanied by integration tests. One integration test is a full application run, specified by a parameter JSON file. All JSON files inside `data/integration_tests` are automatically executed in the CI.

The folder `data/integration_tests` is automatically downloaded from the [Data2 Repository](https://github.com/wildmeshing/data2) whenever the toolkit is compiled. To add a new integration test, add your JSON file to the [Data2 Repository](https://github.com/wildmeshing/data2) in the folder `integration_tests`. Also make sure that the input data is available. If possible, please reuse models that are already available in the `models` folder. If you need an input that is not available there, also push that input along with your JSON file. Once the data repository is updated, you also need to update the `GIT_TAG` inside `cmake/wmtk_data`.

An integration test is considered successful if it terminates without throwing an exception. Therefore, please make sure that inside your component, exceptions are thrown if the desired outcome is not reached. For example, if your component minimizes an energy, throw an exception (use `log_and_throw_error()`) if the optimization did not converge. You can also add a `bool` to your JSON spec file (`..._spec.json`) to toggle that check. As an example, the _tetwild_ component contains the "throw_on_fail" entry in the JSON spec file that does exactly that.
