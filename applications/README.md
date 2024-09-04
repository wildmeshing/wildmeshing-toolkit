# Applications

The applications are executables using the WMTK toolkit and its components. There is no general API for the applications, but if integration tests should be used, some common API features are necessary. Each component must be documented individually.

## Integration Tests

An application must be able to take a JSON settings file as input, e.g., `-j app_test.json`. This settings file must also contain a path to write statistics in a JSON file. These statistics are used for the integration test later on. The statistics JSON must also contain an entry named `"input"` which contains the settings JSON. Following the example from before that means it must contain:

```
"input" : { ... content of app_test.json ... }
```
