# Applications

The applications are executables using the WMTK toolkit and its components. There is no general API for the applications, but if integration tests should be used, some common API features are necessary. Each component must be documented individually.

## Integration Tests

### Input json requirements
An application must be able to take a JSON settings file as input, e.g., `-j app_test.json`. This settings file must also contain a path to write statistics in a JSON file. These statistics are used for the integration test later on. The statistics JSON must also contain an entry named `"input"` which contains the settings JSON. The input directory need to contain a `input_directory_tag` to specify relative paths. Following the example from before that means it must contain:

```
"input" : { ... content of app_test.json ... }
```

### Configuration

The application must contain a configuration file for the integration test (e.g., `app_test_config.json`), that contains

```json
{
    "test_directory": "dir",
    "tests": [
        "....json"
    ],
    "input_tag": "input",
    "oracle_tag": "...",
    "input_directory_tag": "...",
    "checks": []
}
```

`test_directory` is the folder  that contains the json files for the test (empty if it is the root directory), relative to the main [data directory (more later)](#cmake).

`tests` is the list of jsons you want to run for the test.

`input_tag` is the tag used in the statistics output to point to the input json (usually just `input`).

`oracle_tag` is the tag used to specify the statistic file.

`input_directory_tag` the tag used to specify the relative path for the different files.

`checks` paths in the json to check reference with oracle. if absent, it checks everything. if empty list it just runs the json without verifying.

### CMake

To register an integration test you need to add
```cmake
wmtk_register_integration_test(<app_exec_name>
    <path_to_test_config>
    <github_repo_with_data>
    <git_hash>)
```
`app_exec_name` is the name of the executable of your application.

`path_to_test_config` this is the name of the [config file](#configuration).

`github_repo_with_data` name of the repo that contains data. This is the root of the data for the tests. Use `test_directory` in the [config file](#configuration) to select a subfolder.

`git_hash` the commit hash.


### Running the test

Go in the build folder and run

```bash
python ../applications/integration_test.py
```