# Guide for Using Integration Test

1. Write a JSON file under the folder `/data/unit_test/`, you can refer to those existing JSON files to learn how to write it.
2. Put the mesh for your test under the folder `/data/unit_test/meshes`.
3. Add the JSON file's name to `/components/tests/integration_test.cpp`, i.e. add the line `WMTK_INTEGRATION("YOUR TEST NAME", false);`.
4. The first time you run your integration test, set the `DO_VALIDATION` parameter `false`, i.e. `WMTK_INTEGRATION("YOUR TEST NAME", false);`. The integration test will then add the test output (number of simplices) to your JSON. Afterward, set the parameter to `true`. Now, the integration test will compare the result with your original output and fail if the two do not match.
5. Push the JSON and meshes to the [wmtk data repo](https://github.com/wildmeshing/data).

Look at `integrations_test.cpp` for more details.
