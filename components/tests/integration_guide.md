# Guide for Using Integration Test

When you want to start a new integration test, then you need to have following a few steps to set up the test.

- First, write a JSON file under the folder `/data/unit_test/`, you can refer to those existing JSON files to learn how to write it.
- Second, you need the model to run your integration test. Put your model under the folder `/data/unit_test/meshes`.
- Third, after you have done the steps above, you should add the JSON file's name to `/components/tests/integration_test_list.txt`. 
- Fourth, for the first time you run your integration test, you should add a star `*` before your JSON file's name in the `/components/tests/integration_test_list.txt`. With the mark `*`, `integraion_test.cpp` will generate the original performance of your code. After that, you should remove the mark `*`. In the later integration test, `integraion_test.cpp` will compare the result with your original performance to make sure your new version code's performance always consistent with the original one.

NOTE: you can have a look at the `integrations_test.cpp` to get more details if you want.