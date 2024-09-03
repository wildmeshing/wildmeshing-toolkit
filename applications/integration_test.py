import unittest
import glob
import os
import sys
import json
import subprocess
import tempfile


class IntegrationTest(unittest.TestCase):
    BINARY_FOLDER = ""

    def test_run_all(self):
        this_dir = os.path.dirname(os.path.realpath(__file__))
        print('Running all integration tests in', this_dir)

        for config_file in glob.glob(os.path.join(this_dir,'**/*.json'), recursive=True):
            if "test_output" in config_file:
                continue
            with open(config_file) as f:
                config = json.load(f)

            test_folder = os.path.join(this_dir, "..", config["data_folder"])

            input_tag = config["input_tag"]
            oracle_tag = config["oracle_tag"]
            root_tag = config["root_tag"]
            checks = [] if "checks" not in config else config["checks"]

            executable = config["executable"]
            executable = os.path.join(this_dir, "..", IntegrationTest.BINARY_FOLDER, executable)

            print("Running", executable)

            for test_file_name in config["tests"]:
                print("Running test ", test_file_name)

                test_file = os.path.join(test_folder, test_file_name)
                self.assertTrue(os.path.exists(test_file))

                with open(test_file) as f:
                    test_oracle = json.load(f)

                input = test_oracle[input_tag].copy()
                input[oracle_tag] = f"{test_file_name}_run.json"

                if root_tag in input:
                    if not os.path.isabs(input[root_tag]):
                        input[root_tag] = os.path.join(test_folder, input[root_tag])
                else:
                    input[root_tag] = test_folder

                with tempfile.NamedTemporaryFile() as tmp:
                    with open(tmp.name, "w") as ftmp:
                        json.dump(input, ftmp)

                    res = subprocess.run([executable, "-j", tmp.name])

                    self.assertEqual(res.returncode, 0)

                    with open(input[oracle_tag]) as f:
                        result = json.load(f)

                    for check in checks:
                        self.assertTrue(check(result[check], test_oracle[check]))

                    if len(checks) == 0:
                        for k in test_oracle:
                            if k == input_tag:
                                continue

                            self.assertTrue(k in result)
                            self.assertEqual(result[k], test_oracle[k])

                        # self.assertEqual(result, test_oracle)



        self.assertTrue(True)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        IntegrationTest.BINARY_FOLDER = sys.argv.pop()

    unittest.main()